#!/bin/bash

#Passive CAN diagnostics for the "bus goes silent while the apps keep running" freeze.
#Runs on the laptop and on the PocketBeagle plates (needs only can-utils + iproute2).
#NEVER sends frames and NEVER changes the interface state - safe to run while the bus is frozen.
#
#Usage: ./capture_can_freeze.sh start    [iface]   start background recorders + freeze watchdog (do this BEFORE launching the app)
#       ./capture_can_freeze.sh status              is everything still running? has the watchdog fired?
#       ./capture_can_freeze.sh snapshot [iface]   capture evidence now (~5 s, read-only) - the watchdog runs this for you on a freeze
#       ./capture_can_freeze.sh stop     [iface]   stop the recorders and print the last-frame-per-ID report
#       ./capture_can_freeze.sh report             re-print the report for the current capture
#
#Unattended: 'start' also launches a watchdog. Once it has seen traffic, if RX or TX packets stop rising for SILENCE_S
#seconds it takes a snapshot by itself (and re-arms if traffic resumes). Nobody has to be watching when the freeze happens.
#Note it will also fire when you stop the app normally at the end of a run - ignore that one.
#
#Env:   OUT_ROOT     where captures go                                (default ~/can_freeze)
#       POLL_S       interface state poll period in s                 (default 1)
#       SILENCE_S    seconds of stalled RX/TX before auto-snapshot    (default 3)
#       APP_PATTERN  regex matching the app's processes/threads       (default 'APP|corc|canopend')
#
#Output (in $OUT_ROOT/<host>_<date>/): can.log (candump -L, all data + error frames, epoch timestamps), state.log (ip -details -statistics
#once per POLL_S), watch.log (watchdog events), meta.txt, snapshot_*.txt
#Clock: timestamps use the local clock - a PocketBeagle has no RTC, so sync it first (script/syncBBtime.sh).

MODE="${1:-}"
IFACE="${2:-can0}"
OUT_ROOT="${OUT_ROOT:-$HOME/can_freeze}"
POLL_S="${POLL_S:-1}"
SILENCE_S="${SILENCE_S:-3}"
APP_PATTERN="${APP_PATTERN:-APP|corc|canopend}"
CURRENT_FILE="$OUT_ROOT/.current"
SELF="$(readlink -f "$0")"

die() { echo "$*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "'$1' not found (sudo apt install can-utils iproute2)"; }
alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }
current_dir() { [ -f "$CURRENT_FILE" ] || die "No capture found - run '$0 start' first"; cat "$CURRENT_FILE"; }

counters() { ip -s link show "$IFACE" | awk '/RX:/{getline; rx=$2} /TX:/{getline; tx=$2} END{print rx+0, tx+0}'; }
can_state() { ip -details link show "$IFACE" | grep -o 'can state [A-Z-]*' | head -1; }

do_start() {
    need candump; need ip
    ip link show "$IFACE" >/dev/null 2>&1 || die "Interface $IFACE not found"
    if [ -f "$CURRENT_FILE" ]; then
        local prev; prev="$(cat "$CURRENT_FILE")"
        alive "$prev/candump.pid" && die "A capture is already running ($prev). Run '$0 stop' first."
    fi

    local out="$OUT_ROOT/$(hostname)_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$out" && echo "$out" > "$CURRENT_FILE"

    {
        echo "host  : $(hostname)"
        echo "iface : $IFACE"
        echo "start : $(date +'%F %T.%N %z')  epoch=$(date +%s.%N)"
        uname -a
        ip -details link show "$IFACE"
    } > "$out/meta.txt" 2>&1

    #nohup so the recorders survive an ssh drop; candump flushes per frame, so a hard crash loses ~nothing
    nohup candump -L "$IFACE,0:0,#FFFFFFFF" > "$out/can.log" 2> "$out/candump.err" &
    echo $! > "$out/candump.pid"
    nohup bash -c "while true; do date +'%T.%N'; ip -details -statistics link show '$IFACE'; sleep $POLL_S; done" \
        > "$out/state.log" 2>&1 &
    echo $! > "$out/state.pid"
    nohup "$SELF" _watch "$IFACE" > "$out/watch.log" 2>&1 &
    echo $! > "$out/watch.pid"

    echo "Recording $IFACE on $(hostname) into $out"
    echo "Watchdog armed: auto-snapshot if RX/TX stall for ${SILENCE_S}s (arms itself once traffic is seen)."
    echo "Start your app now and walk away. Afterwards:  $0 status   then   $0 stop $IFACE"
}

#Unattended trigger. "Healthy" = RX and TX packet counters both rose in the last second. After the first healthy second
#the watchdog is armed; SILENCE_S consecutive unhealthy seconds = freeze -> snapshot, then re-arm when traffic resumes.
do_watch() {
    local out; out="$(current_dir)" || exit 1
    local armed=0 bad=0 rx tx prx ptx
    read -r prx ptx <<< "$(counters)"
    while sleep 1; do
        read -r rx tx <<< "$(counters)"
        if [ "$rx" -gt "$prx" ] && [ "$tx" -gt "$ptx" ]; then
            armed=1; bad=0
        elif [ "$armed" = 1 ]; then
            bad=$((bad + 1))
        fi
        prx=$rx; ptx=$tx
        if [ "$armed" = 1 ] && [ "$bad" -ge "$SILENCE_S" ]; then
            echo "$(date +'%F %T') FREEZE: RX/TX stalled for ${bad}s (rx=$rx tx=$tx, $(can_state)) - taking snapshot"
            "$SELF" snapshot "$IFACE" > /dev/null 2>> "$out/watch.log"   #snapshot also tees to its own snapshot_*.txt
            echo "$(date +'%F %T') snapshot done"
            armed=0; bad=0
            read -r prx ptx <<< "$(counters)"
        fi
    done
}

do_status() {
    local out; out="$(current_dir)" || exit 1
    echo "capture dir: $out"
    for f in candump state watch; do
        if alive "$out/$f.pid"; then echo "$f: running"; else echo "$f: NOT running"; fi
    done
    echo "$IFACE: $(can_state)"
    echo "--- watch.log"; cat "$out/watch.log" 2>/dev/null
    echo "--- snapshots"; ls "$out"/snapshot_*.txt 2>/dev/null || echo "(none - no freeze detected yet)"
}

do_report() {
    local out; out="$(current_dir)" || exit 1
    local log="$out/can.log"
    [ -s "$log" ] || { echo "$log is empty - nothing was recorded"; return; }

    echo "===== last frame per CAN ID (quietest first: IDs that went silent EARLIEST are at the top) ====="
    echo "expected IDs: 3E0 master cmd | plate0: 3E1/3E2 forces, 3E3 CoP, 3E4 calib cmd, 3E5 status | plate1 (+0x10): 3F1..3F5"
    echo "              080 SYNC | 7xx heartbeat (master nodeId 80 -> 750) | ERR:xxxxxxxx = error frame"
    awk '
        { ts=$1; gsub(/[()]/, "", ts); ts+=0
          split($3, a, "#"); id=a[1]
          if (length(id)==8 && substr(id,1,1)=="2") id="ERR:" id
          last[id]=ts; cnt[id]++
          if (first==0) first=ts
          if (ts>end) end=ts }
        END {
            printf "log spans %.1f s, ends at epoch %.6f\n", end-first, end
            printf "%10s  %-14s %s\n", "s-before-end", "CAN ID", "frames"
            fflush()
            cmd="sort -rn"
            for (i in last) printf "%10.3f  %-14s %d\n", end-last[i], i, cnt[i] | cmd
            close(cmd)
        }' "$log"
    echo
    echo "Bus load / gap check: if every ID ends within ~0.1 s of each other the whole bus stopped at once (bus-level event);"
    echo "if one node's IDs end earlier, that node stopped first."
}

do_snapshot() {
    local out; out="$(current_dir)" || exit 1
    local snap="$out/snapshot_$(date +%H%M%S).txt"
    {
        echo "===== SNAPSHOT $(date +'%F %T.%N') epoch=$(date +%s.%N) on $(hostname) / $IFACE ====="

        echo; echo "--- interface state (before)"
        ip -details -statistics link show "$IFACE"
        read -r rx1 tx1 <<< "$(counters)"

        echo; echo "--- live listen, 3 s (what THIS node hears right now)"
        timeout 3 candump -L "$IFACE,0:0,#FFFFFFFF" | awk '
            { split($3, a, "#"); id=a[1]
              if (length(id)==8 && substr(id,1,1)=="2") e++; else n++ }
            END { printf "data frames: %d   error frames: %d\n", n+0, e+0 }'
        sleep 2   #counter window = 5 s total
        read -r rx2 tx2 <<< "$(counters)"

        echo; echo "--- verdict inputs"
        echo "state          : $(can_state)"
        echo "RX packets +5s : $((rx2 - rx1))   (0 = this node received nothing)"
        echo "TX packets +5s : $((tx2 - tx1))   (0 = this node completed no transmissions)"
        echo "reading it:"
        echo "  BUS-OFF + TX frozen               -> controller gave up after an error burst (needs restart-ms / check wiring)"
        echo "  RX>0 on a plate, master silent    -> bus is alive; master adapter/USB side is the problem"
        echo "  TX frozen, state OK/ACTIVE        -> node's CAN thread stalled, not the bus"
        echo "  RX and TX both 0, ERROR-PASSIVE   -> nobody is acking this node"

        echo; echo "--- kernel log (CAN / USB), last 30"
        (sudo -n dmesg 2>/dev/null || dmesg 2>&1) | grep -iE 'can[0-9]|gs_usb|c_can|d_can|dcan|bus-?off|usb.*(reset|disconnect|error)' | tail -30

        echo; echo "--- app processes/threads matching '$APP_PATTERN' (STAT D/S/R, WCHAN = where a thread is blocked)"
        ps -eLo pid,tid,stat,pcpu,wchan:24,comm,args | awk -v p="$APP_PATTERN" 'NR==1 || ($0 ~ p && $0 !~ /awk/)' | head -40

        echo; echo "--- last 15 frames recorded"
        tail -n 15 "$out/can.log"

        echo; echo "--- state.log, last poll"
        tail -n 25 "$out/state.log"

        echo
        do_report
    } 2>&1 | tee "$snap"
    echo; echo "Saved: $snap"
    echo "Now (only after saving the above) you can try the recovery test:  sudo ip link set $IFACE down && sudo ip link set $IFACE up"
}

do_stop() {
    local out; out="$(current_dir)" || exit 1
    for f in watch candump state; do   #watchdog first, so it can't fire on the shutdown
        if alive "$out/$f.pid"; then
            pkill -P "$(cat "$out/$f.pid")" 2>/dev/null
            kill "$(cat "$out/$f.pid")" 2>/dev/null
        fi
    done
    echo "Stopped. Files in $out:"
    ls -lh "$out"
    echo
    do_report
}

case "$MODE" in
    start)    do_start ;;
    _watch)   do_watch ;;
    status)   do_status ;;
    snapshot) need candump; need ip; do_snapshot ;;
    stop)     do_stop ;;
    report)   do_report ;;
    *)        sed -n '3,22p' "$0"; exit 1 ;;
esac
