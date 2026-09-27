#!/bin/bash

#Set the clock of the PocketBeagles from this laptop (which is NTP-synced).
#A PocketBeagle has no RTC / battery, so its time resets to a stale date at every power-up: run this after each boot,
#BEFORE ./capture_can_freeze.sh start, so timestamps in can.log / CORC.log line up across devices.
#Accuracy is roughly +-0.3 s (ssh connection latency) - fine for lining up events, not for frame-level comparison.
#
#Usage: ./script/syncBBtime.sh [ip ...]      (default: 192.168.7.2 192.168.6.2)
#-t on ssh lets sudo ask for a password if the plate needs one.

SSH_USER="debian"
HOSTS=("$@")
[ ${#HOSTS[@]} -eq 0 ] && HOSTS=(192.168.7.2 192.168.6.2)

for ip in "${HOSTS[@]}"; do
    echo -n "$ip: "
    ping -W 1 -c 1 "$ip" >/dev/null 2>&1 || { echo "no reply, skipping"; continue; }
    before=$(ssh -q -o ConnectTimeout=3 "$SSH_USER@$ip" 'date +"%F %T"')
    ssh -q -t -o ConnectTimeout=3 "$SSH_USER@$ip" "sudo date -u -s @$(date +%s.%N) >/dev/null" \
        || { echo "could not set the time (ssh / sudo?)"; continue; }
    echo "was $before -> now $(ssh -q "$SSH_USER@$ip" 'date +"%F %T"')   (laptop: $(date +'%F %T'))"
done
