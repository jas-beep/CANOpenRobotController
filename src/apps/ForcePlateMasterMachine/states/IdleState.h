#include "ForcePlateMaster.h"
#include "State.h"

class IdleState : public State {
    public:
    ForcePlateMaster *robot;
    IdleState(ForcePlateMaster *robot, const char *name = "");
    void entry(void);
    void during(void);
    void exit(void) {}

    private:
    int copCalibPulseCountdown = 0;  //!< Cycles left to hold COP_CALIB on the CAN bus before clearing it back to NONE.
    int calibPulseCountdown = 0;
    size_t pulsingPlate = 0;
    size_t activePlate = 0;
};
