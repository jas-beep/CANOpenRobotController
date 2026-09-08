#include "ForcePlateMaster.h"
#include "State.h"

class InitState : public State {
    public:
    ForcePlateMaster *robot;
    InitState(ForcePlateMaster *robot, const char *name = "");
    void entry(void);
    void during(void) {}
    void exit(void) {}
};
