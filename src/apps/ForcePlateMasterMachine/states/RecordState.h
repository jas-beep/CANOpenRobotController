#include "ForcePlateMaster.h"
#include "State.h"

class RecordState : public State {
    public:
    ForcePlateMaster *robot;
    RecordState(ForcePlateMaster *robot, const char *name = "");
    void entry(void);
    void during(void);
    void exit(void);
};
