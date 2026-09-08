#ifndef FORCEPLATEMASTER_H
#define FORCEPLATEMASTER_H

#include "ForcePlateMaster.h"
#include "StateMachine.h"

//State Classes (mimicking Justin's old code)
#include "InitState.h"
#include "IdleState.h"
#include "RecordState.h"

class ForcePlateMasterMachine : public StateMachine {
    public:
        ForcePlateMasterMachine();
        ~ForcePlateMasterMachine();
        void init();
        void end();

        ForcePlateMaster *robot() { return static_cast<ForcePlateMaster*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)
};

#endif // FORCEPLATEMASTER_H