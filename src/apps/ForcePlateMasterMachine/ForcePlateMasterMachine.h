#ifndef FORCEPLATEMASTER_H
#define FORCEPLATEMASTER_H

#include "ForcePlateMaster.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

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
        void hwStateUpdate();

        ForcePlateMaster *robot() { return static_cast<ForcePlateMaster*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)
        std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server

};

#endif // FORCEPLATEMASTER_H