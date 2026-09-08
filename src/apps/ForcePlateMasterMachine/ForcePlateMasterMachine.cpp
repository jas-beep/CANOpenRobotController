#include "ForcePlateMasterMachine.h"

bool isAPressed(StateMachine & SM) {
    ForcePlateMasterMachine & sm = static_cast<ForcePlateMasterMachine &>(SM); //Cast to specific StateMachine type
    return sm.robot()->keyboard->getA();
}
bool isSPressed(StateMachine & SM) {
    ForcePlateMasterMachine & sm = static_cast<ForcePlateMasterMachine &>(SM); //Cast to specific StateMachine type
    return sm.robot()->keyboard->getS();
}

ForcePlateMasterMachine::ForcePlateMasterMachine() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<ForcePlateMaster>("ForcePlateMaster", "forceplatemaster_params.yaml"));

    //Create state instances and add to the State Machine
    addState("InitState", std::make_shared<InitState>(robot()));
    addState("IdleState", std::make_shared<IdleState>(robot()));
    addState("RecordState", std::make_shared<RecordState>(robot()));

    //Define transitions between states
    addTransitionFromAny(&isAPressed, "IdleState");
    addTransitionFromAny(&isSPressed, "RecordState");
    addTransition("RecordState", &isSPressed, "IdleState");

    // Initialize the state machine with first state of the designed state machine (taring)
    setInitState("InitState");
}

void ForcePlateMasterMachine::init() {
    robot()->initialise();

    logHelper.initLogger("ForcePlateMasterLog", "logs/master.csv", LogFormat::CSV, true);
    logHelper.add(runningTime(), "Time (s)");
    for (size_t i = 0; i < robot()->numPlates(); i++) {
        logHelper.add(robot()->getForces(i), "Plate" + std::to_string(i) + "_Forces");
        logHelper.add(robot()->getCoP(i), "Plate" + std::to_string(i) + "_CoP");
    }
}

void ForcePlateMasterMachine::end() {
    StateMachine::end();    
}

ForcePlateMasterMachine::~ForcePlateMasterMachine() {
    // Destructor implementation (if needed)
}