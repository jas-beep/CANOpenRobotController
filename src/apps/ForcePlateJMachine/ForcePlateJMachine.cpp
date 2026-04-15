#include "ForcePlateJMachine.h"

using namespace std;

bool endCalib(StateMachine & sm) {
    return (sm.state<CalibState>("CalibState"))->isCalibDone();
}

bool goToNextState(StateMachine & SM) {
    ForcePlateJMachine & sm = static_cast<ForcePlateJMachine &>(SM); //Cast to specific StateMachine type

    //keyboard
    if ( (sm.robot()->keyboard->getNb()==1) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GTNS") ) {
        sm.UIserver->sendCmd(string("OK"));
        return true;
    }

    //Otherwise false
    return false;
}

bool standby(StateMachine & SM) {
    ForcePlateJMachine & sm = (ForcePlateJMachine &)SM; //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getQ()==1) {
        return true;
    }
    return false;
}


ForcePlateJMachine::ForcePlateJMachine() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<ForcePlate>("ForcePlate"));

    //Create state instances and add to the State Machine
    addState("StandbyState", std::make_shared<StandbyState>(robot()));
    addState("CalibState", std::make_shared<CalibState>(robot()));


    //Define transitions between states
    addTransition("CalibState", &endCalib, "StandbyState");
    addTransitionFromAny(&standby, "StandbyState");

    //Initialize the state machine with first state of the designed state machine
    setInitState("CalibState");
}
ForcePlateJMachine::~ForcePlateJMachine() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void ForcePlateJMachine::init() {
    spdlog::debug("ForcePlateJMachine::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("ForcePlateJMachineLog", "logs/ForcePlateJMachine.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getStrainReadings(), "F");
        UIserver = std::make_shared<FLNLHelper>("192.168.7.2");
        UIserver->registerState(runningTime());
        UIserver->registerState(robot()->getStrainReadings());
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}

void ForcePlateJMachine::end() {
    if(running())
        UIserver->closeConnection();
    StateMachine::end();
}


/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void ForcePlateJMachine::hwStateUpdate() {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();
}
