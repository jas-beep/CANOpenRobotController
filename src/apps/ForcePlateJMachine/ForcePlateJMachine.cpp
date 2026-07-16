#include "ForcePlateJMachine.h"

using namespace std;

bool endCalib(StateMachine & sm) {
    return (sm.state<CalibState>("CalibState"))->isCalibDone();
}

bool goToCalib(StateMachine & SM)
{
    ForcePlateJMachine & sm = static_cast<ForcePlateJMachine &>(SM); //Cast to specific StateMachine type

    if ( (sm.robot()->keyboard->getNb()==4) )
        return true;

    if (sm.UIserver->isCmd("GTCS") ) 
    {
        sm.UIserver->sendCmd(string("OK"));
        return true;
    }

    return false;

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

bool goToPerCornerCalib(StateMachine & SM) {
    ForcePlateJMachine & sm = static_cast<ForcePlateJMachine &>(SM); //Cast to specific StateMachine type

    //keyboard 
    if ( (sm.robot()->keyboard->getNb()==2) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GTPCC") ) {
        sm.UIserver->sendCmd(string("OK"));
        return true;
    }

    //Otherwise false
    return false;
}

bool goToCOPCalib(StateMachine & SM) {
    ForcePlateJMachine & sm = static_cast<ForcePlateJMachine &>(SM); //Cast to specific StateMachine type

    //keyboard 
    if ( (sm.robot()->keyboard->getNb()==3) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GTCC") ) {
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

bool endWeightedCalib(StateMachine & SM) {
    ForcePlateJMachine & sm = (ForcePlateJMachine &)SM; //Cast to specific StateMachine type
    return (sm.state<SetScale>("SetScale"))->isWeightedCalibDone();
}

bool endWeightedCalibPerCorner(StateMachine & SM) {
    ForcePlateJMachine & sm = (ForcePlateJMachine &)SM; //Cast to specific StateMachine type
    return (sm.state<SetScalePerCorner>("SetScalePerCorner"))->isPerCornerCalibDone();
}

bool endCOPCalib(StateMachine & SM) {
    ForcePlateJMachine & sm = (ForcePlateJMachine &)SM; //Cast to specific StateMachine type
    return (sm.state<CalibrateCOP>("CalibrateCOP"))->isCalibDone(); //not uniquely named but defined in ForcePlateJStates.h
}


ForcePlateJMachine::ForcePlateJMachine() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<ForcePlate>("ForcePlate"));

    //Create state instances and add to the State Machine
    addState("StandbyState", std::make_shared<StandbyState>(robot()));
    addState("CalibState", std::make_shared<CalibState>(robot()));
    addState("SetScale", std::make_shared<SetScale>(robot(), 4.2069));                     //change weight here
    addState("SetScalePerCorner", std::make_shared<SetScalePerCorner>(robot(), 4.2069));   //change weight here
    addState("CalibrateCOP", std::make_shared<CalibrateCOP>(robot(), 4.2069));             //change weight here


    //Define transitions between states
    addTransition("CalibState", &endCalib, "StandbyState");
    addTransitionFromAny(&standby, "StandbyState");
    addTransition("StandbyState", &goToNextState, "SetScale");                      // 1 for center calibration
    addTransition("SetScale", &endWeightedCalib, "StandbyState");
    addTransition("StandbyState", &goToPerCornerCalib, "SetScalePerCorner");        // 2 for per corner calibration
    addTransition("SetScalePerCorner", &endWeightedCalibPerCorner, "StandbyState");
    addTransition("StandbyState", &goToCOPCalib, "CalibrateCOP");  
    addTransition("CalibrateCOP", &endCOPCalib, "StandbyState");                    // 3 for CoP calibration
    addTransition("StandbyState", &goToCalib, "CalibState");                        // 4 to restart offset

    // Initialize the state machine with first state of the designed state machine (taring)
    // Maybe rename CalibState to TareState to avoid confusion, also setScale
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

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream logFileName;
    //Put time in name for debugging and to avoid overwriting previous logs
    logFileName << "logs/ForcePlateJMachine_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    if(robot()->initialise()) {
        logHelper.initLogger("ForcePlateJMachineLog", logFileName.str(), LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getStrainReadings(), "F");
        logHelper.add(robot()->getCOP(), "CoP");
        UIserver = std::make_shared<FLNLHelper>("192.168.7.2");
        UIserver->registerState(runningTime());
        //WARNING: cannot take a fixed size Vector (e.g. Vector3d)
        UIserver->registerState(robot()->getStrainReadings());
        UIserver->registerState(robot()->getCOP());
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
