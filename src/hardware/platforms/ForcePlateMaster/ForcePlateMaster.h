#ifndef FORCEPLATEMASTER_ROBOT_H
#define FORCEPLATEMASTER_ROBOT_H

#include "Robot.h"
#include "Keyboard.h"
#include "ForcePlateReceiver.h"
#include "ForcePlate.h"

class ForcePlateMaster : public Robot {
    private:
        std::vector<int> plateIDs; //loaded from yaml config file
        std::vector<ForcePlateReceiver*> plates; 
        TPDO *cmdTPDO;
        ForcePlateCommand cmdDATA = NONE;

    public:
        Keyboard *keyboard;
        ForcePlateMaster(std::string robot_name="", std::string yaml_config_file="");

        bool initialiseJoints() {return true;};
        bool initialiseInputs();
        bool initialiseNetwork() {return true;}
        bool configureMasterPDOs();
        bool loadParametersFromYAML(YAML::Node params) override;

        void startStreaming() {cmdDATA = STARTSTREAM;};
        void stopStreaming() {cmdDATA = STOP;};
        void triggerCOPCalibration() {cmdDATA = COP_CALIB;};  //!< Broadcast COP calibration trigger to all plates. Caller is responsible for clearing it (see clearCommand()) once the CAN TPDO has had time to transmit it, otherwise every plate will re-enter CalibrateCOP as soon as they next return to StandbyState.
        void clearCommand() {cmdDATA = NONE;};
        void triggerCOPCalibration(size_t plateIdx) {plates[plateIdx]->sendCalibCommand(COP_CALIB);}; 
        void advancePlacement(size_t plateIdx) {plates[plateIdx]->sendCalibCommand(ADVANCE_PLACEMENT);};
        void clearCalibCommand(size_t plateIdx) {plates[plateIdx]->sendCalibCommand(NONE);};
        
        Eigen::VectorXf &getForces(int plateIdx) {return plates[plateIdx]->getForces();}
        Eigen::VectorXf &getCoP(int plateIdx) {return plates[plateIdx]->getCoP();}
        float getSummedForce(int plateIdx) {return plates[plateIdx]->getForces().sum();}
        Eigen::VectorXf &getGlobalCoP();
        size_t numPlates() {return plates.size();}

    private:
        Eigen::VectorXf globalCoP = Eigen::VectorXf::Zero(2);
};

#endif