#ifndef FORCEPLATERECEIVER_H
#define FORCEPLATERECEIVER_H

#include "InputDevice.h"
#include "TPDO.h"
#include "RPDO.h"
#include <Eigen/Dense>


class ForcePlateReceiver : public InputDevice {
    public: 
    ForcePlateReceiver(int forceID1, int forceID2, int copID, int calibCmdID);
    bool configureMasterPDOs();
    void updateInput() override {};
    Eigen::VectorXf &getForces() { return forces; };
    Eigen::VectorXf &getCoP() { return cop; };
    void sendCalibCommand(int cmd) { calibCmdData = cmd; };

    private:
    int forceID1, forceID2, copID; //each plate transmits 4 forces and 2 CoP values.
    int calibCmdID; 
    Eigen::VectorXf forces = Eigen::VectorXf::Zero(4); //forces from the force plate
    Eigen::VectorXf cop = Eigen::VectorXf::Zero(2); //
    int calibCmdData = 0; 
    RPDO *rpdoForce1, *rpdoForce2, *rpdoCoP; 
    TPDO *tpdoCalibCmd;
};

#endif
