#ifndef FORCEPLATERECEIVER_H
#define FORCEPLATERECEIVER_H

#include "InputDevice.h"
#include "TPDO.h"
#include "RPDO.h"
#include <Eigen/Dense>


class ForcePlateReceiver : public InputDevice {
    public:
    ForcePlateReceiver(int forceID1, int forceID2, int copID, int calibCmdID, int statusID);
    bool configureMasterPDOs();
    void updateInput() override {};
    Eigen::VectorXf &getForces() { return forces; };
    Eigen::VectorXf &getCOP() { return cop; };
    void sendCalibCommand(int cmd) { calibCmdData = cmd; };
    bool isCalibReady() { return calibReady != 0; }
    int getSamplesCollected() { return samplesCollected; }

    private:
    int forceID1, forceID2, copID; //each plate transmits 4 forces and 2 CoP values.
    int calibCmdID;
    int statusID;
    Eigen::VectorXf forces = Eigen::VectorXf::Zero(4); //forces from the force plate
    Eigen::VectorXf cop = Eigen::VectorXf::Zero(2); //
    int calibCmdData = 0;
    int calibReady = 0;
    int samplesCollected = 0;
    RPDO *rpdoForce1, *rpdoForce2, *rpdoCoP;
    TPDO *tpdoCalibCmd;
    RPDO *rpdoStatus;
};

#endif
