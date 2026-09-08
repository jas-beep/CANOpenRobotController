#include "ForcePlateReceiver.h"

ForcePlateReceiver::ForcePlateReceiver(int forceID1_, int forceID2_, int copID_, int calibCmdID_) {
    spdlog::info("Force Plate Receiver Created");

    // Change the parameters
    forceID1 = forceID1_;
    forceID2 = forceID2_;
    copID = copID_;
    calibCmdID = calibCmdID_;
}

bool ForcePlateReceiver::configureMasterPDOs() {
    
    //configure RPDOs to receive forces and CoP from the force plate
    UNSIGNED16 dataSize[2] = {4,4};
    void *f1[8] = {(void *)&forces(0), (void *)&forces(1)};
    void *f2[8] = {(void *)&forces(2), (void *)&forces(3)};
    void *c[8] = {(void *)&cop(0), (void *)&cop(1)};
    rpdoForce1 = new RPDO(forceID1, 0xff, f1, dataSize, 2);
    rpdoForce2 = new RPDO(forceID2, 0xff, f2, dataSize, 2);
    rpdoCoP = new RPDO(copID, 0xff, c, dataSize, 2);

    //configure TPDO to send calibration command to the force plate
    UNSIGNED16 dataSizeCalib[1] = {4};
    void *calibPointer[1] = {(void *)&calibCmdData};
    tpdoCalibCmd = new TPDO(calibCmdID, 0xff, calibPointer, dataSizeCalib, 1);
    tpdoCalibCmd->commParam.eventTimer = 20; 

    return true;
}