#include "ForcePlateMaster.h"

ForcePlateMaster::ForcePlateMaster(std::string robot_name, std::string yaml_config_file)
 : Robot(robot_name, yaml_config_file) {
    initialiseFromYAML(yaml_config_file); 
    initialiseInputs();}

bool ForcePlateMaster::loadParametersFromYAML(YAML::Node params) {
    if (params["ForcePlateMaster"]["plateIDs"]) {
        plateIDs.clear();
        for (auto id : params["ForcePlateMaster"]["plateIDs"]) {
            plateIDs.push_back(id.as<int>());
        }
        spdlog::info("ForcePlateMaster: loaded {} plate ID(s)", plateIDs.size());
    }
    return true;
}

bool ForcePlateMaster::initialiseInputs() {
    inputs.push_back(keyboard = new Keyboard());

    for (int id : plateIDs) {
        plates.push_back(new ForcePlateReceiver(
            FP_CMDRPDO + 1 + id,
            FP_CMDRPDO + 2 + id,
            FP_CMDRPDO + 3 + id,
            FP_CMDRPDO + 4 + id,
            FP_CMDRPDO + 5 + id
        ));
    }

    for (auto p : plates) {
        inputs.push_back(p);
    }

    return true;
}

Eigen::VectorXd &ForcePlateMaster::getSummedForces() {
    summedForces = Eigen::VectorXd::Zero(plates.size());
    for (size_t i = 0; i < plates.size(); i++) {
        summedForces(i)= getSummedForce(i);
    }
    return summedForces;
}

//janky workaround to get double tpe for UIserver, since Eigen::VectorXf is not compatible
Eigen::VectorXd &ForcePlateMaster::getCOPd() {
    copD = Eigen::VectorXd::Zero(plates.size()*2);
    for (size_t i = 0; i < plates.size(); i++) {
        copD(i*2) = getCOP(i)(0);
        copD(i*2+1) = getCOP(i)(1);
    }
    return copD;
}

void ForcePlateMaster::updateRobot() {
    Robot::updateRobot();
    getSummedForces(); // refresh for UIserver/logHelper, which just re-read these members' addresses each tick
    getCOPd();
}

bool ForcePlateMaster::configureMasterPDOs() {
    Robot::configureMasterPDOs();
    for (auto p: plates) p->configureMasterPDOs();

    UNSIGNED16 dataSize[1] = {4};
    void *cmdPtr[] = {(void *)&cmdDATA};
    cmdTPDO = new TPDO(FP_CMDRPDO, 0xff, cmdPtr, dataSize, 1); 
    //cmdTPDO->commParam.eventTimer = 20; //20ms
    return true;
}