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
            FP_CMDRPDO + 4 + id 
        ));
    }

    for (auto p : plates) {
        inputs.push_back(p);
    }

    return true;
}

Eigen::VectorXf &ForcePlateMaster::getGlobalCoP() {
    globalCoP = Eigen::VectorXf::Zero(2);
    float totalForce = 0;
    for (size_t i = 0; i < plates.size(); i++) {
        float F = getSummedForce(i);
        // TODO: offset by each plate's physical (x,y) origin once plate geometry is added to forceplatemaster_params.yaml
        globalCoP += F * getCoP(i);
        totalForce += F;
    }
    if (std::abs(totalForce) > 1e-3f) {
        globalCoP /= totalForce;
    }
    return globalCoP;
}

bool ForcePlateMaster::configureMasterPDOs() {
    Robot::configureMasterPDOs();
    for (auto p: plates) p->configureMasterPDOs();

    UNSIGNED16 dataSize[1] = {4};
    void *cmdPtr[] = {(void *)&cmdDATA};
    cmdTPDO = new TPDO(FP_CMDRPDO, 0xff, cmdPtr, dataSize, 1); 
    cmdTPDO->commParam.eventTimer = 20; //20ms
    return true;
}