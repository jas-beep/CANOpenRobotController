#include "ForcePlate.h"

ForcePlate::ForcePlate(std::string robot_name, std::string yaml_config_file) :  Robot(robot_name, yaml_config_file) {
    spdlog::debug("ForcePlate created");

    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);
    initialiseInputs();
    if (hasYamlScaleFactors) {
        setStrainScaleFactors(yamlScaleFactors);
    }
}

ForcePlate::~ForcePlate() {
    spdlog::debug("Delete ForcePlate object begins");

    // Delete any joints (there shouldn't be any)
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();

    // Delete the Inputs
    delete keyboard;

    delete strainGauges;
    inputs.clear();

    spdlog::debug("ForcePlate deleted");
}


bool ForcePlate::initialiseInputs() {
    // Useful for testing, not really required but doesn't hurt to have it, even without a keyboard
    addInput(keyboard = new Keyboard());

    Eigen::Matrix<int, NFORCE, 2> inputPins;

    //TODO: change these ifdef to proper switch and set config in construcor. Create small struct or so to define pins. Maybe even in YAML file.
    ////// For BeagleBone Black //////

    //Pins are define as {PORT, PIN} pairs
    #ifdef FP_BBB
    // Force Plate 1
    inputPins << 8, 10,
                8, 12,
                8, 14,
                8, 16;
    Eigen::Vector2i clock = {8,8}; // Clock Pin


    // Force Plate 2
    inputPins2 <<   8, 9,
                    8, 11,
                    8, 15,
                    8, 17;
    Eigen::Vector2i clock2 = {8, 7};  // Clock Pin
    #endif
    #ifdef FP_PB
    // Force Plate 1
    inputPins << 2, 4,
                2, 6,
                2, 8,
                2, 10;
    Eigen::Vector2i clock = {2,2}; // Clock Pin


    /* Force Plate 2
    inputPins2 <<   2, 18,
                    2, 20,
                    2, 22,
                    2, 24;
    Eigen::Vector2i clock2 = {2, 17};  // Clock Pin*/
    #endif

    //TODO consider adding as inputs and use standard input update method
    strainGauges = new HX711(inputPins, clock);
    addInput(strainGauges);
    spdlog::info("Starting SGs");
    strainGauges->begin(128);

    return true;
}

void ForcePlate::printStatus() {
    std::cout << std::setprecision(3) << std::fixed << std::showpos;
    std::cout << "Cmd=" << currCommand << "\t";
    std::cout << "Gauges=[ " << getStrainReadings().transpose() << " ]\t";
    std::cout << "COP=[ " << getCOP().transpose() << " ]\t";
    std::cout <<  std::endl;
    std::cout <<  std::noshowpos << std::defaultfloat << std::setprecision(6);
}
void ForcePlate::printJointStatus() {
    printStatus();
}

Eigen::VectorXd& ForcePlate::getStrainReadings() {
    strainForces = Eigen::VectorXd::Zero(NFORCE);
    strainForces.segment<NFORCE>(0) = strainGauges->getAllForces();
    return strainForces;
}

VF4i ForcePlate::getRawStrainReadings() {
    VF4i rawData = Eigen::VectorXi::Zero(NFORCE);
    if(strainGauges->nbGauges() == NFORCE) {
        rawData = strainGauges->getAllRawData();
        std::cout << rawData << "\n\n";
    }
    else {
        spdlog::error("ForcePlate: Wrong HX711 sensors number.");
    }
    return rawData;
}

void ForcePlate::setStrainOffsets(Eigen::Vector4i offsets) {
    for (int i = 0; i<NFORCE; i++) {
        strainGauges->setOffset(i, offsets(i));
    }
}

void ForcePlate::setStrainScaleFactors(Eigen::Vector4d scaleFactors) {
    for (int i = 0; i<NFORCE; i++) {
        strainGauges->setScale(i, scaleFactors(i));
    }
}

void ForcePlate::setCOPRatios(VF4 xRatio, VF4 yRatio) {
    sensorXRatio = xRatio;
    sensorYRatio = yRatio;
}

void ForcePlate::setCOPLinearCalibration(double xSlope, double xIntercept, double ySlope, double yIntercept) {
    copXSlope = xSlope;
    copXIntercept = xIntercept;
    copYSlope = ySlope;
    copYIntercept = yIntercept;
}

Eigen::VectorXd &ForcePlate::getCOP(){
    VF4 F = getStrainReadings().head<NFORCE>();
    double total = F.sum();                              

    if (std::abs(total) < 5.0) {      // check if weight on plate 
        currentCOP = Eigen::VectorXd::Zero(2);
        return currentCOP;
    }

    VF4 f = F / total;                                  // normalize forces to sum to 1.0,
    currentCOP(0) = sensorXRatio.dot(f);                // ratio may not be exactly 1:1 (x off by as much as .25)
    currentCOP(1) = sensorYRatio.dot(f);

    // Per-axis (ML/AP) linear calibration fit, applied after the ratio correction above
    currentCOP(0) = copXSlope * currentCOP(0) + copXIntercept;
    currentCOP(1) = copYSlope * currentCOP(1) + copYIntercept;

    return currentCOP;
}

Eigen::VectorXd &ForcePlate::getCOPRatio(){
    currentCOPRatio.head<NFORCE>() = sensorXRatio;
    currentCOPRatio.tail<NFORCE>() = sensorYRatio;
    return currentCOPRatio;
}

Eigen::VectorXd &ForcePlate::getCOPLinearRegression(){
    currentCOPLinearRegression(0) = copXSlope;
    currentCOPLinearRegression(1) = copXIntercept;
    currentCOPLinearRegression(2) = copYSlope;
    currentCOPLinearRegression(3) = copYIntercept;
    return currentCOPLinearRegression;
}

bool ForcePlate::loadParametersFromYAML(YAML::Node params) {
    if (params["ForcePlate"]["plateID"]) {
        plateID = params["ForcePlate"]["plateID"].as<int>();
        startTPDO = FP_CMDRPDO + 1 + plateID;
        spdlog::info("ForcePlate: plateID set to {} and startTPDO set to {}", plateID, startTPDO);
    }
    if (params["ForcePlate"]["calibMassKg"]) {
        calibMassKg = params["ForcePlate"]["calibMassKg"].as<double>();
        spdlog::info("ForcePlate: calibMassKg set to {}", calibMassKg);
    }
    if (params["ForcePlate"]["scaleFactors"]) {
        auto sf = params["ForcePlate"]["scaleFactors"];
        if (sf.size() == NFORCE) {
            for (int i = 0; i<NFORCE; i++) {
                yamlScaleFactors(i) = sf[i].as<double>();
            }
            hasYamlScaleFactors = true;
            spdlog::info("ForcePlate: scaleFactors set to [{}, {}, {}, {}]", yamlScaleFactors(0), yamlScaleFactors(1),
            yamlScaleFactors(2), yamlScaleFactors(3));
            } else {
            spdlog::error("ForcePlate: scaleFactors in YAML file must have {} elements, but has {}", NFORCE, sf.size());
        }
    }
    return true;
}
 
bool ForcePlate::configureMasterPDOs() {
    spdlog::debug("ForcePlate configure Master PDO");
    Robot::configureMasterPDOs();
    
    strainForcesTPDO = Eigen::VectorXf(NFORCE);
    copTPDO = Eigen::VectorXf(2);

    UNSIGNED16 dataSize[2] = {4, 4};
    UNSIGNED16 dataSizeCOP[2] = {4, 4};

    UNSIGNED16 RPDO_CMD = FP_CMDRPDO;
    UNSIGNED16 TPDOStart = startTPDO; //0x3E1 (default if no yaml config file)

    // Create TPODs for the measurements
    for (uint i = 0; i<2; i++) {
        void *dataPointer[] = {(void *)&strainForcesTPDO(2*i), (void *)&strainForcesTPDO(2*i+1)};
        TPDO *tpdo = new TPDO(TPDOStart+i, 0xff, dataPointer, dataSize, 2);
        tpdo->commParam.eventTimer = 20;
        tpdos.push_back(tpdo);
    }

    void *dataPointerCOP[] = {(void *)&copTPDO(0), (void *)&copTPDO(1)};
    TPDO *tpdoCOP = new TPDO(TPDOStart+2, 0xff, dataPointerCOP, dataSizeCOP, 2);
    tpdoCOP->commParam.eventTimer = 20;
    tpdos.push_back(tpdoCOP);

    //receives commands from master.
    UNSIGNED16 dataCmdSize[2] = {4};
    void *cmdPointer[] = {(void *)&currCommand};
    rpdoCmd = new RPDO(RPDO_CMD, 0xff, cmdPointer, dataCmdSize, 1);

    //receives per plate calibration commands (separate from shared above)
    UNSIGNED16 RPDO_CALIB = FP_CMDRPDO + 4 + plateID; 
    UNSIGNED16 dataCalibSize[1] = {4};
    void *calibPointer[] = {(void *)&calibCommand};
    rpdoCalibCmd = new RPDO(RPDO_CALIB, 0xff, calibPointer, dataCalibSize, 1);

    return true;
}

void ForcePlate::updateRobot() {
    spdlog::trace("ForcePlate update");
    Robot::updateRobot();

    if (currCommand == STARTSTREAM) {
        sensorsOn = true;
        resetCommand();
    }
    else if (currCommand == STOP) {
        sensorsOn = false;
        resetCommand();
    }

    getStrainReadings();
    getCOP();
    sumOfForces = strainForces.sum(); //fix
    updatePDOs();
    
}

void ForcePlate::updatePDOs() {
    spdlog::trace("ForcePlate update PDO");
    if (sensorsOn) {
        for (int i = 0; i < strainForces.size(); i++) {
            strainForcesTPDO(i) = strainForces(i);
        }
        copTPDO(0) = currentCOP(0);
        copTPDO(1) = currentCOP(1);
    } else {
        strainForcesTPDO.setZero();
        copTPDO.setZero();
    }
}

ForcePlateCommand ForcePlate::getCommand() {
    return currCommand;
}

ForcePlateCommand ForcePlate::getCalibCommand() {
    return calibCommand;
}

void ForcePlate::resetCalibCommand() {
    calibCommand = NONE;
}

void ForcePlate::resetCommand() {
    currCommand = NONE;
}
