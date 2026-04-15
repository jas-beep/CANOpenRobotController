#include "ForcePlate.h"

ForcePlate::ForcePlate(std::string robot_name, std::string yaml_config_file) :  Robot(robot_name, yaml_config_file) {
    spdlog::debug("ForcePlate created");

    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);
    initialiseInputs();
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
    std::cout <<  std::endl;
    std::cout <<  std::noshowpos;
}
void ForcePlate::printJointStatus() {
    printStatus();
}

Eigen::VectorXd& ForcePlate::getStrainReadings() {
    strainForces = Eigen::VectorXd::Zero(NFORCE);
    strainForces.segment<NFORCE>(NFORCE) = strainGauges->getAllForces();
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

bool ForcePlate::configureMasterPDOs() {
    spdlog::debug("ForcePlate configure Master PDO");
    Robot::configureMasterPDOs();

    strainForcesTPDO = Eigen::VectorXi(NFORCE);
    UNSIGNED16 dataSize[2] = {4, 4};

    UNSIGNED16 RPDO_CMD = FP_CMDRPDO;
    UNSIGNED16 TPDOStart = FP_STARTTPDO;

    // Create TPODs for the measurements
    for (uint i = 0; i<2; i++) {
        void *dataPointer[] = {(void *)&strainForcesTPDO(2*i), (void *)&strainForcesTPDO(2*i+1)};
        tpdos.push_back(new TPDO(TPDOStart+i, 0xff, dataPointer, dataSize, 2));
    }

    UNSIGNED16 dataCmdSize[2] = {4};
    void *cmdPointer[] = {(void *)&currCommand};
    rpdoCmd = new RPDO(RPDO_CMD, 0xff, cmdPointer, dataCmdSize, 1);

    return true;
}

void ForcePlate::updateRobot() {
    spdlog::trace("ForcePlate update");
    Robot::updateRobot();
    getStrainReadings();
    updatePDOs();
}

void ForcePlate::updatePDOs() {
    spdlog::trace("ForcePlate update PDO");
    for (int i = 0; i < strainForces.size(); i++) {
        strainForcesTPDO(i) = strainForces(i);
    }
}

ForcePlateCommand ForcePlate::getCommand() {
    return currCommand;
}

void ForcePlate::resetCommand() {
    currCommand = NONE;
}
