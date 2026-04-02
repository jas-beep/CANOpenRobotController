#include "ForcePlate4.h"

ForcePlate4::ForcePlate4(std::string robot_name, std::string yaml_config_file) :  Robot(robot_name, yaml_config_file) {
    spdlog::debug("ForcePlate4 created");

    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);

    initialiseInputs();
}

ForcePlate4::~ForcePlate4() {
    spdlog::debug("Delete ForcePlate4 object begins");

    // Delete any joints (there shouldn't be any)
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();

    // Delete the Inputs
    delete keyboard;

    for (auto sg: strainGauges){
        delete sg;
    }
    inputs.clear();

    spdlog::debug("ForcePlate4 deleted");
}


bool ForcePlate4::initialiseInputs() {
    // Useful for testing, not really required but doesn't hurt to have it, even without a keyboard
    addInput(keyboard = new Keyboard());

    Eigen::Matrix<int, 4, 2> inputPins;
    Eigen::Matrix<int, 4, 2> inputPins2;

    //TODO: change these ifdef to proper swicth and set config in construcor . Create small strcuts or so to define pins.
    ////// For BeagleBone Black //////
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


    // Force Plate 2
    inputPins2 <<   2, 18,
                    2, 20,
                    2, 22,
                    2, 24;
    Eigen::Vector2i clock2 = {2, 17};  // Clock Pin
    #endif

    //TODO consider adding as inputs and use standard input update method
    strainGauges.push_back(new HX711(inputPins, clock));
    strainGauges.push_back(new HX711(inputPins2, clock2));

    for (auto sg : strainGauges ){
        spdlog::info("Starting SGs");
        inputs.push_back(sg);
        sg->begin(128);
    }
    return true;
}

void ForcePlate4::printStatus() {
    std::cout << std::setprecision(3) << std::fixed << std::showpos;
    std::cout << "Cmd=" << currCommand << "\t";
    std::cout << "Gauges=[ " << getStrainReadings().transpose() << " ]\t";
    std::cout <<  std::endl;
    std::cout <<  std::noshowpos;
}
void ForcePlate4::printJointStatus() {
    printStatus();
}

Eigen::VectorXd& ForcePlate4::getStrainReadings() {
    strainForces = Eigen::VectorXd::Zero(strainGauges.size()*4);
    int i = 0;
    for (auto sg : strainGauges) {
        strainForces.segment<4>(i * 4) = sg->getAllForces();
        i++;
    }
    return strainForces;
}

VF4i ForcePlate4::getRawStrainReadings() {
    VF4i rawData = Eigen::VectorXi::Zero(strainGauges.size() * 4);
    std::cout << rawData << "\n\n";
    int i = 0;
    for (auto sg : strainGauges) {
        rawData.segment<4>(i * 4) = sg->getAllRawData();
        i++;
        std::cout << rawData << "\n\n";
    }
    return rawData;
}

void ForcePlate4::setStrainOffsets(Eigen::VectorXi offsets) {
    int j = 0;
    for (auto sg : strainGauges) {
        for (int i = 0; i < 4; i++) {
            sg->set_offset(i, offsets(j));
            j++;
        }
    }
}

bool ForcePlate4::configureMasterPDOs() {
    spdlog::debug("ForcePlate4 configure Master PDO");
    Robot::configureMasterPDOs();

    strainForcesTPDO = Eigen::VectorXi(4*strainGauges.size());
    UNSIGNED16 dataSize[2] = {4, 4};

    UNSIGNED16 RPDO_CMD = FP_CMDRPDO;
    UNSIGNED16 TPDOStart = FP_STARTTPDO;

    // Create TPODs for the measurements
    for (uint i = 0; i < strainGauges.size()*2; i++) {
        void *dataPointer[] = {(void *)&strainForcesTPDO(2*i), (void *)&strainForcesTPDO(2*i+1)};
        tpdos.push_back(new TPDO(TPDOStart+i, 0xff, dataPointer, dataSize, 2));
    }

    UNSIGNED16 dataCmdSize[2] = {4};
    void *cmdPointer[] = {(void *)&currCommand};
    rpdoCmd = new RPDO(RPDO_CMD, 0xff, cmdPointer, dataCmdSize, 1);

    return true;
}

void ForcePlate4::updateRobot() {
    spdlog::trace("ForcePlate4 update");
    Robot::updateRobot();
    getStrainReadings();
    updatePDOs();
}

void ForcePlate4::updatePDOs() {
    spdlog::trace("ForcePlate4 update PDO");
    for (int i = 0; i < strainForces.size(); i++) {
        strainForcesTPDO(i) = strainForces(i);
    }
}

ForcePlateCommand ForcePlate4::getCommand() {
    return currCommand;
}

void ForcePlate4::resetCommand() {
    currCommand = NONE;
}
