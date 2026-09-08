/**
 * \file ForcePlate.h
 * \author Justin Fong
 * \version 0.2
 * \date 2026-04-02
 * \copyright Copyright (c) 2021, 2026
 *
 * \brief The ForcePlate class is a force plate object, which measures 4 strain gauges (via HX711) - designed to provide force and COP measurements.
 * This class is designed to work with the sensor system developed at the University of Melbourne's Human Robotics Laboratory.
 * The 4 strain gauges - and their 4 HX711 - are handled by a single HX711 object (with shared clock).
 *
 */

#ifndef FORCEPLATE_H
#define FORCEPLATE_H

#include "Robot.h"
#include "Keyboard.h"
#include "HX711.h"

#define FP_PB
//#define FP_BBB

//TODO: Was defined in cmake originally: understand and cleanup
// Will need to be =/= for each plate and match master reading
//Likely to end-up on a YAML config file (global one with NodeID or separate one for each plate)
//TPDO defined in loadParamtersFromYaml 
#define FP_CMDRPDO 0x3E0

#define NFORCE 4 //!< Nb of overall force readings
typedef Eigen::Vector4d VF4; //!< Convenience alias for double Vector of length 4
typedef Eigen::Vector2d VF2; //!< Convenience alias for double Vector of length 2
typedef Eigen::Vector4i VF4i; //!< Convenience alias for Vector of length 4 for raw readings

enum ForcePlateCommand {
    NONE = 0,
    CALIBRATE = 1, //check what this did (maybe legacy?)
    STARTSTREAM = 2,
    RECORD = 3,
    STOP = 4,
    COP_CALIB = 5,  
    ADVANCE_PLACEMENT = 6 
};

enum ForcePlateStateID
{
    STANDBY = 0,
    TARE = 1,
    SET_SCALE_CORNER = 2,
    CALIBRATE_COP = 3,
};

class ForcePlate : public Robot {
   private:

    HX711 *strainGauges;
    Eigen::VectorXd strainForces;
    Eigen::VectorXf strainForcesTPDO;  // float for precision, smaller data format for better sending over bus
    Eigen::VectorXd currentCOP = Eigen::VectorXd::Zero(2); //purely for the registerstate
    Eigen::VectorXf copTPDO = Eigen::VectorXf::Zero(2); 
    bool sensorsOn =  false;
    ForcePlateCommand currCommand = NONE; 
    ForcePlateCommand calibCommand = NONE; // separate command (per plate) to not interfere with the shared command
    int currentStateID = STANDBY;

    // yaml params (defaults if not set in yaml)
    int plateID = 0x00;
    int startTPDO = FP_CMDRPDO + 1; //0x3E1 (default if no yaml config file)
    double calibMassKg = 4.2069;

    // 1 = sensor at y/2 of plate but footmount (where calibration force is recorded) is 8.9cm from center of bolts
    // y/2 of current plate is 17.65cm so ratioY = (17.65+8.9)/17.65 = 1.504.
    // whatever you intialize with will anyway be recalibrated. 
    //TODO: check COP correction factors and hardcode if consistent in this corner setup for all plates.
    VF4 sensorXRatio = VF4(-1, -1, 1, 1);  // Assumes sensor perfectly in corner uses bolt centroid
    VF4 sensorYRatio = VF4(-1, 1, -1, 1);

    Eigen::VectorXd currentCOPRatio = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd currentCOPLinearRegression = Eigen::VectorXd::Zero(4); // slope+intercept for x and y
    double sumOfForces = 0;

    // Per-axis (ML/AP) linear calibration fit, applied after the ratio correction above.
    // corrected = slope*ratioCOP + intercept.
    double copXSlope = 1.0;
    double copXIntercept = 0.0;
    double copYSlope = 1.0;
    double copYIntercept = 0.0;

    // 1=BL=(-1,-1), 2=TL=(-1,1), 3=BR=(1,-1), 4=TR=(1,1)


    std::vector<TPDO*> tpdos;
    RPDO *rpdoCalibCmd; // separate RPDO for the per-plate advance placement command, so that it doesn't interfere with the shared command (see ForcePlateMaster::triggerCOPCalibration())
    RPDO *rpdoCmd;
    void updatePDOs();

    //ugly fix (initialiseinputs for hx711 is called after the yaml config is loaded so cannot be set directly)
    VF4 yamlScaleFactors = VF4::Ones(); //default to 1.0 if not set in yaml
    bool hasYamlScaleFactors = false; //default to false if not set in yaml


   public:
    Keyboard *keyboard;

    ForcePlate(std::string robot_name="", std::string yaml_config_file="");
    ~ForcePlate();

    bool initialiseJoints() { return true; };
    bool initialiseInputs();
    bool initialiseNetwork() { return true; };

    void printStatus();
    void printJointStatus();

    void setStrainOffsets(Eigen::Vector4i offsets);
    void setStrainScaleFactors(Eigen::Vector4d scaleFactors);
    void setStateID(int id) {currentStateID = id;}
    void setSensorsOn(bool on) {sensorsOn = on;} //!< force live PDO transmission on/off, independent of the broadcast STARTSTREAM/STOP command
    void setCOPRatios(VF4 xRatio, VF4 yRatio); // recalibrate COP
    void setCOPLinearCalibration(double xSlope, double xIntercept, double ySlope, double yIntercept); // per-axis (ML/AP) correction applied after the ratio COP

    Eigen::VectorXd &getStrainReadings(); //!< Return calibrated readings from strain gauges
    VF4i getRawStrainReadings(); //!< Return raw readings from strain gauges
    Eigen::VectorXd &getCOP(); //!< Return the current CoP (in normalized plate coordintaes)
    Eigen::VectorXd &getCOPRatio(); //!<CORC needs dynamic vector sizing for return by reference
    Eigen::VectorXd &getCOPLinearRegression(); //!< Return the slope+intercept 
    int &getStateID() {return currentStateID;} //!< Return stateID for easier log csv analysis
    double &getSumOfForces() {return sumOfForces;} //!< Return sum of strain readings (updated in updateRobot)
    double getCalibMassKg() {return calibMassKg;} //!< Return the calibration mass (in kg) used for COP calibration
    
    bool configureMasterPDOs();
    bool loadParametersFromYAML(YAML::Node params) override;

    void updateRobot();

    ForcePlateCommand getCommand();
    ForcePlateCommand getCalibCommand();

    void resetCommand();
    void resetCalibCommand();
};

#endif /*ForcePlate.h*/
