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
#define FP_CMDRPDO 0x3E0
#define FP_STARTTPDO 0x3E1


//TODO: add actual force plate sizes and other parameters and in-built methods to provide CoP and force vector


#define NFORCE 4 //!< Nb of overall force readings
typedef Eigen::Vector4d VF4; //!< Convenience alias for double Vector of length 4
typedef Eigen::Vector2d VF2; //!< Convenience alias for double Vector of length 2
typedef Eigen::Vector4i VF4i; //!< Convenience alias for Vector of length 4 for raw readings

//TODO: check if interferes at all with current state of the code
enum ForcePlateCommand { 
    NONE = 0,
    CALIBRATE = 1,
    STARTSTREAM = 2,
    RECORD = 3,
    STOP = 4,
};


class ForcePlate : public Robot {
   private:

    HX711 *strainGauges;
    Eigen::VectorXd strainForces;
    Eigen::VectorXi strainForcesTPDO;  // Smaller data format for better sending over bus
    bool sensorsOn =  false;
    ForcePlateCommand currCommand = NONE;
    VF4 copXCoeffs = VF4::Zero(); 
    VF4 copYCoeffs = VF4::Zero(); 
    double copXIntercept = 0.0; 
    double copYIntercept = 0.0;
    bool copCalibrated = false; 

    std::vector<TPDO*> tpdos;
    RPDO *rpdoCmd;
    void updatePDOs();


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
    void setCOPCalibrationCoefficients(VF4 xCoeffs, VF4 yCoeffs, double xIntercept, double yIntercept);


    Eigen::VectorXd &getStrainReadings(); //!< Return calibrated readings from strain gauges
    VF4i getRawStrainReadings(); //!< Return raw readings from strain gauges
    VF2 getCOP(); //!< Return the current CoP (in plate coordinates) based on the current readings and calibration coefficients

    bool configureMasterPDOs();

    void updateRobot();

    ForcePlateCommand getCommand();

    void resetCommand();
};

#endif /*ForcePlate.h*/
