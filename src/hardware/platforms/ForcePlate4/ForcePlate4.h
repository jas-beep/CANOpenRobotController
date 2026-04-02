/**
 *
 * \file ForcePlate4.h
 * \author Justin Fong
 * \version 0.2
 * \date 2026-04-02
 * \copyright Copyright (c) 2021, 2026
 *
 * \brief  The ForcePlate4 class is a force plate object, which measures 4 strain gauages - designed to provide force and COP measurements
 *
 * This class is designed to work with the sensor system developed at the University of Melbourne's Human Robotics Laboratory
 *
 */

#ifndef FORCEPLATE4_H_INCLUDED
#define FORCEPLATE4_H_INCLUDED

#include "Keyboard.h"
#include "Robot.h"
#include "HX711.h"

#define FP_PB
//#define FP_BBB

//TODO: Was defined in cmake originally: understand and cleanup
// Will need to be =/= for each plate and match master reading
//Likely to end-up on a YAML config file (global one with NodeID or separate one for each plate)
#define FP_CMDRPDO 0x3E0
#define FP_STARTTPDO 0x3E1


typedef Eigen::Vector4d VF4; //!< Convenience alias for double  Vector of length 4
typedef Eigen::Vector4i VF4i; //!< Convenience alias for Vector of length 4 for raw readings

enum ForcePlateCommand {
    NONE = 0,
    CALIBRATE = 1,
    STARTSTREAM = 2,
    RECORD = 3,
    STOP = 4,
};


class ForcePlate4 : public Robot {
   private:

    std::vector<HX711*> strainGauges;
    Eigen::VectorXd strainForces;
    Eigen::VectorXi strainForcesTPDO;  // Smaller data format for better sending over bus
    bool sensorsOn =  false;
    ForcePlateCommand currCommand = NONE;

    std::vector<TPDO*> tpdos;
    RPDO* rpdoCmd;
    void updatePDOs();


   public:
    Keyboard *keyboard;

    ForcePlate4(std::string robot_name="", std::string yaml_config_file="");
    ~ForcePlate4();

    // Functions which are needed for the Robot Class - they don't do anything at the moment
    bool initialiseJoints() { return true; };
    bool initialiseInputs();
    bool initialiseNetwork() { return true; };  // this one might need to be changed

    void printStatus();
    void printJointStatus();

    void setStrainOffsets(Eigen::VectorXi offsets);

    Eigen::VectorXd &getStrainReadings(); //!< Return calibrated readings from stain gauges
    VF4i getRawStrainReadings(); //!< Return raw readings from stain gauges

    bool configureMasterPDOs();

    void updateRobot();

    ForcePlateCommand getCommand();

    void resetCommand();
};

#endif /*ForcePlate.h*/
