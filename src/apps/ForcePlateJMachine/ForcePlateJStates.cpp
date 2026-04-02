#include "ForcePlateJStates.h"
#include "ForcePlateJMachine.h"

using namespace std;


void CalibState::entry(void) {
    calibDone=false;
    calibValues.clear();
    nbCalibValues = 100;

    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Average a number of empty readings to offset
void CalibState::during(void) {

    //Still collecting values
    if(iterations()<=nbCalibValues){
        //add current reading to the list
        calibValues.push_back(robot->getRawStrainReadings());
        std::cout << calibValues[0] <<"\n";
        std::cout << ".";
    }
    //we have enough values
    else {
        //sum them to get mean and use as offset
        VF4 offset = VF4::Zero();
        for(VF4i v: calibValues) {
            std::cout << offset <<"\n\n";
            std::cout << v <<"\n";
            std::cout << v.cast<double>() <<"\n";
            offset += v.cast<double>()/(double)nbCalibValues;

        }
        printf("bb\n");
        VF4i offseti = offset.cast<int>();
        robot->setStrainOffsets(offseti);
        calibDone = true;
    }
}
void CalibState::exit(void) {
    std::cout << " done/n";
    robot->printStatus();
}


void StandbyState::entry(void) {
    //Check if command is different from NONE or CALIBRATE??
}
void StandbyState::during(void) {

    //TODO
    //Actively call getStrainReadings() (not sure if necessary) or uncomment some code in the updateRobot ?

    //Regular display status
    if(iterations()%10==1) {
        robot->printStatus();
    }
}
void StandbyState::exit(void) {
}

