#include "ForcePlateJStates.h"
#include "ForcePlateJMachine.h"

#include <fstream>

using namespace std;


void CalibState::entry(void) {
    calibDone=false;
    calibValues.clear();
    nbCalibValues = 200;

    robot->printJointStatus();
    robot->setStateID(TARE);
    std::cout << "Calibrating (keep clear)..." << std::flush;
}

//TODO : add methods that can do stuff like set dimensions of plate, CoP, etc.
//Average a number of empty readings to offset
void CalibState::during(void) {

    //Still collecting values with check range is 1e4-1e5 so reject 1e6 (simple fix, may need IQR outlier rejects instead)
    if(calibValues.size() < nbCalibValues){
        VF4i raw = robot->getRawStrainReadings();
        if (raw.cwiseAbs().maxCoeff() <= 1e6){
            calibValues.push_back(raw);
            std::cout << calibValues.back() <<"\n";
        }
    }
    //we have enough values
    else {
        //sum them to get mean and use as offset
        VF4 offset = VF4::Zero();
        for(VF4i v: calibValues) {
            std::cout << offset <<"\n\n";
            std::cout << v <<"\n";
            std::cout << v.cast<double>() <<"\n";             //Why all this casting?
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
        robot->setStateID(STANDBY);
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

// Remove SetScale in future
void SetScale::entry(void) {
    robot->setStateID(SET_SCALE);
    weightedCalibDone = false;
    waitingForUser = true; 
    rawADCwithWeight.clear(); 
    nbWeightedCalibValues = 200;

    robot->printJointStatus();
    robot->setStrainScaleFactors(Eigen::Vector4d::Ones()); //set scales to 1 before calcing in during

    std::cout << "Weighted Calibration:" << "\n";
    std::cout << "Place " << weight << "kg on the plate and press 1 to continue..." << std::flush;
    

}
void SetScale::during(void) {
    if (waitingForUser){
        if (robot->keyboard->getNb()==1) {
            waitingForUser = false;
            std::cout << "Collecting samples (keep clear)..." << std::flush;
        }
        return;
    }
    //Sample collection
    if(rawADCwithWeight.size()<nbWeightedCalibValues){
        rawADCwithWeight.push_back(robot->getStrainReadings().head<NFORCE>());
        std::cout << rawADCwithWeight[0] <<"\n";
        std::cout << ".";
        return;
    }

    //average collected samples to get mean and use as scale factor
    if (weightedCalibDone) return; //safety
    VF4 mean = VF4::Zero();
    for (VF4 v: rawADCwithWeight){
        mean += v / (double)nbWeightedCalibValues;
    }
    for (int i=0; i<NFORCE; i++){
        std::cout << "Mean reading for gauge " << i << ": " << mean(i) << "\n";
        
        scaleFactors(i) = (weight/4*9.81)/mean(i);
    }
    robot->setStrainScaleFactors(scaleFactors);

    std::cout << "Weighted calibration done. Scale factor set to: " << scaleFactors.transpose() << '\n';
    std::cout << "Remove weight" << std::flush;

    weightedCalibDone = true;
        
}
void SetScale::exit(void) {
    std::cout << " done/n";
    robot->printStatus();
}

// Keep, flip plate and calibrate each sensor individually
void SetScalePerCorner::entry(void){
    robot->setStateID(SET_SCALE_CORNER);
    perCornerCalibDone = false;
    waitingForUser = true;
    rawADCwithWeight.clear();
    nbWeightedCalibValues = 200;
    currentGauge = 0;

    robot->printJointStatus();

    //set scales to 1 before calcing in during (overrides previous scale factors)
    robot->setStrainScaleFactors(Eigen::Vector4d::Ones()); 
    std::cout << "Per Corner Calibration:" << "\n";
    std::cout << "Place " << weight << "kg on corner:" << currentGauge+1 << " and press 2 to continue..." << std::flush;
}
void SetScalePerCorner::during(void){
    if (waitingForUser){
        if (robot->keyboard->getNb()==2){
            waitingForUser = false;
            std::cout << "Collecting samples (keep clear)..." << std::flush;
            }
        return;
        }
    if(rawADCwithWeight.size()<nbWeightedCalibValues){
        VF4i raw = robot->getRawStrainReadings();
        if (raw.cwiseAbs().maxCoeff() <= 1e6){   // May need more advanced fix (sensor 3 has spurious readings)
            rawADCwithWeight.push_back(raw);
            std::cout << rawADCwithWeight.back() <<"\n";
        }
        return;
    }
    
    VF4 mean = VF4::Zero();
    for (VF4i v: rawADCwithWeight){
        mean += v.cast<double>() / (double)nbWeightedCalibValues;
    }

    scaleFactors(currentGauge) = (weight*9.81)/mean(currentGauge);
    std::cout << "Guage " << currentGauge << " scale factor set to: " << scaleFactors(currentGauge) << '\n';
    currentGauge++;
    rawADCwithWeight.clear();

    if (currentGauge < NFORCE){
        waitingForUser = true;
        std::cout << "Place " << weight << "kg on corner: " << currentGauge+1 << " and press 2 to continue..." << std::flush;
    }
    else {
        
        //ofstream to append scalefactors of multiple calibs to file for checking repeatability
        robot->setStrainScaleFactors(scaleFactors); 
        std::cout << "Per corner calibration done. Scale factors set to: " << scaleFactors.transpose() << '\n';
        std::cout << "Remove weight" << std::flush;
        std::ofstream logFile("logs/scaleFactorHistory.csv", std::ios::app);
        auto t = std::time(nullptr);
        logFile << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S") << ","
                << scaleFactors(0) << "," << scaleFactors(1) << ","
                << scaleFactors(2) << "," << scaleFactors(3) << '\n';
        perCornerCalibDone = true;

        }
}
void SetScalePerCorner::exit(void){
    std::cout << " done/n";
    robot->printStatus();
};

// COP defined in getCOP, unused
void CalibrateCOP::entry(void) {
    robot->setStateID(CALIBRATE_COP);
    calibDone=false;
    waitingForUser=true;
    calibValues.clear();
    placementValues.clear();
    nbCalibValues = 200;
    placementIndex = 0;
    rejectedSamples = 0;


    //Center origin y-up and x-right normalized positions.
    //TODO: add semantic labels for each position in the future, e.g. "center", "top-left", etc.
    knownPositions = {
        //VF2(0, 0),                                      // center origin
        VF2(-1, -1), VF2(-1, 1), VF2(1, -1), VF2(1, 1) // corners same order as forceplate.h (gotta be a better way than this!) BUG-PRONE
        };
    
    robot->setCOPRatios(VF4(-1, -1, 1, 1), VF4(-1, 1, -1, 1)); //set to generic perfect assumption at re-entry
    robot->printJointStatus();

    std::cout << "Validating Center of Pressure (CoP):" << "\n";
    std::cout << "Place " << weight << " kg on the plate at position: " << placementIndex+1 << " and press 3 to continue..." << std::flush;
}
void CalibrateCOP::during(void) {
   if (calibDone) return; //safety

   if (waitingForUser){
        if (robot->keyboard->getNb()==3){
        waitingForUser = false;
        std::cout << "Collecting samples (keep clear)..." << std::flush;
        }
        return;
   }

    // makes sure that not calibrating with sensor noise
    if(calibValues.size()<nbCalibValues){
        VF4 forces = robot->getStrainReadings().head<NFORCE>();
        if (forces.cwiseAbs().maxCoeff() <= 500){
            VF2 cop = robot->getCOP();
            if(!cop.isZero()){
                calibValues.push_back(cop);
                std::cout << calibValues.back() <<'\n';
                std::cout << ".";
            }
        } else {
            rejectedSamples++;
        }
        return;
    }

    
    VF2 mean = VF2::Zero();
    for (VF2 v: calibValues){
        mean += v / (double)nbCalibValues;
    }

    placementValues.push_back(mean);
    std::cout << "Placement " << placementIndex+1 << " mean reading: " << mean.transpose()
               << " (" << rejectedSamples << " glitch(es) rejected)" << '\n'; //TODO label positions semantically
    placementIndex++;
    calibValues.clear();
    rejectedSamples = 0;

    if (placementIndex < knownPositions.size()){
        waitingForUser = true;

        // Order must match knownPositions in entry() and sensor order in ForcePlate.h (F1=BL, F2=TL, F3=BR, F4=TR)
        static const std::vector<std::string> positionLabels = {
            "Bottom Left Corner (sensor 1)",
            "Top Left Corner (sensor 2)",
            "Bottom Right Corner (sensor 3)",
            "Top Right Corner (sensor 4)"
        };
        std::cout << "Place " << weight << "kg on the plate at position " << placementIndex+1
                   << ": " << positionLabels[placementIndex] << " and press 3 to continue..." << std::flush;
    }
    else {
        
        computeCOPRatio(); //this call resets the ratios
        calibDone = true;
        std::cout << "CoP validation done. Remove weight." << std::flush;
    }
}
void CalibrateCOP::exit(void) {
    std::cout << " done/n";
    robot->printStatus();
}

void CalibrateCOP::computeCOPRatio(){
    Eigen::VectorXd ratios = robot->getCOPRatio(); //8
    VF4 oldXRatio = ratios.head<NFORCE>(); //4
    VF4 oldYRatio = ratios.tail<NFORCE>(); //4

    int n = placementValues.size();
    VF4 xCorrection, yCorrection;

    for (int i=0; i < n; i++){
        xCorrection[i] =  knownPositions[i][0] / placementValues[i][0];
        yCorrection[i] =  knownPositions[i][1] / placementValues[i][1];

    }
    VF4 newXRatio = oldXRatio.cwiseProduct(xCorrection);
    VF4 newYRatio = oldYRatio.cwiseProduct(yCorrection);
    robot->setCOPRatios(newXRatio, newYRatio);

}
