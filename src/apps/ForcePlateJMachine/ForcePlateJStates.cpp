#include "ForcePlateJStates.h"
#include "ForcePlateJMachine.h"

using namespace std;


void CalibState::entry(void) {
    calibDone=false;
    calibValues.clear();
    nbCalibValues = 200;

    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}

//TODO : add methods that can do stuff like set dimensions of plate, CoP, etc.
//TODO : calibrate, get some proper readings with weigths

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

void SetScale::entry(void) {
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

//Faulty, may need to be removed in final version.
void SetScalePerCorner::entry(void){
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
    //TODO: implement per corner calibration, e.g. place weight on each corner and get scale factors for each corner
    if (waitingForUser){
        if (robot->keyboard->getNb()==2){
            waitingForUser = false;
            std::cout << "Collecting samples (keep clear)..." << std::flush;
            }
        return;
        }
    if(rawADCwithWeight.size()<nbWeightedCalibValues){
        rawADCwithWeight.push_back(robot->getStrainReadings().head<NFORCE>()); //getStrainReadings is from Hardware HX711 IO class
        std::cout << rawADCwithWeight[0] <<'\n';
        std::cout << ".";
        return;
    }
    VF4 mean = VF4::Zero();
    for (VF4 v: rawADCwithWeight){
        mean += v / (double)nbWeightedCalibValues;
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
        
        robot->setStrainScaleFactors(scaleFactors);
        std::cout << "Per corner calibration done. Scale factors set to: " << scaleFactors.transpose() << '\n';
        std::cout << "Remove weight" << std::flush;
        perCornerCalibDone = true;
    }
}
void SetScalePerCorner::exit(void){
    std::cout << " done/n";
    robot->printStatus();
};

void CalibrateCOP::entry(void) {
    calibDone=false;
    waitingForUser=true;
    calibValues.clear();
    placementValues.clear();
    nbCalibValues = 200;
    placementIndex = 0;
    xCoefficients = VF4::Zero();
    yCoefficients = VF4::Zero();
    xIntercept = 0;
    yIntercept = 0;

    //Center origin y-up and x-right normalized positions.
    //TODO: add semantic labels for each position in the future, e.g. "center", "top-left", etc.    
    knownPositions = {
        VF2(0, 0),                                      // center origin
        VF2(-1, -1), VF2(1, -1), VF2(1, 1), VF2(-1, 1), // corners
        VF2(-1, 0), VF2(1, 0), VF2(0, -1), VF2(0, 1)    // edges
    };

    robot->printJointStatus();

    std::cout << "Calibrating Center of Pressure (CoP):" << "\n";
    std::cout << "Place " << weight << "kg on the plate at position (center): " << placementIndex+1 << " and press 3 to continue..." << std::flush;    
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

   if(calibValues.size()<nbCalibValues){
        calibValues.push_back(robot->getStrainReadings().head<NFORCE>());
        std::cout << calibValues[0] <<'\n';
        std::cout << ".";
        return;
    }

    VF4 mean = VF4::Zero();
    for (VF4 v: calibValues){
        mean += v / (double)nbCalibValues;
    }
   
    placementValues.push_back(mean);
    std::cout << "Placement " << placementIndex+1 << " mean reading: " << mean.transpose() << '\n'; //TODO label positions semantically
    placementIndex++;
    calibValues.clear();

    if (placementIndex < knownPositions.size()){
        waitingForUser = true;
        std::cout << "Place " << weight << "kg on the plate at position: " << placementIndex+1 << " and press 3 to continue..." << std::flush;
    }
    else {
        fitRegression();
        robot->setCOPCalibrationCoefficients(xCoefficients, yCoefficients, xIntercept, yIntercept);
        calibDone = true;
        std::cout << "CoP calibration done. Remove weight." << std::flush;
    }
}
void CalibrateCOP::exit(void) {
    std::cout << " done/n";
    robot->printStatus();
}
void CalibrateCOP::fitRegression() {
    // A*beta = b, where beta = [x1, x2, x3, x4, intercept] for x and y respectively

    int n = placementValues.size();
    Eigen::MatrixXd A(n, 5);
    Eigen::VectorXd bx(n), by(n);

    for (int i = 0; i < n; i++){

        double total = placementValues[i].sum();

        A(i, 0) = placementValues[i](0) / total; // F1 (ratio)
        A(i, 1) = placementValues[i](1) / total; // F2
        A(i, 2) = placementValues[i](2) / total; // F3
        A(i, 3) = placementValues[i](3) / total; // F4
        A(i, 4) = 1;                     // intercept term

        bx(i) = knownPositions[i](0);   // known x position
        by(i) = knownPositions[i](1);   // known y position
    }

    Eigen::VectorXd betaX = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bx);
    Eigen::VectorXd betaY = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(by);

    xCoefficients = betaX.head<4>();
    xIntercept = betaX(4);
    yCoefficients = betaY.head<4>();
    yIntercept = betaY(4);

    // Report fit quality (residuals)
    double xRMS = std::sqrt((A * betaX - bx).squaredNorm() / n);
    double yRMS = std::sqrt((A * betaY - by).squaredNorm() / n);
    std::cout << "Coefficients calculated: " << std::endl;
    std::cout << "xCoefficients: " << xCoefficients.transpose() << ", xIntercept: " << xIntercept << std::endl;
    std::cout << "yCoefficients: " << yCoefficients.transpose() << ", yIntercept: " << yIntercept << std::endl;
    std::cout << "Fit quality (RMS error): x = " << xRMS << ", y = " << yRMS << std::endl;
    
}