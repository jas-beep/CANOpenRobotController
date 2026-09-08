#include "ForcePlateJStates.h"
#include "ForcePlateJMachine.h"

#include <fstream>

using namespace std;


void CalibState::entry(void) {
    calibDone=false;
    calibValues.clear();
    nbCalibValues = 200;

    robot->printJointStatus();
    robot->setStateID(TARE); //very useful for segmenting data in post
    std::cout << "Calibrating (keep clear)..." << std::flush;
}

//TODO : add methods that can do stuff like set dimensions of plate, CoP, etc.
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

// ADD feature that you can restart this from master.
void SetScalePerCorner::during(void){
    if (waitingForUser){
        if (robot->keyboard->getNb()==2){
            waitingForUser = false;
            std::cout << "Collecting samples (keep clear)..." << std::flush;
            }
        return;
        }

    // scale is set to 1 in entry so reading (raw-offset) during this collection
    if(rawADCwithWeight.size()<nbWeightedCalibValues){
        VF4 raw = robot->getStrainReadings().head<NFORCE>();
        if (raw.cwiseAbs().maxCoeff() <= 1e6){    // May need more advanced fix
            rawADCwithWeight.push_back(raw);
            std::cout << rawADCwithWeight.back() <<"\n";
        }
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
    robot->setSensorsOn(true); // stream live force/CoP to master during calibration, regardless of broadcast STARTSTREAM
    calibDone = false;
    waitingForUser = true;
    modeSelected = true; // default to ratio calibration
    calibMode = RATIO; // default to ratio calibration
    calibValues.clear();
    placementValues.clear();
    nbCalibValues = 200;
    placementIndex = 0;
    rejectedSamples = 0;

    
    robot->printJointStatus();

    std::cout << "Validating Center of Pressure (CoP):" << "\n";
    //std::cout << "Select calibration mode: 1=ratio correction, 2=per-axis linear fit, 3=both" << std::flush;
    beginRatioCalibration();
}
void CalibrateCOP::during(void) {
   if (calibDone) return; //safety

 /* 
   if (!modeSelected){
        if (robot->keyboard->getNb()==1){
                calibMode = RATIO;
                modeSelected = true;
                beginRatioCalibration();
        }
        else if (robot->keyboard->getNb()==2){
                calibMode = LINEAR;
                modeSelected = true;
                beginLinearCalibration();
        }
        else if (robot->keyboard->getNb()==3){
                calibMode = RATIO_AND_LINEAR;
                modeSelected = true;
                beginRatioCalibration();
        }
        return;
   }
*/

   if (waitingForUser){
        if (robot->keyboard->getNb()==3 || robot->getCalibCommand() == ADVANCE_PLACEMENT){
        if (robot->getCalibCommand() == ADVANCE_PLACEMENT) {
            robot->resetCalibCommand();
        }
        waitingForUser = false;
        std::cout << "Collecting samples (keep clear)..." << std::flush;
        }
        return;
   }

    // makes sure that not calibrating with sensor noise
    if(calibValues.size()<nbCalibValues){
        VF4 forces = robot->getStrainReadings().head<NFORCE>();
        if (forces.cwiseAbs().maxCoeff() <= 500){ // TODO more robust noise rejection
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

    if (placementIndex < (int)knownPositions.size()){
        waitingForUser = true;
        promptNextPlacement();
    }
    else if (phase == 0) {
        computeCOPRatio(); //this call resets the ratios (~0.027 mean position error at this stage)

        if (calibMode == RATIO_AND_LINEAR){
        beginLinearCalibration();
        }
        else {
        // first save previous linear calibration values
        robot->setCOPLinearCalibration(savedSlopes(0), savedIntercepts(0), savedSlopes(1), savedIntercepts(1));
        calibDone = true;
        std::cout << "CoP calibration done. Remove weight." << std::flush;
        }
    }
    else {
        computeLinearFit();
        calibDone = true;
        std::cout << "CoP calibration done. Remove weight." << std::flush;
    }
}
void CalibrateCOP::exit(void) {
    robot->setSensorsOn(false); // stop streaming once calibration ends - the drop to zero on the master IS the "done" signal
    std::cout << " done/n";
    robot->printStatus();
}

void CalibrateCOP::promptNextPlacement(){
    // Order must match knownPositions in beginRatio/LinearCalibration and sensor order in ForcePlate.h (F1=BL, F2=TL, F3=BR, F4=TR)
    static const std::vector<std::string> ratioPhaseLabels = {
        "Bottom Left Corner (sensor 1)",
        "Top Left Corner (sensor 2)",
        "Bottom Right Corner (sensor 3)",
        "Top Right Corner (sensor 4)"
    };
    static const std::vector<std::string> linearPhaseLabels = {
        "Center", "Corner1 (BL)", "Corner2 (TL)", "Corner3 (BR)", "Corner4 (TR)",
        "Left edge", "Right edge", "Bottom edge", "Top edge"
    };
    const std::vector<std::string> &labels = (phase == 0) ? ratioPhaseLabels : linearPhaseLabels;
    std::cout << "Place " << weight << "kg on the plate at position " << placementIndex+1
               << ": " << labels[placementIndex] << " and press 3 to continue..." << std::flush;
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

//Poor results. 
void CalibrateCOP::computeLinearFit(){
    // Per-axis (ML/AP) least-squares line: known = slope*measured + intercept, solved via QR
    int n = placementValues.size();
    Eigen::MatrixXd Ax(n, 2), Ay(n, 2);
    Eigen::VectorXd bx(n), by(n);

    for (int i=0; i < n; i++){
        Ax(i, 0) = placementValues[i][0]; Ax(i, 1) = 1.0; bx(i) = knownPositions[i][0];
        Ay(i, 0) = placementValues[i][1]; Ay(i, 1) = 1.0; by(i) = knownPositions[i][1];
    }

    Eigen::Vector2d solX = Ax.householderQr().solve(bx);
    Eigen::Vector2d solY = Ay.householderQr().solve(by);
    double xSlope = solX(0), xIntercept = solX(1);
    double ySlope = solY(0), yIntercept = solY(1);

    robot->setCOPLinearCalibration(xSlope, xIntercept, ySlope, yIntercept);

    std::cout << "Linear fit (ML/AP): x slope=" << xSlope << " intercept=" << xIntercept
               << " | y slope=" << ySlope << " intercept=" << yIntercept << '\n';
}

void CalibrateCOP::beginRatioCalibration(){
    phase = 0;
    placementIndex = 0;
    placementValues.clear();

    //Center origin y-up and x-right normalized positions.
    knownPositions = {
        VF2(-1, -1), VF2(-1, 1), VF2(1, -1), VF2(1, 1) // corners same order as forceplate.h (gotta be a better way than this!) BUG-PRONE
        };
    
    // This code block saves any previous linear calib before reset.
    Eigen::VectorXd linearVals = robot->getCOPLinearRegression();
    savedSlopes = VF2(linearVals(0), linearVals(2));
    savedIntercepts = VF2(linearVals(1), linearVals(3));
    robot->setCOPLinearCalibration(1.0, 0.0, 1.0, 0.0); //reset to default (perfect) assumption

    robot->setCOPRatios(VF4(-1, -1, 1, 1), VF4(-1.357, 1.363, -1.357, 1.363)); //reset to default (x, y)
    waitingForUser = true;
    std::cout << "\nRatio correction phase: 4-corner calibration\n";
    promptNextPlacement();
}

//Results are not great. seems to fit to noise (ratio error is only 0.01) frequently. More points?
void CalibrateCOP::beginLinearCalibration(){
    phase = 1;
    placementIndex = 0;
    placementValues.clear();
    knownPositions = {
        VF2(0, 0),                                       // center
        VF2(-1, -1), VF2(-1, 1), VF2(1, -1), VF2(1, 1),  // corners, same order/convention as phase 1
        VF2(-1, 0), VF2(1, 0), VF2(0, -1), VF2(0, 1)      // left, right, bottom, top edges
    };
    robot->setCOPLinearCalibration(1.0, 0.0, 1.0, 0.0); //reset to default (perfect) assumption
    waitingForUser = true;
    std::cout << "\nLinear correction phase: 9 point calibration\n";
    promptNextPlacement();
}