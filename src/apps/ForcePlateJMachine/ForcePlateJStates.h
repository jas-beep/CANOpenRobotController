/**
 * \file ForcePlateJStates.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2025-04-14
 *
 * \copyright Copyright (c) 2025
 *
 */

#ifndef FORCEPLATEJ_H
#define FORCEPLATEJ_H

#include "State.h"
#include "ForcePlate.h"

class ForcePlateJMachine;

/**
 * \brief Generic state type including a pointer to ForcePlate
 *
 */
class ForcePlateState : public State
{
protected:
    ForcePlate *robot; //!< Pointer to state machines robot object

    ForcePlateState(ForcePlate *_robot, const char *name = NULL) : State(name), robot(_robot) { spdlog::debug("Created ForcePlateState {}", name); };
};

class StandbyState : public ForcePlateState
{

public:
    StandbyState(ForcePlate *_robot, const char *name = "Standby") : ForcePlateState(_robot, name) {};

    void entry(void);
    void during(void);
    void exit(void);
};

/**
 * \brief zero tare calibration state for the force plate.
 *
 */
class CalibState : public ForcePlateState
{

public:
    CalibState(ForcePlate *_robot, const char *name = "Calibration") : ForcePlateState(_robot, name) {};

    void entry(void);
    void during(void);
    void exit(void);

    bool isCalibDone() { return calibDone; }

private:
    bool calibDone = false;
    std::vector<VF4i> calibValues;
    u_int nbCalibValues;
};

class SetScale : public ForcePlateState
{

public:
    SetScale(ForcePlate *_robot, double weightKg, const char *name = "Set Scale") : ForcePlateState(_robot, name), weight(weightKg) {};

    void entry(void);
    void during(void);
    void exit(void);

    bool isWeightedCalibDone() { return weightedCalibDone; }

private:
    bool weightedCalibDone = false;
    bool waitingForUser = true;
    // int currentGauge = 0;
    VF4 scaleFactors = VF4::Zero();
    double weight; //!< Calibration weight in kg, passed from ForcePlateJMachine constructor
    std::vector<VF4i> rawADCwithWeight;
    u_int nbWeightedCalibValues;
};

class SetScalePerCorner : public ForcePlateState
{

public:
    SetScalePerCorner(ForcePlate *_robot, double weightKg, const char *name = "Set Scale Per Corner") : ForcePlateState(_robot, name), weight(weightKg) {};

    void entry(void);
    void during(void);
    void exit(void);

    bool isPerCornerCalibDone() { return perCornerCalibDone; }

private:
    bool perCornerCalibDone = false;
    bool waitingForUser = true;
    int currentGauge = 0;
    VF4 scaleFactors = VF4::Zero();
    double weight; //!< Calibration weight in kg, passed from ForcePlateJMachine constructor
    std::vector<VF4i> rawADCwithWeight;
    u_int nbWeightedCalibValues;
};

class CalibrateCOP : public ForcePlateState
{

public:
    CalibrateCOP(ForcePlate *_robot, double weightKg, const char *name = "Calibrate COP") : ForcePlateState(_robot, name), weight(weightKg) {};

    void entry(void);
    void during(void);
    void exit(void);

    bool isCalibDone() { return calibDone; }

private:
    bool calibDone = false;
    bool waitingForUser = true;

    std::vector<VF2> calibValues;      // current placement only, gets cleared.
    std::vector<VF2> placementValues;  // averaged VF4 for each placement, index i <-> knownPositions[i] - used to validate sensorXRatio/sensorYRatio in ForcePlate.h
    std::vector<VF2> knownPositions;   // known positions for each placement

    u_int nbCalibValues;
    int placementIndex = 0;
    int rejectedSamples = 0;  // implausible-force readings skipped during the current placement's collection
    double weight; // TODO: allow user to specify weight for COP calibration, or use default value

    void computeCOPRatio();
    // Regression fit COP - superseded by ForcePlate::sensorXRatio/sensorYRatio, kept for reference/comparison
    // VF4 xCoefficients, yCoefficients;
    // double xIntercept, yIntercept;
    // void fitRegression();
    // void fitScaleFactors();
};
#endif
