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
class ForcePlateState : public State {
   protected:
    ForcePlate * robot;                               //!< Pointer to state machines robot object

    ForcePlateState(ForcePlate* _robot, const char *name = NULL): State(name), robot(_robot){spdlog::debug("Created ForcePlateState {}", name);};
};


class StandbyState : public ForcePlateState {

   public:
    StandbyState(ForcePlate * _robot, const char *name = "Standby"):ForcePlateState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);
};



/**
 * \brief Position calibration example. Go to stops of robot at constant torque for absolute position calibration.
 *
 */
class CalibState : public ForcePlateState {

   public:
    CalibState(ForcePlate * _robot, const char *name = "Calibration"):ForcePlateState(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isCalibDone() {return calibDone;}

   private:
    bool calibDone=false;
    std::vector<VF4i> calibValues;
    u_int nbCalibValues;
};

#endif
