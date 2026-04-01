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
#include "ForcePlate4.h"


class ForcePlateJMachine;

/**
 * \brief Generic state type including a pointer to ForcePlate4
 *
 */
class ForcePlate4State : public State {
   protected:
    ForcePlate4 * robot;                               //!< Pointer to state machines robot object

    ForcePlate4State(ForcePlate4* _robot, const char *name = NULL): State(name), robot(_robot){spdlog::debug("Created ForcePlate4State {}", name);};
};


class StandbyState : public ForcePlate4State {

   public:
    StandbyState(ForcePlate4 * _robot, const char *name = "ForcePlate4 Standby"):ForcePlate4State(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);
};



/**
 * \brief Position calibration example. Go to stops of robot at constant torque for absolute position calibration.
 *
 */
class CalibState : public ForcePlate4State {

   public:
    CalibState(ForcePlate4 * _robot, const char *name = "ForcePlate4 Standby"):ForcePlate4State(_robot, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isCalibDone() {return calibDone;}

   private:
     double stop_reached_time;
     bool at_stop;
     bool calibDone=false;

};

#endif
