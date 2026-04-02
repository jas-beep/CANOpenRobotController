/**
 * \file ForcePlateJMachine.h
 * \author Vincent Crocher
 * \brief The ForcePlateJMachine class represents an example implementation of a state machine.
 * \version 0.1
 * \date 2025-04-14
 *
 * \copyright Copyright (c) 2025
 *
 */
#ifndef FORCEPLATEJMACHINE_H
#define FORCEPLATEJMACHINE_H


#include "StateMachine.h"
#include "ForcePlate4.h"
#include "FLNLHelper.h"

// State Classes
#include "ForcePlateJStates.h"


/**
 * StateMachine intended to run on a PocketBeagle connected to 4 HX711 and straingauge to act as a force plate.
 *
 */
class ForcePlateJMachine : public StateMachine {

   public:
    ForcePlateJMachine();
    ~ForcePlateJMachine();
    void init();
    void end();

    void hwStateUpdate();

    ForcePlate4 *robot() { return static_cast<ForcePlate4*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server
};

#endif
