#include "IdleState.h"
#include <iostream>
#include <iomanip>
IdleState::IdleState(ForcePlateMaster *robot, const char *name) : State(name), robot(robot) {}
void IdleState::entry(void) { spdlog::info("IdleState entry: S to record, getNB to select plate, W to trigger COP calibration, X to advance placement"); }

void IdleState::during(void) {
    
    if (iterations() % 10 == 1) {
        std::cout << std::setprecision(3) << std::fixed << std::showpos;
        std::cout << "Plate " << activePlate << " F=[ " << robot->getForces(activePlate).transpose()
                   << " ] CoP=[ " << robot->getCoP(activePlate).transpose() << " ]" << std::endl;
        std::cout << std::noshowpos << std::defaultfloat << std::setprecision(6);
    }

    
    if (copCalibPulseCountdown > 0) {
        if (--copCalibPulseCountdown == 0) {
            robot->clearCommand();
        }
        return;
    }
    if (calibPulseCountdown > 0) {
        if (--calibPulseCountdown == 0) {
            robot->clearCalibCommand(pulsingPlate);
        }
        return; //don't accept a new addressed trigger while one is still pulsing
    }

    if (robot->keyboard->getD()) {
        spdlog::info("Triggering COP calibration on all plates");
        robot->triggerCOPCalibration();
        copCalibPulseCountdown = 10;
    }

    //to select plate
    int nb = robot->keyboard->getNb();
    if (nb >= 1 && (size_t)nb <= robot->numPlates()) {
        activePlate = (size_t)nb - 1;
        spdlog::info("Active plate set to {}", activePlate);
    }

    if (robot->keyboard->getW()){
        spdlog::info("Triggering COP calibration on plate {}", activePlate);
        robot->triggerCOPCalibration(activePlate);
        pulsingPlate = activePlate;
        calibPulseCountdown = 10;
    }

    if (robot->keyboard->getX()){
        spdlog::info("Advancing placement on plate {}", activePlate);
        robot->advancePlacement(activePlate);
        pulsingPlate = activePlate;
        calibPulseCountdown = 10;
    }
}