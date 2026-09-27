#include "IdleState.h"
#include <iostream>
#include <iomanip>
IdleState::IdleState(ForcePlateMaster *robot, const char *name) : State(name), robot(robot) {}
void IdleState::entry(void) { spdlog::info("IdleState entry: S to record, getNB to select plate, W to trigger COP calibration, X to advance placement, T to tare selected plate, Z to tare all plates"); }

void IdleState::during(void) {
    
    if (iterations() % 10 == 1) {
        std::cout << std::setprecision(3) << std::fixed << std::showpos;
        std::cout << "Plate " << activePlate << " F=[ " << robot->getForces(activePlate).transpose()
                   << " ] CoP=[ " << robot->getCOP(activePlate).transpose() << " ]";
        std::cout << std::noshowpos << std::defaultfloat << std::setprecision(6);
        if (robot->isCalibReady(activePlate)) {
            std::cout << " [READY for next placement]";
        } else if (robot->getSamplesCollected(activePlate) > 0) {
            std::cout << " [collecting " << robot->getSamplesCollected(activePlate) << "/200]";
        }
        std::cout << std::endl;
    }

    
    if (copCalibPulseCountdown > 0) {
        if (--copCalibPulseCountdown == 0) {
            robot->clearCommand(); // clears whichever broadcast command (COP_CALIB or TARE_CMD) was pulsing
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

    if (robot->keyboard->getKeyUC() == 'Z') {
        spdlog::info("Triggering tare on all plates");
        robot->triggerTare();
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

    if (robot->keyboard->getKeyUC() == 'T') {
        spdlog::info("Triggering tare on plate {}", activePlate);
        robot->triggerTare(activePlate);
        pulsingPlate = activePlate;
        calibPulseCountdown = 10;
    }

    if (robot->keyboard->getX()){
        if (robot->isCalibReady(activePlate)) {
            spdlog::info("Advancing placement on plate {}", activePlate);
            robot->advancePlacement(activePlate);
            pulsingPlate = activePlate;
            calibPulseCountdown = 10;
        } else {
            spdlog::info("Plate {} not ready yet ({}/200 samples collected) - ignoring X", activePlate, robot->getSamplesCollected(activePlate));
        }
    }
}