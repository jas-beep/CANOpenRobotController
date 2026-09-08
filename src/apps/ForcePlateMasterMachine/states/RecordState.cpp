#include "RecordState.h"
#include <iostream>
#include <iomanip>

RecordState::RecordState(ForcePlateMaster *robot, const char *name) : State(name), robot(robot) {}

void RecordState::entry(void) {
    spdlog::info("RecordState entry - S to stop");
    robot->startStreaming();
}

void RecordState::during(void) {
    if (iterations() % 50 == 1) {
        std::cout << std::setprecision(3) << std::fixed << std::showpos;
        for (size_t i = 0; i < robot->numPlates(); i++) {
            std::cout << "P" << i << " F=" << robot->getSummedForce(i) << "\t";
        }
        std::cout << "GlobalCoP=[ " << robot->getGlobalCoP().transpose() << " ]" << std::endl;
        std::cout << std::noshowpos << std::defaultfloat << std::setprecision(6);
    }
}

void RecordState::exit(void) {
    robot->stopStreaming();
    spdlog::info("RecordState exit");
}

