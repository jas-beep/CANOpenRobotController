#include "InitState.h"
InitState::InitState(ForcePlateMaster *robot, const char *name) : State(name), robot(robot) {}
void InitState::entry(void) { spdlog::info("InitState entry - press A to continue"); }