#ifndef UR5_CONTROLLER  
#define UR5_CONTROLLER 

#include <controller/controller.hpp>
// #include <panda_controller/panda_controller_paramConfig.h>
// #include <panda_controller/paramForDebug.h>

#define DIM 6
typedef my_robot::Robot<DIM> Robot6;
// typedef my_controller::Controller<DIM, panda_controller::paramForDebug, panda_controller::panda_controller_paramConfig> Robot6Controller;