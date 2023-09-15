#ifndef PANDA_CONTROLLER
#define PANDA_CONTROLLER

#include "controller.hpp"
#include <panda_controller/panda_controller_paramConfig.h>
#include <panda_controller/paramForDebug.h>

// #define DIM 7
typedef my_robot::Robot<DIM> Robot7;
typedef robot_controller::Controller<DIM, panda_controller::paramForDebug> PandaController;
typedef PD<DIM> PandaPDController;
typedef Backstepping<DIM> PandaBacksteppingController;
typedef ComputedTorqueMethod<DIM> PandaComputedTorqueMethodController;

extern Robot7 *pPanda;
extern PandaController *pController;
void pandaInit(std::string &urdfPath, std::string &TcpName);
void pandaStart(const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &theta,
                const Eigen::Matrix<double, DIM, 1> &dq, const Eigen::Matrix<double, DIM, 1> &tau,
                const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E, int recordPeriod);

void pandaRun(const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq, const Eigen::Matrix<double, DIM, 1> &theta,
              const Eigen::Matrix<double, DIM, 1> &tau, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,
              const Eigen::Affine3d &TO2E, Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug,
              Eigen::Matrix<double, DIM, 1> &q_d);

void pandaGetDyn(const Eigen::Matrix<double, 7, 7> &M, const Eigen::Matrix<double, 7, 1> &c,
                 const Eigen::Matrix<double, 7, 1> &G, const Eigen::Matrix<double, 6, 7> &J);

#endif