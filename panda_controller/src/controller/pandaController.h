#ifndef PANDA_CONTROLLER
#define PANDA_CONTROLLER

#include <controller/controller.hpp>
#include <trajectory/trajectory.hpp>
#include <panda_controller/panda_controller_paramConfig.h>
#include <panda_controller/paramForDebug.h>

// #define DIM 7
typedef my_robot::Robot<DIM> Robot7;
typedef robot_controller::Controller<DIM, panda_controller::paramForDebug, panda_controller::panda_controller_paramConfig> Robot7Controller;

namespace panda_controller
{
    //**//
    // 计算力矩控制器
    class ComputedTorqueMethod : public robot_controller::Controller<DIM, panda_controller::paramForDebug, panda_controller::panda_controller_paramConfig>
    {
    public:
        Eigen::Matrix<double, DIM, DIM> Kv;
        Eigen::Matrix<double, DIM, DIM> Kp;

        Eigen::Matrix<double, DIM, DIM> Kv_d;
        Eigen::Matrix<double, DIM, DIM> Kp_d;

    public:
        void calDesire(my_robot::Robot<DIM> *robot);
        bool setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d);

        void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config);
        void controllerParamRenew();
        void pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<DIM> *robot);
    };

    //**//
    // 反步控制器
    class Backstepping : public robot_controller::Controller<DIM, panda_controller::paramForDebug, panda_controller::panda_controller_paramConfig>
    {
    };
    //**//
    // 回零控制器
    class MoveZero : public robot_controller::Controller<DIM, panda_controller::paramForDebug, panda_controller::panda_controller_paramConfig>
    {
    };
};

//**//
extern Robot7 *pPanda;
extern Robot7Controller *pController;
void pandaInit();
void pandaStart(Eigen::Matrix<double, DIM, 1> q0, int recordPeriod);
void pandaRun(Eigen::Matrix<double, DIM, 1> q, Eigen::Matrix<double, DIM, 1> dq, Eigen::Matrix<double, DIM, 1> tau, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Affine3d TO2E, Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug);
void pandaGetDyn(Eigen::Matrix<double, 7, 7> M, Eigen::Matrix<double, 7, 1> C, Eigen::Matrix<double, 7, 1> G, Eigen::Matrix<double, 6, 7> J);
#endif