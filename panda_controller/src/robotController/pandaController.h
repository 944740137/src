#ifndef PANDA_CONTROLLER
#define PANDA_CONTROLLER

#include "controller.hpp"
#include <trajectory/trajectory.hpp>
#include <panda_controller/panda_controller_paramConfig.h>
#include <panda_controller/paramForDebug.h>

enum TaskSpace
{
    jointSpace,
    cartesianSpace
};

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
        TaskSpace taskSpace;
        // 关节空间
        Eigen::Matrix<double, DIM, DIM> jointKv;
        Eigen::Matrix<double, DIM, DIM> jointKp;

        Eigen::Matrix<double, DIM, DIM> jointKv_d;
        Eigen::Matrix<double, DIM, DIM> jointKp_d;

        // 笛卡尔空间
        Eigen::Matrix<double, 6, 6> cartesianKp;
        Eigen::Matrix<double, 6, 6> cartesianKv;

        Eigen::Matrix<double, 6, 6> cartesianKp_d;
        Eigen::Matrix<double, 6, 6> cartesianKv_d;

    public:
        ComputedTorqueMethod(TaskSpace taskSpace);
        void calDesire(my_robot::Robot<DIM> *robot);
        void setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d);

        void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config);
        void controllerParamRenew();
        void pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<DIM> *robot);
    };

    //**//
    // 反步控制器
    class Backstepping : public robot_controller::Controller<DIM, panda_controller::paramForDebug, panda_controller::panda_controller_paramConfig>
    {
        TaskSpace taskSpace;
        // 关节空间
        Eigen::Matrix<double, DIM, DIM> jointK1;
        Eigen::Matrix<double, DIM, DIM> jointK2;

        Eigen::Matrix<double, DIM, DIM> jointK1_d;
        Eigen::Matrix<double, DIM, DIM> jointK2_d;

        // 笛卡尔空间
        Eigen::Matrix<double, 6, 6> cartesianK1;
        Eigen::Matrix<double, 6, 6> cartesianK2;

        Eigen::Matrix<double, 6, 6> cartesianK1_d;
        Eigen::Matrix<double, 6, 6> cartesianK2_d;

        // 误差中间变量
        Eigen::Matrix<double, 7, 1> e1;
        Eigen::Matrix<double, 7, 1> e2;
        Eigen::Matrix<double, 7, 1> r;
        Eigen::Matrix<double, 7, 1> dr;

    public:
        Backstepping(TaskSpace taskSpace);
        void calDesire(my_robot::Robot<DIM> *robot);
        void setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d);

        void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config);
        void controllerParamRenew();
        void pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<DIM> *robot);
    };

    //**//
    // PD+重力补偿
    class PD : public robot_controller::Controller<DIM, panda_controller::paramForDebug, panda_controller::panda_controller_paramConfig>
    {
    public:
        TaskSpace taskSpace;
        // 关节空间
        Eigen::Matrix<double, DIM, DIM> jointKv;
        Eigen::Matrix<double, DIM, DIM> jointKp;

        Eigen::Matrix<double, DIM, DIM> jointKv_d;
        Eigen::Matrix<double, DIM, DIM> jointKp_d;

        // 笛卡尔空间
        Eigen::Matrix<double, 6, 6> cartesianKp;
        Eigen::Matrix<double, 6, 6> cartesianKv;

        Eigen::Matrix<double, 6, 6> cartesianKp_d;
        Eigen::Matrix<double, 6, 6> cartesianKv_d;

    public:
        PD(TaskSpace taskSpace);
        void calDesire(my_robot::Robot<DIM> *robot);
        void setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d);

        void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config);
        void controllerParamRenew();
        void pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<DIM> *robot);
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
void pandaStart(Eigen::Matrix<double, DIM, 1> q0, Eigen::Vector3d position, Eigen::Quaterniond orientation, int recordPeriod);
void pandaRun(Eigen::Matrix<double, DIM, 1> q, Eigen::Matrix<double, DIM, 1> dq, Eigen::Matrix<double, DIM, 1> tau, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Affine3d TO2E, Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug);
void pandaGetDyn(Eigen::Matrix<double, 7, 7> M, Eigen::Matrix<double, 7, 1> C, Eigen::Matrix<double, 7, 1> G, Eigen::Matrix<double, 6, 7> J);
#endif