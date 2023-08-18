#ifndef PANDA_CONTROLLER
#define PANDA_CONTROLLER

#include "controller.hpp"
// #include <trajectory/trajectory.hpp>
#include <panda_controller/panda_controller_paramConfig.h>
#include <panda_controller/paramForDebug.h>

// #define DIM 7
typedef my_robot::Robot<DIM> Robot7;
typedef robot_controller::Controller<DIM, panda_controller::paramForDebug, panda_controller::panda_controller_paramConfig> Robot7Controller;

namespace panda_controller
{
    //**//
    // 计算力矩控制器
    class ComputedTorqueMethod : public robot_controller::ControllerLaw<DIM, panda_controller::panda_controller_paramConfig>
    {
    public:
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

        void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config, unsigned int time);
        void controllerParamRenew(double filterParams);
    };

    //**//
    // 反步控制器
    class Backstepping : public robot_controller::ControllerLaw<DIM, panda_controller::panda_controller_paramConfig>
    {
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

        void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config, unsigned int time);
        void controllerParamRenew(double filterParams);
    };

    //**//
    // PD+重力补偿
    class PD : public robot_controller::ControllerLaw<DIM, panda_controller::panda_controller_paramConfig>
    {
    public:
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

        void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config, unsigned int time);
        void controllerParamRenew(double filterParams);
    };

    //**//
    // 回零控制器
};

//**//

extern Robot7 *pPanda;
extern Robot7Controller *pController;
void pandaInit();
void pandaStart(const Eigen::Matrix<double, DIM, 1> &q0, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, int recordPeriod);
void pandaRun(const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq, const Eigen::Matrix<double, DIM, 1> &theta, const Eigen::Matrix<double, DIM, 1> &tau, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E, Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug, Eigen::Matrix<double, DIM, 1> &q_d);
void pandaGetDyn(const Eigen::Matrix<double, 7, 7> &M, const Eigen::Matrix<double, 7, 1> &c, const Eigen::Matrix<double, 7, 1> &G, const Eigen::Matrix<double, 6, 7> &J);

#endif