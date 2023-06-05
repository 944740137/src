#include "pandaController.h"

bool panda_controller::ComputedTorqueMethod::setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d_in)
{
    this->qc = this->ddq_d + Kp * this->jointError + Kv * this->djointError;
    this->tau_d << robot->getM() * (this->qc) + robot->getC() * robot->getdq() /* + G */;
    tau_d_in = this->tau_d;
    return true;
}
void panda_controller::ComputedTorqueMethod::calDesire(my_robot::Robot<DIM> *robot)
{
    double TPP = 0.3; // 位置参数，最大为1
    double TVP = 0.5; // 速度参数，最大为1
    Eigen::Matrix<double, 7, 1> deltaAngle;
    Eigen::Matrix<double, 7, 1> dDeltaAngle;
    Eigen::Matrix<double, 7, 1> ddDeltaAngle;
    Eigen::Matrix<double, 7, 1> selectAxis;
    selectAxis << 1, 0, 1, 1, 1, 1, 1; //设置0，1，选择运动轴

    JointCosTrajectory<7>(selectAxis, this->time / 1000.0, TPP, TVP, deltaAngle, dDeltaAngle, ddDeltaAngle);

    this->q_d = robot->getq0() + deltaAngle;
    this->dq_d = dDeltaAngle;
    this->ddq_d = ddDeltaAngle;
}
void panda_controller::ComputedTorqueMethod::pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<DIM> *robot)
{
    for (int i = 0; i < DIM; i++)
    {
        param_debug.jointError[i] = this->jointError[i];
        param_debug.q[i] = robot->getq()[i];
        param_debug.q_d[i] = this->q_d[i];
        param_debug.dq[i] = robot->getdq()[i];
        param_debug.dq_d[i] = this->dq_d[i];
        param_debug.tau_d[i] = this->tau_d[i];
    }
    for (int i = 0; i < 6; i++)
    {
        param_debug.cartesianError[i] = this->cartesianError[i];
    }
    for (int i = 0; i < 3; i++)
    {
        param_debug.position[i] = robot->getPosition()[i];
        param_debug.position_d[i] = this->position_d[i];
        param_debug.orientation[i] = robot->getOrientation().toRotationMatrix().eulerAngles(2, 1, 0)[i];
        param_debug.orientation_d[i] = this->orientation_d.toRotationMatrix().eulerAngles(2, 1, 0)[i];
    }
}
void panda_controller::ComputedTorqueMethod::dynamicSetParameter(panda_controller::panda_controller_paramConfig &config)
{
    std::array<double, 7> Kp_values = {config.Kp1, config.Kp2, config.Kp3, config.Kp4, config.Kp5, config.Kp6, config.Kp7};
    std::array<double, 7> Kv_values = {config.Kv1, config.Kv2, config.Kv3, config.Kv4, config.Kv5, config.Kv6, config.Kv7};

    if (this->time == 0)
    {
        for (int i = 0; i < 7; i++)
        {
            Kp(i, i) = Kp_values[i];
            Kv(i, i) = Kv_values[i];
        }
    }
    for (int i = 0; i < 7; i++)
    {
        Kp_d(i, i) = Kp_values[i];
        Kv_d(i, i) = Kv_values[i];
    }
}
void panda_controller::ComputedTorqueMethod::controllerParamRenew()
{
    Kv = this->filterParams * Kv_d + (1.0 - this->filterParams) * Kv;
    Kp = this->filterParams * Kp_d + (1.0 - this->filterParams) * Kp;
}
