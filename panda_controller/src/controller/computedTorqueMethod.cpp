#include "pandaController.h"

namespace panda_controller
{
    ComputedTorqueMethod::ComputedTorqueMethod(TaskSpace taskSpace)
    {
        this->taskSpace = taskSpace;
    }

    void ComputedTorqueMethod::setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d_in)
    {
        if (taskSpace == jointSpace)
        {
            this->qc = this->ddq_d + jointKp * this->jointError + jointKv * this->djointError;
            this->tau_d << robot->getM() * (this->qc) + robot->getC() * robot->getdq() /* + G */;
            tau_d_in = this->tau_d;
        }
        else
        {
            // Eigen::Matrix<double, 6, 1> ddX;
            // ddX = ddX_d - cartesianKp_d * cartesianError - cartesianKv_d * dcartesianError;
            // this->tau_d << robot->getM() * (robot->getJ_inv() * (ddX - robot->getExternJ() * robot->getdq())) + robot->getC() * robot->getdq() /* + G */;
            // tau_d_in = this->tau_d;
        }
    }
    void ComputedTorqueMethod::calDesire(my_robot::Robot<DIM> *robot)
    {
        if (taskSpace == jointSpace)
        {
            double TPP = 0.3; // 位置参数，最大为1
            double TVP = 0.4; // 速度参数，最大为1
            Eigen::Matrix<double, 7, 1> deltaAngle;
            Eigen::Matrix<double, 7, 1> dDeltaAngle;
            Eigen::Matrix<double, 7, 1> ddDeltaAngle;
            Eigen::Matrix<double, 7, 1> selectAxis;
            selectAxis << 1, 1, 1, 1, 1, 1, 1; // 设置0，1，选择运动轴

            JointSinTrajectory<7>(selectAxis, this->time / 1000.0, TPP, TVP, deltaAngle, dDeltaAngle, ddDeltaAngle);

            this->q_d = robot->getq0() + deltaAngle;
            this->dq_d = dDeltaAngle;
            this->ddq_d = ddDeltaAngle;
        }
        else
        {
            this->position_d = this->position_d;
            this->orientation_d = this->orientation_d;
            this->dposition_d = this->dposition_d;
            this->dposition_d = this->dposition_d;
            this->ddX_d = this->ddX_d;
        }
    }
    void ComputedTorqueMethod::dynamicSetParameter(panda_controller::panda_controller_paramConfig &config)
    {
        if (taskSpace == jointSpace)
        {
            std::array<double, 7> Kp_values = {config.Kp1, config.Kp2, config.Kp3, config.Kp4, config.Kp5, config.Kp6, config.Kp7};
            std::array<double, 7> Kv_values = {config.Kv1, config.Kv2, config.Kv3, config.Kv4, config.Kv5, config.Kv6, config.Kv7};
            if (this->time == 0)
            {
                for (int i = 0; i < 7; i++)
                {
                    jointKp(i, i) = Kp_values[i];
                    jointKv(i, i) = Kv_values[i];
                }
            }
            for (int i = 0; i < 7; i++)
            {
                jointKp_d(i, i) = Kp_values[i];
                jointKv_d(i, i) = Kv_values[i];
            }
        }
        else
        {
            if (this->time == 0)
            {
                cartesianKp.topLeftCorner(3, 3) << config.Kp_pos * Eigen::Matrix3d::Identity();
                cartesianKp.bottomRightCorner(3, 3) << config.Kp_ori * Eigen::Matrix3d::Identity();

                cartesianKv.topLeftCorner(3, 3) << config.Kv_pos * Eigen::Matrix3d::Identity();
                cartesianKv.bottomRightCorner(3, 3) << config.Kv_ori * Eigen::Matrix3d::Identity();
            }
            else
            {
                cartesianKp_d.topLeftCorner(3, 3) << config.Kp_pos * Eigen::Matrix3d::Identity();
                cartesianKp_d.bottomRightCorner(3, 3) << config.Kp_ori * Eigen::Matrix3d::Identity();

                cartesianKv_d.topLeftCorner(3, 3) << config.Kv_pos * Eigen::Matrix3d::Identity();
                cartesianKv_d.bottomRightCorner(3, 3) << config.Kv_ori * Eigen::Matrix3d::Identity();
            }
        }
    }
    void ComputedTorqueMethod::controllerParamRenew()
    {
        this->jointKp = this->filterParams * this->jointKp_d + (1.0 - this->filterParams) * this->jointKp;
        this->jointKv = this->filterParams * this->jointKv_d + (1.0 - this->filterParams) * this->jointKv;

        this->cartesianKp = this->filterParams * this->cartesianKp_d + (1.0 - this->filterParams) * this->cartesianKp;
        this->cartesianKv = this->filterParams * this->cartesianKv_d + (1.0 - this->filterParams) * this->cartesianKv;
    }
    void ComputedTorqueMethod::pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<DIM> *robot)
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
}