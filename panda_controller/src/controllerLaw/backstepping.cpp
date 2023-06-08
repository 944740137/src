#include "robotController/pandaController.h"

namespace panda_controller
{
    Backstepping::Backstepping(TaskSpace taskSpace)
    {
        this->taskSpace = taskSpace;
        this->controllerLawName = "Backstepping";
        std::cout << "[robotController] controllerLawName:" << controllerLawName << std::endl;
    }

    void Backstepping::setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d_in)
    {
        if (taskSpace == jointSpace)
        {
            this->e1 = this->jointError;
            this->e2 = this->djointError + this->jointK1 * e1;
            this->r = this->dq_d + this->jointK1 * e1;
            this->dr = this->ddq_d + this->jointK1 * this->djointError;

            this->tau_d << robot->getM() * (this->dr) + robot->getC() * (this->dr) /* + G */ + this->jointK2 * this->e2 + this->e1;
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
    void Backstepping::calDesire(my_robot::Robot<DIM> *robot)
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
            // this->position_d = this->position_d;
            // this->orientation_d = this->orientation_d;
            // this->dposition_d = this->dposition_d;
            // this->dposition_d = this->dposition_d;
            // this->ddX_d = this->ddX_d;
        }
    }
    void Backstepping::dynamicSetParameter(panda_controller::panda_controller_paramConfig &config)
    {
        if (taskSpace == jointSpace)
        {
            std::array<double, 7> K1_values = {config.K1_1, config.K1_2, config.K1_3, config.K1_4, config.K1_5, config.K1_6, config.K1_7};
            std::array<double, 7> K2_values = {config.K2_1, config.K2_2, config.K2_3, config.K2_4, config.K2_5, config.K2_6, config.K2_7};
            if (this->time == 0)
            {
                for (int i = 0; i < 7; i++)
                {
                    jointK1(i, i) = K1_values[i];
                    jointK2(i, i) = K2_values[i];
                }
            }
            for (int i = 0; i < 7; i++)
            {
                jointK1_d(i, i) = K1_values[i];
                jointK2_d(i, i) = K2_values[i];
            }
        }
        else
        {
        }
    }
    void Backstepping::controllerParamRenew()
    {
        this->jointK1 = this->filterParams * this->jointK1_d + (1.0 - this->filterParams) * this->jointK1;
        this->jointK2 = this->filterParams * this->jointK2_d + (1.0 - this->filterParams) * this->jointK2;

        this->cartesianK1 = this->filterParams * this->cartesianK1_d + (1.0 - this->filterParams) * this->cartesianK1;
        this->cartesianK2 = this->filterParams * this->cartesianK2_d + (1.0 - this->filterParams) * this->cartesianK2;
    }
    void Backstepping::pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<DIM> *robot)
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