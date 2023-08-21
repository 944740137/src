#include "robotController/pandaController.h"

namespace panda_controller
{
    PD::~PD()
    {
    }
    PD::PD(TaskSpace taskSpace) : jointKv(Eigen::Matrix<double, DIM, DIM>::Zero()),
                                  jointKp(Eigen::Matrix<double, DIM, DIM>::Zero()),
                                  jointKv_d(Eigen::Matrix<double, DIM, DIM>::Zero()),
                                  jointKp_d(Eigen::Matrix<double, DIM, DIM>::Zero()),
                                  cartesianKp(Eigen::Matrix<double, 6, 6>::Zero()),
                                  cartesianKv(Eigen::Matrix<double, 6, 6>::Zero()),
                                  cartesianKp_d(Eigen::Matrix<double, 6, 6>::Zero()),
                                  cartesianKv_d(Eigen::Matrix<double, 6, 6>::Zero())
    {
        this->taskSpace = taskSpace;
        this->controllerLawName = "PD";
        std::cout << "[robotController] 设置控制律: " << controllerLawName << std::endl;
    }

    void PD::setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d_in)
    {
        if (taskSpace == jointSpace)
        {
            this->tau_d << this->ddq_d + jointKp * this->jointError + jointKv * this->djointError /* + G */;
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

    void PD::dynamicSetParameter(panda_controller::panda_controller_paramConfig &config, unsigned int time)
    {
        if (taskSpace == jointSpace)
        {
            std::array<double, 7> Kp_values = {config.Kp1, config.Kp2, config.Kp3, config.Kp4, config.Kp5, config.Kp6, config.Kp7};
            std::array<double, 7> Kv_values = {config.Kv1, config.Kv2, config.Kv3, config.Kv4, config.Kv5, config.Kv6, config.Kv7};
            if (time == 0)
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
            if (time == 0)
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
    void PD::controllerParamRenew(double filterParams)
    {
        this->jointKp = filterParams * this->jointKp_d + (1.0 - filterParams) * this->jointKp;
        this->jointKv = filterParams * this->jointKv_d + (1.0 - filterParams) * this->jointKv;

        this->cartesianKp = filterParams * this->cartesianKp_d + (1.0 - filterParams) * this->cartesianKp;
        this->cartesianKv = filterParams * this->cartesianKv_d + (1.0 - filterParams) * this->cartesianKv;
    }

}