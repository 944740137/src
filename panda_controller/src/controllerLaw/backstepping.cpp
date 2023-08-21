#include "robotController/pandaController.h"

namespace panda_controller
{
    Backstepping::~Backstepping()
    {
    }
    Backstepping::Backstepping(TaskSpace taskSpace) : jointK1(Eigen::Matrix<double, DIM, DIM>::Zero()),
                                                      jointK2(Eigen::Matrix<double, DIM, DIM>::Zero()),
                                                      jointK1_d(Eigen::Matrix<double, DIM, DIM>::Zero()),
                                                      jointK2_d(Eigen::Matrix<double, DIM, DIM>::Zero()),
                                                      cartesianK1(Eigen::Matrix<double, 6, 6>::Zero()),
                                                      cartesianK2(Eigen::Matrix<double, 6, 6>::Zero()),
                                                      cartesianK1_d(Eigen::Matrix<double, 6, 6>::Zero()),
                                                      cartesianK2_d(Eigen::Matrix<double, 6, 6>::Zero()),
                                                      e1(Eigen::Matrix<double, DIM, 1>::Zero()),
                                                      e2(Eigen::Matrix<double, DIM, 1>::Zero()),
                                                      r(Eigen::Matrix<double, DIM, 1>::Zero()),
                                                      dr(Eigen::Matrix<double, DIM, 1>::Zero())
    {
        this->taskSpace = taskSpace;
        this->controllerLawName = "Backstepping";
        std::cout << "[robotController] 设置控制律:" << controllerLawName << std::endl;
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

    void Backstepping::dynamicSetParameter(panda_controller::panda_controller_paramConfig &config, unsigned int time)
    {
        if (taskSpace == jointSpace)
        {
            std::array<double, 7> K1_values = {config.K1_1, config.K1_2, config.K1_3, config.K1_4, config.K1_5, config.K1_6, config.K1_7};
            std::array<double, 7> K2_values = {config.K2_1, config.K2_2, config.K2_3, config.K2_4, config.K2_5, config.K2_6, config.K2_7};
            if (time == 0)
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
    void Backstepping::controllerParamRenew(double filterParams)
    {
        this->jointK1 = filterParams * this->jointK1_d + (1.0 - filterParams) * this->jointK1;
        this->jointK2 = filterParams * this->jointK2_d + (1.0 - filterParams) * this->jointK2;

        this->cartesianK1 = filterParams * this->cartesianK1_d + (1.0 - filterParams) * this->cartesianK1;
        this->cartesianK2 = filterParams * this->cartesianK2_d + (1.0 - filterParams) * this->cartesianK2;
    }

}