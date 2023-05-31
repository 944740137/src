#include <algorithm/pinocchino_interactive.h>
#include "controller/controller.h"

// PandaController *pController = nullptr;
// Panda *pPanda = nullptr;

namespace my_controller
{
    // void Controller::startControllerManager()
    // {
    //     if (pinInteractive == nullptr)
    //         pinInteractive = new pinLibInteractive();
    // }
    template <int _Dofs>
    void Controller<_Dofs>::calError(my_robot::Robot<_Dofs> *robot)
    {
        this->jointError = q_d - robot->getq();
        this->djointError = dq_d - robot->getdq();

        this->cartesianError.head(3) = position_d - robot->getPosition();
        if (orientation_d.coeffs().dot(robot->getOrientation().coeffs()) < 0.0)
        {
            robot->getOrientation().coeffs() << -robot->getOrientation().coeffs();
        }
        Eigen::Quaterniond error_quaternion(robot->getOrientation().inverse() * orientation_d);
        this->cartesianError.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        this->cartesianError.tail(3) << robot->getT().rotation() * this->cartesianError.tail(3);
    }
    template <int _Dofs>
    void Controller<_Dofs>::updateTime()
    {
        this->time++;
    }
    template <int _Dofs>
    void Controller<_Dofs>::setRecord(int record)
    {
        this->recordPeriod = record;
    }

    // 可重写的函数
    template <int _Dofs>
    void Controller<_Dofs>::recordData(my_robot::Robot<_Dofs> *robot)
    {
        if (recordPeriod == 0)
            return;
        if (time == 1)
        {
            myfile.open("/home/wd/Controller.txt");
            myfile << "Controller" << std::endl;
        }
        else
        {
            myfile << "time: " << time << "_" << std::endl;
            myfile << "q: " << robot->q.transpose() << std::endl;
            myfile << "dq: " << robot->dq.transpose() << std::endl;
            myfile << "q_d: " << q_d.transpose() << std::endl;
            myfile << "dq_d: " << dq_d.transpose() << std::endl;
        }
    }
    template <int _Dofs>
    void Controller<_Dofs>::pubData(main_controller::paramForDebug &param_debug, my_robot::Robot<_Dofs> *robot)
    {
        for (int i = 0; i < _Dofs; i++)
        {
            param_debug.jointError[i] = this->jointError[i];
            param_debug.q_d[i] = this->q_d[i];
            param_debug.q[i] = this->q[i];
            // param_debug.tau_d[i] = this->tau_d[i];
        }
        for (int i = 0; i < 6; i++)
        {
            param_debug.cartesianError[i] = this->cartesianError[i];
        }
        for (int i = 0; i < 3; i++)
        {
            param_debug.position[i] = this->position[i];
            param_debug.position_d[i] = this->position_d[i];
            // param_debug.orientation[i] = this->orientation[i];
            // param_debug.orientation_d[i] = this->orientation_d[i];
        }
    }

    // 计算力矩法
    // 轨迹与控制律计算 保存
    template <int _Dofs>
    void ComputedTorqueMethod<_Dofs>::dynamicSetParameter(main_controller::main_controller_paramConfig &config)
    {
        Kv_d = config.Kv * Eigen::MatrixXd::Identity(_Dofs, _Dofs);
        Kp_d = config.Kp * Eigen::MatrixXd::Identity(_Dofs, _Dofs);
    }
    template <int _Dofs>
    void ComputedTorqueMethod<_Dofs>::controllerParamRenew()
    {
        Kv = this->filterParams * Kv_d + (1.0 - this->filterParams) * Kv;
        Kp = this->filterParams * Kp_d + (1.0 - this->filterParams) * Kp;
    }
    template <int _Dofs>
    bool ComputedTorqueMethod<_Dofs>::setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d)
    {
        this->qc = /* ddq_d */ +Kp * this->jointError + Kv * this->jointError;
        this->tau_d << robot->getM() * (this->qc) + robot->getC() /* + G */;
        tau_d = this->tau_d;
        return true;
    }
    template <int _Dofs>
    void ComputedTorqueMethod<_Dofs>::calDesire(my_robot::Robot<_Dofs> *robot)
    {
        double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * this->time)) * 0.2;
        double dot_delta_angle = M_PI / 8 * M_PI / 5 * (std::sin(M_PI / 5.0 * this->time)) * 0.2;
        double ddot_delta_angle = M_PI / 8 * M_PI / 5 * M_PI / 5 * (std::cos(M_PI / 5.0 * this->time)) * 0.2;
        for (size_t i = 0; i < 7; ++i)
        {
            if (i == 4)
            {
                this->q_d[i] = robot->q0[i] - delta_angle;
                this->dq_d[i] = -dot_delta_angle;
                this->ddq_d[i] = -ddot_delta_angle;
            }
            else
            {
                this->q_d[i] = robot->q0[i] + delta_angle;
                this->dq_d[i] = dot_delta_angle;
                this->ddq_d[i] = ddot_delta_angle;
            }
        }
    }
    template <int _Dofs>
    void ComputedTorqueMethod<_Dofs>::recordData(my_robot::Robot<_Dofs> *robot)
    {
        if (recordPeriod == 0)
            return;
        if (time == 1)
        {
            myfile.open("/home/wd/Controller.txt");
            myfile << "Controller" << std::endl;
        }
        else
        {
            myfile << "time: " << time << "_" << std::endl;
            myfile << "q: " << robot->q.transpose() << std::endl;
            myfile << "dq: " << robot->dq.transpose() << std::endl;
            myfile << "q_d: " << q_d.transpose() << std::endl;
            myfile << "dq_d: " << dq_d.transpose() << std::endl;
        }
    }
};

template <int _Dofs>
void robotRun(Eigen::Matrix<double, _Dofs, 1> q, Eigen::Matrix<double, _Dofs, 1> dq, Eigen::Matrix<double, _Dofs, 1> tau, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Affine3d TO2E, Eigen::Matrix<double, _Dofs, 1> &tau_d)
{

    pPanda->updateJointData(q, dq, tau);
    pPanda->updateEndeffectorData(position, orientation, TO2E);
    pPanda->calculation(pController->ddq_d);

    pController->updateTime();
    pController->calDesire(pPanda);
    pController->calError(pPanda);
    pController->setControllerLaw(pPanda, tau_d);

    pController->recordData(pPanda);
}
template <int _Dofs>
void robotStart(Eigen::Matrix<double, _Dofs, 1> q0, int recordPeriod)
{
    std::cout << "--------------robotStart--------------" << std::endl;
    if (pController == nullptr)
    {
        pController = new PandaController1();
    }
    if (pPanda == nullptr)
    {
        pPanda = new Panda();
    }
    pController->setRecord(recordPeriod);
    pPanda->setq0(q0);
    std::cout << "--------------robotStart over--------------" << std::endl;
}
