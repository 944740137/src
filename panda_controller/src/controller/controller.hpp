#pragma once
// #include <algorithm/pinocchino_interactive.h>

#include <fstream>
#include <iostream>
#include "robot/robot.hpp"

#include <panda_controller/panda_controller_paramConfig.h>
#include <panda_controller/paramForDebug.h>
namespace my_controller
{
    template <int _Dofs = 7>
    class Controller
    {

    public:
        // debug，绘图
        int recordPeriod = 1;
        double time = 0;
        std::ofstream myfile;
        double filterParams = 0.005;
        // error
        Eigen::Matrix<double, _Dofs, 1> jointError;
        Eigen::Matrix<double, _Dofs, 1> djointError;
        Eigen::Matrix<double, 6, 1> cartesianError;

        // controllerLaw
        Eigen::Matrix<double, _Dofs, 1> tau_d;
        Eigen::Matrix<double, _Dofs, 1> qc;

        // desire
        Eigen::Matrix<double, _Dofs, 1> q_d;
        Eigen::Matrix<double, _Dofs, 1> dq_d;
        Eigen::Matrix<double, _Dofs, 1> ddq_d;
        Eigen::Vector3d position_d;
        Eigen::Quaterniond orientation_d;

    public:
        void calError(my_robot::Robot<_Dofs> *robot);
        void updateTime();
        void setRecord(int record);

        // 可重写
        virtual void recordData(my_robot::Robot<_Dofs> *robot);
        virtual void pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<_Dofs> *robot);
        // 必须重写
        virtual void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config) = 0;
        virtual bool setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d) = 0;
        virtual void calDesire(my_robot::Robot<_Dofs> *robot) = 0;
        virtual void controllerParamRenew() = 0;
    };

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
            myfile << "time: " << this->time << "_" << std::endl;
            myfile << "q: " << robot->getq().transpose() << std::endl;
            myfile << "dq: " << robot->getdq().transpose() << std::endl;
            myfile << "q_d: " << q_d.transpose() << std::endl;
            myfile << "dq_d: " << dq_d.transpose() << std::endl;
        }
    }
    template <int _Dofs>
    void Controller<_Dofs>::pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<_Dofs> *robot)
    {
        for (int i = 0; i < _Dofs; i++)
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
            param_debug.orientation[i] = robot->getOrientation().toRotationMatrix().eulerAngles(2,1,0)[i];
            param_debug.orientation_d[i] = this->orientation_d.toRotationMatrix().eulerAngles(2,1,0)[i];
        }
    }
};

namespace my_controller
{
    // 计算力矩控制器
    template <int _Dofs = 7>
    class ComputedTorqueMethod : public Controller<_Dofs>
    {
    public:
        Eigen::Matrix<double, _Dofs, _Dofs> Kv;
        Eigen::Matrix<double, _Dofs, _Dofs> Kp;

        Eigen::Matrix<double, _Dofs, _Dofs> Kv_d;
        Eigen::Matrix<double, _Dofs, _Dofs> Kp_d;

    public:
        // ComputedTorqueMethod();
        void recordData(my_robot::Robot<_Dofs> *robot);
        void dynamicSetParameter(panda_controller::panda_controller_paramConfig &config);
        bool setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d);
        void calDesire(my_robot::Robot<_Dofs> *robot);
        void controllerParamRenew();
    };

    // 计算力矩法
    // 轨迹与控制律计算 保存
    template <int _Dofs>
    void ComputedTorqueMethod<_Dofs>::dynamicSetParameter(panda_controller::panda_controller_paramConfig &config)
    {
        if (this->time == 0)
        {
            Kv = config.Kv * Eigen::MatrixXd::Identity(_Dofs, _Dofs);
            Kp = config.Kp * Eigen::MatrixXd::Identity(_Dofs, _Dofs);
        }
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
    bool ComputedTorqueMethod<_Dofs>::setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d_)
    {
        this->qc = this->ddq_d + Kp * this->jointError + Kv * this->djointError;
        this->tau_d << robot->getM() * (this->qc) + robot->getC() * robot->getdq() /* + G */;
        tau_d_ = this->tau_d;
        return true;
    }
    template <int _Dofs>
    void ComputedTorqueMethod<_Dofs>::calDesire(my_robot::Robot<_Dofs> *robot)
    {
        double TPP = 0.5;  // Trajectory position parameters
        double TVP = 10.0; // Trajectory velocity parameters
        double delta_angle = M_PI / 4 * std::sin(M_PI / TVP * this->time / 1000.0) * TPP;
        double dot_delta_angle = M_PI / 4 * M_PI / TVP * (std::cos(M_PI / TVP * this->time / 1000.0)) * TPP;
        double ddot_delta_angle = -M_PI / 4 * M_PI / TVP * M_PI / TVP * (std::sin(M_PI / TVP * this->time / 1000.0)) * TPP;

        for (size_t i = 0; i < 7; ++i)
        {
            this->q_d[i] = robot->getq0()[i] + delta_angle;
            this->dq_d[i] = dot_delta_angle;
            this->ddq_d[i] = ddot_delta_angle;
        }
    }
    template <int _Dofs>
    void ComputedTorqueMethod<_Dofs>::recordData(my_robot::Robot<_Dofs> *robot)
    {
        if (this->recordPeriod == 0)
            return;
        if (this->time == 1)
        {
            this->myfile.open("/home/wd/ComputedTorqueMethod.txt");
            this->myfile << "ComputedTorqueMethod" << std::endl;
            this->myfile << "------程序编译日期:" << __DATE__ << "------" << std::endl;
            this->myfile << "------程序编译时刻:" << __TIME__ << "------" << std::endl;
        }

        this->myfile << "time: " << this->time << "_" << std::endl;
        this->myfile << "q0: " << robot->getq0().transpose() << std::endl;
        this->myfile << "q: " << robot->getq().transpose() << std::endl;
        this->myfile << "dq: " << robot->getdq().transpose() << std::endl;
        this->myfile << "q_d: " << this->q_d.transpose() << std::endl;
        this->myfile << "dq_d: " << this->dq_d.transpose() << std::endl;
        this->myfile << "ddq_d: " << this->ddq_d.transpose() << std::endl;
        this->myfile << "Position: " << robot->getPosition().transpose() << std::endl;

        this->myfile << "Orientation: " << std::endl;
        this->myfile << robot->getOrientation().toRotationMatrix() << std::endl;

        this->myfile << "T: " << std::endl;
        this->myfile << robot->getT().matrix() << std::endl;
        this->myfile << "M: " << std::endl;
        this->myfile << robot->getM() << std::endl;
        this->myfile << "C: " << std::endl;
        this->myfile << robot->getC() * robot->getdq() << std::endl;
        this->myfile << "G: " << std::endl;
        this->myfile << robot->getG() << std::endl;

        this->myfile << "Kp: " << std::endl;
        this->myfile << Kv << std::endl;
        this->myfile << "Kv: " << std::endl;
        this->myfile << Kp << std::endl;

        this->myfile << "J: " << std::endl;
        this->myfile << robot->getJ() << std::endl;
        this->myfile << "qc: " << this->qc.transpose() << std::endl;

        this->myfile << "getTorque: " << robot->getTorque().transpose() << std::endl;
        this->myfile << "tau_d: " << this->tau_d.transpose() << std::endl;
        this->myfile << "-------------------" << std::endl;
    }
};
typedef my_controller::Controller<robotDim> Robot7Controller;
typedef my_controller::ComputedTorqueMethod<robotDim> Robot7Controller1;
Robot7Controller *pController = nullptr;

template <int _Dofs>
void robotRun(Eigen::Matrix<double, _Dofs, 1> q, Eigen::Matrix<double, _Dofs, 1> dq, Eigen::Matrix<double, _Dofs, 1> tau, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Affine3d TO2E, Eigen::Matrix<double, _Dofs, 1> &tau_d)
{
    pController->updateTime();

    pPanda->updateJointData(q, dq, tau);

    pPanda->updateEndeffectorData(position, orientation, TO2E);

    pPanda->calculation(pController->ddq_d);

    pController->calDesire(pPanda);

    pController->calError(pPanda);
    pController->setControllerLaw(pPanda, tau_d);

    pController->recordData(pPanda);
}

void robotInit()
{
    if (pinInteractive == nullptr)
    {
        pinInteractive = new pinLibInteractive();
    }
    if (pController == nullptr)
    {
        pController = new Robot7Controller1();
    }
    if (pPanda == nullptr)
    {
        pPanda = new Robot7();
    }
}

template <int _Dofs>
void robotStart(Eigen::Matrix<double, _Dofs, 1> q0, int recordPeriod)
{
    pController->setRecord(recordPeriod);
    pPanda->setq0(q0);
}