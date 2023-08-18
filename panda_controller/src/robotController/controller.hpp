#pragma once

#include "robot/robot.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include "communication/communication.h"
#include <vector>
#include <queue>

#include <cmath>

enum TaskSpace
{
    jointSpace,
    cartesianSpace
};
enum ControllerStatus
{
    run,
    stop,
    wait,
};
namespace robot_controller
{
    // 控制律基类
    template <int _Dofs, typename dynParamType>
    class ControllerLaw
    {
    public:
        TaskSpace taskSpace;
        ControllerStatus controllerStatus;
        std::string controllerLawName;

        // 当前时刻误差
        Eigen::Matrix<double, _Dofs, 1> jointError;
        Eigen::Matrix<double, _Dofs, 1> djointError;
        Eigen::Matrix<double, 6, 1> cartesianError;
        Eigen::Matrix<double, 6, 1> dcartesianError;

        // 当前时刻期望 关节空间
        Eigen::Matrix<double, _Dofs, 1> q_d;
        Eigen::Matrix<double, _Dofs, 1> dq_d;
        Eigen::Matrix<double, _Dofs, 1> ddq_d;

        // 当前时刻期望 笛卡尔空间
        Eigen::Vector3d position_d;
        Eigen::Quaterniond orientation_d;
        Eigen::Vector3d dposition_d;
        Eigen::Quaterniond dorientation_d;
        Eigen::Matrix<double, 6, 1> ddX_d; // position+orientation

        // controllerLaw
        Eigen::Matrix<double, _Dofs, 1> tau_d;
        Eigen::Matrix<double, _Dofs, 1> qc;

    public:
        virtual void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d) = 0;
        virtual void dynamicSetParameter(dynParamType &config, unsigned int time) = 0;
        virtual void controllerParamRenew(double filterParams) = 0;
    };

    /***/
    /***/
    /***/

    // _Dofs自由度机器人控制器
    template <int _Dofs, typename pubDataType, typename dynParamType>
    class Controller
    {

    public:
        // debug，绘图
        int recordPeriod = 1;           // 记录周期
        unsigned int time = 0;          // 时刻
        std::ofstream myfile;           // 记录文件
        double filterParams = 0.005;    // 调参滤波参数
        const double cycleTime = 0.001; // 运行周期0.001s

        std::ofstream tmpfile; // 记录文件

        // 最终期望位置
        Eigen::Matrix<double, _Dofs, 1> q_calQueue;
        double Tf = 0; // s

        // 当前状态

        // 运行队列
        std::vector<std::queue<double>> q_dQueue{_Dofs};
        std::vector<std::queue<double>> dq_dQueue{_Dofs};
        std::vector<std::queue<double>> ddq_dQueue{_Dofs};

        // 打包通讯
        struct RobotData robotData;

        // 控制律
        ControllerLaw<_Dofs, dynParamType> *controllerLaw;

    public:
        void init(int recordPeriod);
        void setRecord(int recordPeriod);
        std::string getControllerLawName();

        void updateTime();
        void controllerParamRenew();
        void updateStatus();
        void calDesireQueue(my_robot::Robot<_Dofs> *robot); // 计算队列
        void calDesireNext(my_robot::Robot<_Dofs> *robot);  // 计算下一个周期的期望
        void calError(my_robot::Robot<_Dofs> *robot);
        void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d); // 封装

        void recordData(my_robot::Robot<_Dofs> *robot);
        void pubData(pubDataType &param_debug, my_robot::Robot<_Dofs> *robot);

        void dynamicSetParameter(dynParamType &config); // 封装
    };

    //
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::setRecord(int recordPeriod)
    {
        this->recordPeriod = recordPeriod;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    std::string Controller<_Dofs, pubDataType, dynParamType>::getControllerLawName()
    {
        if (controllerLaw != nullptr)
            return this->controllerLaw.controllerLawName;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::init(int recordPeriod)
    {
        setRecord(recordPeriod);
        Tf = 10;
        for (int i = 0; i < _Dofs; i++)
        {
            q_calQueue[i] = 0;
        }
    }

    //
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::updateTime()
    {
        this->time++;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::controllerParamRenew()
    {
        if (controllerLaw != nullptr)
            controllerLaw->controllerParamRenew(this->filterParams);
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::updateStatus()
    {
    }

    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::calDesireQueue(my_robot::Robot<_Dofs> *robot)
    {
        if (this->Tf == 0)
        {
            return;
        }
        tmpfile.open("/home/wd/log/franka/calDesireQueue.txt");

        static Eigen::Matrix<double, _Dofs, 1> theta0, thetaf, dtheta0, dthetaf, ddtheta0, ddthetaf;
        theta0 = robot->getq();
        tmpfile << "theta0: " << theta0 << "\n";
        thetaf = q_calQueue;
        dtheta0 = Eigen::VectorXd::Zero(_Dofs); /* robot->getdq() */
        dthetaf = Eigen::VectorXd::Zero(_Dofs);
        ddtheta0 = Eigen::VectorXd::Zero(_Dofs); /* robot->getddq() */
        ddthetaf = Eigen::VectorXd::Zero(_Dofs);
        int pointNum = static_cast<int>(this->Tf / this->cycleTime); //
        for (int i = 0; i < _Dofs; i++)
        {
            double a0 = theta0[i];
            double a1 = dtheta0[i] /* robot->getdq()[i] */;
            double a2 = ddtheta0[i] / 2 /* 0.5 * robot->getddq()[i] */;
            double a3 = (20 * thetaf[i] - 20 * theta0[i] - (8 * dthetaf[i] + 12 * dtheta0[i]) * this->Tf - (3 * ddtheta0[i] - ddthetaf[i]) * pow(this->Tf, 2)) / (2 * pow(this->Tf, 3));
            double a4 = (30 * theta0[i] - 30 * thetaf[i] + (14 * dthetaf[i] + 16 * dtheta0[i]) * this->Tf + (3 * ddtheta0[i] - 2 * ddthetaf[i]) * pow(this->Tf, 2)) / (2 * pow(this->Tf, 4));
            double a5 = (12 * thetaf[i] - 12 * theta0[i] - (6 * dthetaf[i] + 6 * dtheta0[i]) * this->Tf - (ddtheta0[i] - ddthetaf[i]) * pow(this->Tf, 2)) / (2 * pow(this->Tf, 5));
            tmpfile << "i: " << i << "\n";
            for (int j = 1; j <= pointNum; j++)
            {
                double t = j * cycleTime;
                double detla = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);
                q_dQueue[i].push(detla);
                detla = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);
                dq_dQueue[i].push(detla);
                detla = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
                ddq_dQueue[i].push(detla);
                tmpfile << "detla: " << detla << "\n";
            }
        }
        this->Tf = 0;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::calDesireNext(my_robot::Robot<_Dofs> *robot)
    {
        for (int i = 0; i < _Dofs; i++)
        {
            if (q_dQueue[i].empty())
            {
                this->controllerLaw->q_d[i] = robot->getq()[i];
                this->controllerLaw->dq_d[i] = 0;
                this->controllerLaw->ddq_d[i] = 0;
            }
            else
            {
                this->controllerLaw->q_d[i] = q_dQueue[i].front();
                this->controllerLaw->dq_d[i] = dq_dQueue[i].front();
                this->controllerLaw->ddq_d[i] = ddq_dQueue[i].front();
                q_dQueue[i].pop();
                dq_dQueue[i].pop();
                ddq_dQueue[i].pop();
            }
        }
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::calError(my_robot::Robot<_Dofs> *robot)
    {
        if (controllerLaw == nullptr)
            return;
        // 关节误差与误差导数
        this->controllerLaw->jointError = this->controllerLaw->q_d - robot->getq();
        this->controllerLaw->djointError = this->controllerLaw->dq_d - robot->getdq();

        // 笛卡尔位姿误差
        Eigen::Quaterniond orientation = robot->getOrientation();
        this->controllerLaw->cartesianError.head(3) = this->controllerLaw->position_d - robot->getPosition();
        if (this->controllerLaw->orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
        {
            orientation.coeffs() << -orientation.coeffs();
        }
        Eigen::Quaterniond error_quaternion(orientation.inverse() * this->controllerLaw->orientation_d);
        this->controllerLaw->cartesianError.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        this->controllerLaw->cartesianError.tail(3) << robot->getT().rotation() * this->controllerLaw->cartesianError.tail(3);

        // 笛卡尔位姿误差导数
        Eigen::Quaterniond dorientation = robot->getdOrientation();
        this->controllerLaw->dcartesianError.head(3) = this->controllerLaw->dposition_d - robot->getdPosition();
        if (this->controllerLaw->dorientation_d.coeffs().dot(dorientation.coeffs()) < 0.0)
        {
            dorientation.coeffs() << -dorientation.coeffs();
        }
        Eigen::Quaterniond derror_quaternion(dorientation.inverse() * this->controllerLaw->dorientation_d);
        this->controllerLaw->cartesianError.tail(3) << derror_quaternion.x(), derror_quaternion.y(), derror_quaternion.z();
        this->controllerLaw->cartesianError.tail(3) << robot->getT().rotation() * this->controllerLaw->cartesianError.tail(3);
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d)
    {
        if (controllerLaw != nullptr)
            controllerLaw->setControllerLaw(robot, tau_d);
    }

    //
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::recordData(my_robot::Robot<_Dofs> *robot)
    {
        if (controllerLaw == nullptr)
            return;
        if (recordPeriod == 0)
            return;
        if (time == 1)
        {
            this->myfile.open("/home/wd/log/franka/pandaController.txt"); // todo
            this->myfile << "pandaController"
                         << "\n";
            this->myfile << this->controllerLaw->controllerLawName << "\n";
            this->myfile << "程序编译日期:" << __DATE__ << "\n";
            this->myfile << "程序编译时刻:" << __TIME__ << std::endl;
        }
        static const char *n = "\n";
        this->myfile << "time: " << this->time << "_" << n;
        // this->myfile << "q0: " << robot->getq0().transpose() << "\n";
        // this->myfile << "q: " << robot->getq().transpose() << "\n";
        // this->myfile << "dq: " << robot->getdq().transpose() << "\n";
        // this->myfile << "q_d: " << this->controllerLaw->q_d.transpose() << "\n";
        // this->myfile << "dq_d: " << this->controllerLaw->dq_d.transpose() << "\n";
        // this->myfile << "ddq_d: " << this->controllerLaw->ddq_d.transpose() << "\n";

        // this->myfile << "Position0: " << robot->getPosition0().transpose() << "\n";
        // this->myfile << "Orientation0: " << robot->getOrientation0().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        // this->myfile << "Position: " << robot->getPosition().transpose() << "\n";
        // this->myfile << "Orientation: " << robot->getOrientation().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        // this->myfile << "Position: " << robot->getdPosition().transpose() << "\n";
        // this->myfile << "Orientation: " << robot->getdOrientation().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        // this->myfile << "T:" << n;
        // this->myfile << robot->getT().matrix() << "\n";

        // this->myfile << "M:" << n;
        // this->myfile << robot->getM() << "\n";
        // this->myfile << "C: " << n;
        // this->myfile << robot->getC() * robot->getdq() << "\n";
        // this->myfile << "G: " << n;
        // this->myfile << robot->getG() << n;
        // this->myfile << "J: " << n;
        // this->myfile << robot->getJ() << "\n";

        // this->myfile << "getTorque: " << robot->getTorque().transpose() << "\n";
        // this->myfile << "tau_d: " << this->controllerLaw->tau_d.transpose() << "\n";
        this->myfile << "-------------------" << std::endl;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::pubData(pubDataType &param_debug, my_robot::Robot<_Dofs> *robot)
    {
        if (controllerLaw == nullptr)
            return;
        for (int i = 0; i < _Dofs; i++)
        {
            param_debug.jointError[i] = this->controllerLaw->jointError[i];
            param_debug.q[i] = robot->getq()[i];
            param_debug.q_d[i] = this->controllerLaw->q_d[i];
            param_debug.dq[i] = robot->getdq()[i];
            param_debug.dq_d[i] = this->controllerLaw->dq_d[i];
            param_debug.tau_d[i] = this->controllerLaw->tau_d[i];
        }
        for (int i = 0; i < 6; i++)
        {
            param_debug.cartesianError[i] = this->controllerLaw->cartesianError[i];
        }
        for (int i = 0; i < 3; i++)
        {
            param_debug.position[i] = robot->getPosition()[i];
            param_debug.position_d[i] = this->controllerLaw->position_d[i];
            param_debug.orientation[i] = robot->getOrientation().toRotationMatrix().eulerAngles(2, 1, 0)[i];
            param_debug.orientation_d[i] = this->controllerLaw->orientation_d.toRotationMatrix().eulerAngles(2, 1, 0)[i];
        }
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::dynamicSetParameter(dynParamType &config)
    {
        if (controllerLaw != nullptr)
            controllerLaw->dynamicSetParameter(config, this->time);
    }

};
