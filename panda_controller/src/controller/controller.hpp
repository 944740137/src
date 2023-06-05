#pragma once
// #include <algorithm/pinocchino_interactive.h>

#include <fstream>
#include <iostream>
#include "robot/robot.hpp"

namespace robot_controller
{
    template <int _Dofs, typename pubDataType, typename dynParamType>
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
        // 无需重写
        void calError(my_robot::Robot<_Dofs> *robot);
        void updateTime();
        void setRecord(int record);

        // 可重写
        virtual void recordData(my_robot::Robot<_Dofs> *robot);

        // 必须重写
        virtual void dynamicSetParameter(dynParamType &config) = 0;
        virtual void pubData(pubDataType &param_debug, my_robot::Robot<_Dofs> *robot) = 0;
        virtual bool setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d) = 0;
        virtual void calDesire(my_robot::Robot<_Dofs> *robot) = 0;
        virtual void controllerParamRenew() = 0;
    };
    // 无需重写的函数
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::calError(my_robot::Robot<_Dofs> *robot)
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
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::updateTime()
    {
        this->time++;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::setRecord(int record)
    {
        this->recordPeriod = record;
    }

    // 可重写的函数
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::recordData(my_robot::Robot<_Dofs> *robot)
    {
        if (recordPeriod == 0)
            return;
        if (time == 1)
        {
            this->myfile.open("/home/wd/pandaController.txt");
            this->myfile << "pandaController" << std::endl;
            this->myfile << "--------程序编译日期:" << __DATE__ << "--------" << std::endl;
            this->myfile << "--------程序编译时刻:" << __TIME__ << "--------" << std::endl;
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
        this->myfile << robot->getOrientation().toRotationMatrix().eulerAngles(2,1,0) << std::endl;
        this->myfile << "T: " << std::endl;
        this->myfile << robot->getT().matrix() << std::endl;

        this->myfile << "M: " << std::endl;
        this->myfile << robot->getM() << std::endl;
        this->myfile << "C: " << std::endl;
        this->myfile << robot->getC() * robot->getdq() << std::endl;
        this->myfile << "G: " << std::endl;
        this->myfile << robot->getG() << std::endl;
        this->myfile << "J: " << std::endl;
        this->myfile << robot->getJ() << std::endl;

        this->myfile << "getTorque: " << robot->getTorque().transpose() << std::endl;
        this->myfile << "tau_d: " << this->tau_d.transpose() << std::endl;
        this->myfile << "-------------------" << std::endl;
    }

};
