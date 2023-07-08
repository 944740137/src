#pragma once

#include "robot/robot.hpp"
#include <fstream>
#include <iostream>
#include <string>

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
        std::string controllerLawName;

        // error
        Eigen::Matrix<double, _Dofs, 1> jointError;
        Eigen::Matrix<double, _Dofs, 1> djointError;
        Eigen::Matrix<double, 6, 1> cartesianError;
        Eigen::Matrix<double, 6, 1> dcartesianError;

        // controllerLaw
        Eigen::Matrix<double, _Dofs, 1> tau_d;
        Eigen::Matrix<double, _Dofs, 1> qc;

        // desire
        Eigen::Matrix<double, _Dofs, 1> q_d;
        Eigen::Matrix<double, _Dofs, 1> dq_d;
        Eigen::Matrix<double, _Dofs, 1> ddq_d;

        Eigen::Vector3d position_d;
        Eigen::Quaterniond orientation_d;

        Eigen::Vector3d dposition_d;
        Eigen::Quaterniond dorientation_d;

        Eigen::Matrix<double, 6, 1> ddX_d; // position+orientation

    public:
        // 无需重写
        void calError(my_robot::Robot<_Dofs> *robot);
        void updateTime();
        void setRecord(int record);
        std::string getControllerLawName();

        // 可重写
        virtual void recordData(my_robot::Robot<_Dofs> *robot);

        // 必须重写
        virtual void dynamicSetParameter(dynParamType &config) = 0;
        virtual void pubData(pubDataType &param_debug, my_robot::Robot<_Dofs> *robot) = 0;
        virtual void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d) = 0;
        virtual void calDesire(my_robot::Robot<_Dofs> *robot) = 0;
        virtual void controllerParamRenew() = 0;
    };
    // 无需重写的函数
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::calError(my_robot::Robot<_Dofs> *robot)
    {
        // 关节误差与误差导数
        this->jointError = q_d - robot->getq();
        this->djointError = dq_d - robot->getdq();

        // 笛卡尔位姿误差
        Eigen::Quaterniond orientation = robot->getOrientation();
        this->cartesianError.head(3) = position_d - robot->getPosition();
        if (this->orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
        {
            orientation.coeffs() << -orientation.coeffs();
        }
        Eigen::Quaterniond error_quaternion(orientation.inverse() * this->orientation_d);
        this->cartesianError.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        this->cartesianError.tail(3) << robot->getT().rotation() * this->cartesianError.tail(3);

        // 笛卡尔位姿误差导数
        Eigen::Quaterniond dorientation = robot->getdOrientation();
        this->dcartesianError.head(3) = dposition_d - robot->getdPosition();
        if (dorientation_d.coeffs().dot(dorientation.coeffs()) < 0.0)
        {
            dorientation.coeffs() << -dorientation.coeffs();
        }
        Eigen::Quaterniond derror_quaternion(dorientation.inverse() * dorientation_d);
        this->cartesianError.tail(3) << derror_quaternion.x(), derror_quaternion.y(), derror_quaternion.z();
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
    template <int _Dofs, typename pubDataType, typename dynParamType>
    std::string Controller<_Dofs, pubDataType, dynParamType>::getControllerLawName()
    {
        return this->controllerLawName;
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
            this->myfile << "pandaController" << "\n";
            this->myfile << this->controllerLawName << "\n";
            this->myfile << "--------程序编译日期:" << __DATE__ << "--------" << "\n";
            this->myfile << "--------程序编译时刻:" << __TIME__ << "--------" << std::endl;
        }

        this->myfile << "time: " << this->time << "_" << "\n";
        this->myfile << "q0: " << robot->getq0().transpose() << "\n";
        this->myfile << "q: " << robot->getq().transpose() << "\n";
        this->myfile << "dq: " << robot->getdq().transpose() << "\n";
        this->myfile << "q_d: " << this->q_d.transpose() << "\n";
        this->myfile << "dq_d: " << this->dq_d.transpose() << "\n";
        this->myfile << "ddq_d: " << this->ddq_d.transpose() << "\n";

        this->myfile << "Position0: " << robot->getPosition0().transpose() << "\n";
        this->myfile << "Orientation0: " << robot->getOrientation0().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        this->myfile << "Position: " << robot->getPosition().transpose() << "\n";
        this->myfile << "Orientation: " << robot->getOrientation().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        this->myfile << "Position: " << robot->getdPosition().transpose() << "\n";
        this->myfile << "Orientation: " << robot->getdOrientation().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        this->myfile << "T: " << "\n";
        this->myfile << robot->getT().matrix() << "\n";

        this->myfile << "M: " << "\n";
        this->myfile << robot->getM() << "\n";
        this->myfile << "C: " << "\n";
        this->myfile << robot->getC() * robot->getdq() << "\n";
        this->myfile << "G: " << "\n";
        this->myfile << robot->getG() << "\n";
        this->myfile << "J: " << "\n";
        this->myfile << robot->getJ() << "\n";

        this->myfile << "getTorque: " << robot->getTorque().transpose() << "\n";
        this->myfile << "tau_d: " << this->tau_d.transpose() << "\n";
        this->myfile << "-------------------" << std::endl;
    }

};
