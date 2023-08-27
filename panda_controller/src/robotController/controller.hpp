#pragma once

#include "planner/planner.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include "communication/communication.h"
#include <vector>

namespace robot_controller
{
    // 控制律基类
    template <int _Dofs, typename dynParamType>
    class ControllerLaw
    {
    public:
        TaskSpace taskSpace;
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
        ControllerLaw(const ControllerLaw &) = delete;
        void operator=(const ControllerLaw &) = delete;

        ControllerLaw();
        virtual ~ControllerLaw();

        virtual void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d) = 0;
        virtual void dynamicSetParameter(dynParamType &config, unsigned int time) = 0;
        virtual void controllerParamRenew(double filterParams) = 0;
    };

    /***/
    /***/
    /***/
    template <int _Dofs, typename dynParamType>
    ControllerLaw<_Dofs, dynParamType>::~ControllerLaw()
    {
    }
    template <int _Dofs, typename dynParamType>
    ControllerLaw<_Dofs, dynParamType>::ControllerLaw() : jointError(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                          djointError(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                          cartesianError(Eigen::Matrix<double, 6, 1>::Zero()),
                                                          dcartesianError(Eigen::Matrix<double, 6, 1>::Zero()),
                                                          q_d(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                          dq_d(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                          ddq_d(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                          position_d(Eigen::Matrix<double, 3, 1>::Zero()),
                                                          orientation_d(Eigen::Quaterniond::Identity()),
                                                          dposition_d(Eigen::Matrix<double, 3, 1>::Zero()),
                                                          dorientation_d(Eigen::Quaterniond::Identity()),
                                                          ddX_d(Eigen::Matrix<double, 6, 1>::Zero()),
                                                          tau_d(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                          qc(Eigen::Matrix<double, _Dofs, 1>::Zero())

    {
        // 全部初始化为0
    }

    // _Dofs自由度机器人控制器
    template <int _Dofs, typename pubDataType, typename dynParamType>
    class Controller
    {

    public:
        // debug，绘图
        int recordPeriod = 1;           // 数据记录周期
        unsigned int time = 0;          // 当前时刻
        std::ofstream myfile;           // 记录文件io对象
        double filterParams = 0.005;    // 调参滤波参数
        const double cycleTime = 0.001; // 运行周期0.001s

        std::ofstream qfile;   // 记录文件io对象
        std::ofstream dqfile;  // 记录文件io对象
        std::ofstream ddqfile; // 记录文件io对象

        // 点动参数
        unsigned int jogSign = 0;
        double jogSpeed = 0;
        double jogSpeed_d = 0;

        // 规划参数
        bool newPlan = false;
        TaskSpace plannerTaskSpace = TaskSpace::jointSpace;
        Eigen::Matrix<double, _Dofs, 1> q_calQueue;
        Eigen::Matrix<double, _Dofs, 1> q_hold;
        double runSpeed = 0;
        double runSpeed_d = 0;
        double Tf = 0;

        // 急停参数
        double stopDistance = 0.1;
        double stopTime = 0.1;

        // 当前运行状态和工作空间
        unsigned int nowCommandNum = 0;       // 当前命令号
        ControllerStatus controllerStatus_d;  // 目标状态
        ControllerStatus nowControllerStatus; // 当前状态

        // 运行队列
        std::vector<std::queue<double>> q_dQueue{_Dofs};
        std::vector<std::queue<double>> dq_dQueue{_Dofs};
        std::vector<std::queue<double>> ddq_dQueue{_Dofs};

        // 急停队列
        std::vector<std::queue<double>> q_stopQueue{_Dofs};
        std::vector<std::queue<double>> dq_stopQueue{_Dofs};
        std::vector<std::queue<double>> ddq_stopQueue{_Dofs};

        // 通讯
        Communication communicationModel;
        bool connectStatus = false;
        struct RobotData *robotDataBuff = nullptr;
        struct ControllerCommand *controllerCommandBUff = nullptr;

        //  控制律
        std::unique_ptr<ControllerLaw<_Dofs, dynParamType>> controllerLaw;

    public:
        Controller(const Controller &) = delete;
        void operator=(const Controller &) = delete;

        Controller();
        virtual ~Controller();

        // api
        void setRecord(int recordPeriod);
        std::string getControllerLawName();

        // init
        void init(int recordPeriod);

        // run
        void updateTime();
        void controllerParamRenew();
        void communication(my_robot::Robot<_Dofs> *robot);
        void updateStatus(my_robot::Robot<_Dofs> *robot);
        void calDesireNext(my_robot::Robot<_Dofs> *robot); // 计算下一个周期的期望
        void calError(my_robot::Robot<_Dofs> *robot);
        void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d);

        //
        void recordData(my_robot::Robot<_Dofs> *robot);
        void pubData(pubDataType &param_debug, my_robot::Robot<_Dofs> *robot);
        void dynamicSetParameter(dynParamType &config); // 封装
    };

    template <int _Dofs, typename pubDataType, typename dynParamType>
    Controller<_Dofs, pubDataType, dynParamType>::~Controller()
    {
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    Controller<_Dofs, pubDataType, dynParamType>::Controller() : q_calQueue(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                                 q_hold(Eigen::Matrix<double, _Dofs, 1>::Zero())
    {
    }

    // API
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

    // init
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::init(int recordPeriod)
    {
        // 设置数据记录周期
        setRecord(this->recordPeriod);

        // first task
        for (int i = 0; i < _Dofs; i++)
        {
            this->q_calQueue[i] = -0.1;
        }
        this->runSpeed = 1.0;
        this->newPlan = true;

        // 建立通信 建立数据映射
        if (this->communicationModel.createConnect((key_t)SM_ID, (key_t)MS_ID, this->robotDataBuff, this->controllerCommandBUff))
        {
            printf("通信模型建立成功\n");
        }

        this->controllerCommandBUff->controllerStatus = this->controllerStatus_d;
        this->controllerCommandBUff->jogSign = this->jogSign;
        this->controllerCommandBUff->jogSpeed = this->jogSpeed_d;
        this->controllerCommandBUff->runSpeed = this->runSpeed_d;
        this->controllerCommandBUff->commandNum = this->nowCommandNum;
    }

    // run
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::updateTime()
    {
        this->time++;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::controllerParamRenew()
    {
        if (this->controllerLaw != nullptr)
            this->controllerLaw->controllerParamRenew(this->filterParams);
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::communication(my_robot::Robot<_Dofs> *robot)
    {
        this->connectStatus = this->communicationModel.comSendMessage();

        if (!this->connectStatus)
            return;
        // send
        for (int i = 0; i < _Dofs; i++)
        {
            this->robotDataBuff->q_d[i] = this->controllerLaw->q_d[i];
            this->robotDataBuff->q[i] = robot->getq()[i];
            this->robotDataBuff->dq[i] = robot->getdq()[i];
            this->robotDataBuff->tau[i] = robot->getTorque()[i];
        }
        for (int i = 0; i < 3; i++)
        {
            this->robotDataBuff->position[i] = robot->getdOrientation().toRotationMatrix().eulerAngles(2, 1, 0)[i];
            this->robotDataBuff->orientation[i] = robot->getdPosition()[i];
        }

        // read 
        this->controllerStatus_d = this->controllerCommandBUff->controllerStatus;
        this->jogSign = this->controllerCommandBUff->jogSign;
        this->jogSpeed_d = this->controllerCommandBUff->jogSpeed;
        this->runSpeed_d = this->controllerCommandBUff->runSpeed;
        if (this->nowCommandNum != this->controllerCommandBUff->commandNum)
        {
            this->plannerTaskSpace = this->controllerCommandBUff->plannerTaskSpace;
            for (int i = 0; i < _Dofs; i++)
            {
                this->q_calQueue[i] = this->controllerCommandBUff->q_final[i];
            }
            for (int i = 0; i < 3; i++)
            {
            }
            this->newPlan = true;
            this->nowCommandNum = this->controllerCommandBUff->commandNum;
        }
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::updateStatus(my_robot::Robot<_Dofs> *robot)
    {
        if (!this->connectStatus)
            return;

        if (this->nowControllerStatus != this->controllerStatus_d)
        {
            if (this->controllerStatus_d == ControllerStatus::stop)
            {
                // calStopQueue(robot);
            }
            if (this->controllerStatus_d == ControllerStatus::run)
            {
                //
            }
            this->nowControllerStatus = this->controllerStatus_d;
        }
        if (this->jogSpeed != this->jogSpeed_d)
        {
            // cal
            this->jogSpeed = this->jogSpeed_d;
        }
        if (this->runSpeed != this->runSpeed_d)
        {
            // recal
            // this->runSpeed = this->runSpeed_d;
        }
        if (this->newPlan)
        {
            double velLimit[_Dofs] = {0.0};
            double accLimit[_Dofs] = {0.0};
            for (int i = 0; i < _Dofs; i++)
            {
                velLimit[i] = this->runSpeed * robot->getdqLimit()[i];
                accLimit[i] = /* this->runSpeed * */ robot->getdqLimit()[i];
            }
            calQuinticPlan(true, this->cycleTime, velLimit, accLimit, robot->getq(), this->q_calQueue, q_dQueue, dq_dQueue, ddq_dQueue);
            this->newPlan = false;
        }
    }

    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::calDesireNext(my_robot::Robot<_Dofs> *robot)
    {
        for (int i = 0; i < _Dofs; i++)
        {
            if (q_dQueue[i].empty())
            {
                this->controllerLaw->q_d[i] = this->q_hold[i];
                this->controllerLaw->dq_d[i] = 0;
                this->controllerLaw->ddq_d[i] = 0;
            }
            else
            {
                this->controllerLaw->q_d[i] = q_dQueue[i].front();
                this->controllerLaw->dq_d[i] = dq_dQueue[i].front();
                this->controllerLaw->ddq_d[i] = ddq_dQueue[i].front();
                this->q_dQueue[i].pop();
                this->dq_dQueue[i].pop();
                this->ddq_dQueue[i].pop();
                if (q_dQueue[i].empty())
                    this->q_hold[i] = this->controllerLaw->q_d[i];
            }
        }
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::calError(my_robot::Robot<_Dofs> *robot)
    {
        if (controllerLaw.get() == nullptr)
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
        if (controllerLaw.get() != nullptr)
            controllerLaw->setControllerLaw(robot, tau_d);
    }

    //
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::recordData(my_robot::Robot<_Dofs> *robot)
    {
        if (controllerLaw.get() == nullptr)
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
        if (time % recordPeriod != 0)
            return;

        static const char *n = "\n";
        // this->myfile << "time: " << this->time << "_" << n;
        // this->myfile << "q0: " << robot->getq0().transpose() << "\n";
        // this->myfile << "q: " << robot->getq().transpose() << "\n";
        // this->myfile << "dq: " << robot->getdq().transpose() << "\n";
        this->myfile << "q_d: " << this->controllerLaw->q_d.transpose() << "\n";
        this->myfile << "dq_d: " << this->controllerLaw->dq_d.transpose() << "\n";
        this->myfile << "ddq_d: " << this->controllerLaw->ddq_d.transpose() << "\n";

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
        // this->myfile << "-------------------" << std::endl;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::pubData(pubDataType &param_debug, my_robot::Robot<_Dofs> *robot)
    {
        if (controllerLaw.get() == nullptr)
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
        if (controllerLaw.get() != nullptr)
            controllerLaw->dynamicSetParameter(config, this->time);
    }

};
