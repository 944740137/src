#pragma once

#include "robot/robot.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include "communication/communication.h"
#include <vector>
#include <queue>

#include <cmath>
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
        int recordPeriod = 1;           // 记录周期
        unsigned int time = 0;          // 时刻
        std::ofstream myfile;           // 记录文件
        double filterParams = 0.005;    // 调参滤波参数
        const double cycleTime = 0.001; // 运行周期0.001s

        // 计算队列
        Eigen::Matrix<double, _Dofs, 1> q_calQueue;
        double Tf = 0; // s

        // 运行状态和工作空间
        // bool plannerOver;
        ControllerStatus ControllerStatus_d;  // 目标
        ControllerStatus nowControllerStatus; // 当前
        TaskSpace noWorkTaskSpace;            // 当前
        TaskSpace plannerTaskSpace;           // 计算队列时使用

        // 运行队列
        std::vector<std::queue<double>> q_dQueue{_Dofs};
        std::vector<std::queue<double>> dq_dQueue{_Dofs};
        std::vector<std::queue<double>> ddq_dQueue{_Dofs};

        // 通讯
        int msgid = -1;
        int shm_id = -1;
        bool connectStatus = false;
        struct Message *messageData = nullptr;
        struct SharedMemory *sharedMemoryData = nullptr;

        // 控制律
        std::unique_ptr<ControllerLaw<_Dofs, dynParamType>> controllerLaw;

    public:
        Controller(const Controller &) = delete;
        void operator=(const Controller &) = delete;

        Controller();
        virtual ~Controller();

        void init(int recordPeriod);
        void setRecord(int recordPeriod);

        void updateTime();
        void communication();
        bool checkConnect(SharedMemory *sharedMemory);
        void updateStatus(my_robot::Robot<_Dofs> *robot);

        void controllerParamRenew();
        void calDesireQueue(my_robot::Robot<_Dofs> *robot); // 计算队列
        void calDesireNext(my_robot::Robot<_Dofs> *robot);  // 计算下一个周期的期望
        void calError(my_robot::Robot<_Dofs> *robot);
        void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d); // 封装

        void recordData(my_robot::Robot<_Dofs> *robot);
        void pubData(pubDataType &param_debug, my_robot::Robot<_Dofs> *robot);

        void dynamicSetParameter(dynParamType &config); // 封装

        std::string getControllerLawName();
    };

    template <int _Dofs, typename pubDataType, typename dynParamType>
    Controller<_Dofs, pubDataType, dynParamType>::~Controller()
    {
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    Controller<_Dofs, pubDataType, dynParamType>::Controller() : q_calQueue(Eigen::Matrix<double, _Dofs, 1>::Zero())
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
        void *shared_memory = nullptr;
        messageData = new Message();

        setRecord(recordPeriod);
        Tf = 5;
        for (int i = 0; i < _Dofs; i++)
        {
            q_calQueue[i] = -0.1;
        }

        shm_id = shmget((key_t)SM_ID, sizeof(struct SharedMemory), 0666 | IPC_CREAT);
        if (shm_id < 0)
        {
            perror("第一次共享内存创建失败");
            exit(1);
        }
        else
            printf("共享内存创建成功\n");

        shared_memory = shmat(shm_id, NULL, 0);
        if (shared_memory == NULL)
        {
            perror("Failed to shmat");
            exit(1);
        }
        else
            printf("共享内存映射成功\n");
        sharedMemoryData = (struct SharedMemory *)shared_memory;
        sharedMemoryData->slaveHeartbeat = 0;

        msgid = msgget((key_t)MS_ID, 0666 | IPC_CREAT);
        if (msgid == -1)
            printf("消息队列创建失败\n");
        else
            printf("消息队列创建成功\n");
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
        if (controllerLaw != nullptr)
            controllerLaw->controllerParamRenew(this->filterParams);
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    bool Controller<_Dofs, pubDataType, dynParamType>::checkConnect(SharedMemory *sharedMemory)
    {
        static int timeout = 0;
        static int checkMasterConnect = sharedMemory->masterHeartbeat;
        static bool connect = false;

        timeout++;
        if (timeout >= 3)
        {
            if (checkMasterConnect == sharedMemory->masterHeartbeat)
                connect = false;
            else
                connect = true;
            checkMasterConnect = sharedMemory->masterHeartbeat;
            timeout = 0;
        }
        return connect;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::communication()
    {
        if (checkConnect(sharedMemoryData))
        {
            messageData->time++;

            /*读取*/
            if (!connectStatus)
            {
                printf("主站连接\n");
                connectStatus = true;
            }
            // printf("主站在线\n");
            if (msgsnd(msgid, (void *)messageData, sizeof(struct Message) - sizeof(long), IPC_NOWAIT) != 0)
            {
                // printf("发送失败\n");
            }
            // printf("messageData: %d\n", messageData->time);
        }
        else
        {
            if (connectStatus)
            {
                printf("主站断开\n");
                connectStatus = false;
            }
            // printf("主站离线\n");
        }

        sharedMemoryData->slaveHeartbeat++;
    }
    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::updateStatus(my_robot::Robot<_Dofs> *robot)
    {
        if (!connectStatus)
            return;
        // send
        for (int i = 0; i < _Dofs; i++)
        {
            messageData->robotData.q_d[i] = this->controllerLaw->q_d[i];
            messageData->robotData.q[i] = robot->getq()[i];
            messageData->robotData.dq[i] = robot->getdq()[i];
            messageData->robotData.tau[i] = robot->getTorque()[i];
        }
        for (int i = 0; i < 3; i++)
        {
            messageData->robotData.position[i] = robot->getdOrientation().toRotationMatrix().eulerAngles(2, 1, 0)[i];
            messageData->robotData.orientation[i] = robot->getdPosition()[i];
        }
        // controllerData.time = this->time;
        // controllerData.controllerLawName = this->controllerLaw->controllerLawName;
        // controllerData.plannerLawName = this->controllerLaw->controllerLawName;

        // // read
        // this->ControllerStatus_d = controllerData.ControllerStatus_d;

        // this->plannerTaskSpace = controllerData.plannerTaskSpace;
        // this->tf = controllerData.Tf;
        // for (int i = 0; i < _Dofs; i++)
        // {
        //     q_calQueue[i] = controllerData.q_final[i];
        // }
    }

    template <int _Dofs, typename pubDataType, typename dynParamType>
    void Controller<_Dofs, pubDataType, dynParamType>::calDesireQueue(my_robot::Robot<_Dofs> *robot)
    {
        if ((this->Tf == 0))
        {
            return;
        }

        static Eigen::Matrix<double, _Dofs, 1> theta0, thetaf, dtheta0, dthetaf, ddtheta0, ddthetaf;
        if (nowControllerStatus == ControllerStatus::run)
        {
        }
        else
        {
            // theta0 = robot->getq();
        }
        theta0 = robot->getq();
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
            for (int j = 1; j <= pointNum; j++)
            {
                double t = j * cycleTime;
                double detla = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);
                q_dQueue[i].push(detla);
                detla = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);
                dq_dQueue[i].push(detla);
                detla = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
                ddq_dQueue[i].push(detla);
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
        static const char *n = "\n";
        this->myfile << "time: " << this->time << "_" << n;
        // this->myfile << "q0: " << robot->getq0().transpose() << "\n";
        this->myfile << "q: " << robot->getq().transpose() << "\n";
        this->myfile << "dq: " << robot->getdq().transpose() << "\n";
        this->myfile << "q_d: " << this->controllerLaw->q_d.transpose() << "\n";
        this->myfile << "dq_d: " << this->controllerLaw->dq_d.transpose() << "\n";
        this->myfile << "ddq_d: " << this->controllerLaw->ddq_d.transpose() << "\n";

        this->myfile << "Position0: " << robot->getPosition0().transpose() << "\n";
        this->myfile << "Orientation0: " << robot->getOrientation0().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        this->myfile << "Position: " << robot->getPosition().transpose() << "\n";
        this->myfile << "Orientation: " << robot->getOrientation().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        this->myfile << "Position: " << robot->getdPosition().transpose() << "\n";
        this->myfile << "Orientation: " << robot->getdOrientation().toRotationMatrix().eulerAngles(2, 1, 0).transpose() << "\n";
        this->myfile << "T:" << n;
        this->myfile << robot->getT().matrix() << "\n";

        this->myfile << "M:" << n;
        this->myfile << robot->getM() << "\n";
        this->myfile << "C: " << n;
        this->myfile << robot->getC() * robot->getdq() << "\n";
        this->myfile << "G: " << n;
        this->myfile << robot->getG() << n;
        this->myfile << "J: " << n;
        this->myfile << robot->getJ() << "\n";

        this->myfile << "getTorque: " << robot->getTorque().transpose() << "\n";
        this->myfile << "tau_d: " << this->controllerLaw->tau_d.transpose() << "\n";
        this->myfile << "-------------------" << std::endl;
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
