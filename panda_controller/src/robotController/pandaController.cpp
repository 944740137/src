#include "pandaController.h"
#include <sys/msg.h>
Robot7 *pPanda = nullptr;
Robot7Controller *pController = nullptr;
// static int msgid = -1;

void pandaInit()
{
    if (pPandaDynLibManager == nullptr)
    {
        pPandaDynLibManager = new pandaDynLibManager();
    }
    if (pController == nullptr)
    {
        pController = new Robot7Controller();
        pController->controllerLaw = new panda_controller::ComputedTorqueMethod(TaskSpace::jointSpace);
        // pController->controllerLaw = new panda_controller::PD(TaskSpace::jointSpace);
    }
    if (pPanda == nullptr)
    {
        pPanda = new my_robot::Robot<DIM>();
    }
}

void pandaStart(const Eigen::Matrix<double, DIM, 1> &q0, const Eigen::Vector3d &position,
                const Eigen::Quaterniond &orientation, int recordPeriod)
{
    pController->init(recordPeriod);
    pPanda->setq0(q0);
    pPanda->setPosAndOri0(position, orientation);
}

void pandaRecvDataFromController()
{
    // int msg = msgsnd(msgid, &(pController->robotData), sizeof(pController->robotData), IPC_NOWAIT);
    // if (msg == -1)
    // {
    //     printf("消息队列发送失败 error %s\n", strerror(errno));
    // }
}

void pandaRun(const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq,
              const Eigen::Matrix<double, DIM, 1> &theta, const Eigen::Matrix<double, DIM, 1> &tau,
              const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E,
              Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug, Eigen::Matrix<double, DIM, 1> &q_d)
{
    // 控制器更新时间
    pController->updateTime();

    // 机器人更新数据
    pPanda->updateJointData(q, theta, dq, tau);
    pPanda->updateEndeffectorData(position, orientation, TO2E);
    pPanda->calculation(pController->controllerLaw->ddq_d); // pinocchio

    // // 本层控制器更新上层控制器命令

    // // 本层控制器计算下发队列，更新控制下发力矩

    // 根据队列计算当前期望和误差
    pController->calDesireQueue(pPanda); // 队列
    pController->calDesireNext(pPanda);  // 当前期望
    pController->calError(pPanda);       // 误差
    pController->setControllerLaw(pPanda, tau_d);

    // // 本层控制器调节控制律参数
    pController->controllerParamRenew();

    // // 本层控制器保存运行数据
    pController->recordData(pPanda);
    // // 本层控制器发布运行数据
    pController->pubData(param_debug, pPanda);

    // // 控制器发送至上层进程（共享内存）
    // // pController->updateData2controller(pPanda);
    // // pandaRecvDataFromController();
}

void pandaGetDyn(const Eigen::Matrix<double, 7, 7> &M, const Eigen::Matrix<double, 7, 1> &c, const Eigen::Matrix<double, 7, 1> &G, const Eigen::Matrix<double, 6, 7> &J)
{
    pPanda->setExternM(M);
    pPanda->setExternc(c);
    pPanda->setExternG(G);
    pPanda->setExternJ(J);
}