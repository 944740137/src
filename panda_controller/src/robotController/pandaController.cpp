#include "pandaController.h"
#include <sys/msg.h>
Robot7 *pPanda = nullptr;
PandaController *pController = nullptr;

void pandaInit(std::string &urdfPath, std::string &TcpName)
{
    if (pPandaDynLibManager == nullptr)
    {
        pPandaDynLibManager = new PandaDynLibManager(urdfPath, TcpName);
    }
    if (pController == nullptr)
    {
        pController = new PandaController();
    }
    if (pPanda == nullptr)
    {
        pPanda = new my_robot::Robot<DIM>();
    }
}

void pandaStart(const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &theta,
                const Eigen::Matrix<double, DIM, 1> &dq, const Eigen::Matrix<double, DIM, 1> &tau,
                const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E, int recordPeriod)
{
    pPanda->setq0(q);
    pPanda->setPosAndOri0(position, orientation);

    pPanda->updateJointData(q, theta, dq, tau);
    pPanda->updateEndeffectorData(position, orientation, TO2E);

    pController->init(recordPeriod, pPanda);
}

void pandaRun(const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq,
              const Eigen::Matrix<double, DIM, 1> &theta, const Eigen::Matrix<double, DIM, 1> &tau,
              const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E,
              Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug, Eigen::Matrix<double, DIM, 1> &q_d)
{
    // 机器人更新传感器数据
    pPanda->updateJointData(q, theta, dq, tau);
    pPanda->updateEndeffectorData(position, orientation, TO2E);
    pPanda->calculation(pController->controllerLaw->ddq_d); // pinocchio

    // 控制器更新时间
    pController->updateTime();

    //  通信，读取缓存
    pController->communication(pPanda);
    //  根据缓存更新状态
    pController->updateStatus(pPanda);

    // 根据队列计算当前期望和误差计算输出力矩
    pController->calDesireNext(pPanda); // 取出队头作为当前期望
    pController->calError(pPanda);      // 误差
    pController->setControllerLaw(pPanda, tau_d);

    // 调节控制律参数 目标参数由回调函数设置
    pController->controllerParamRenew();

    // 本层控制器保存运行数据
    pController->recordData(pPanda);
    // 本层控制器发布运行数据(ROS)
    pController->pubData(param_debug, pPanda);
}

void pandaGetDyn(const Eigen::Matrix<double, 7, 7> &M, const Eigen::Matrix<double, 7, 1> &c, const Eigen::Matrix<double, 7, 1> &G, const Eigen::Matrix<double, 6, 7> &J)
{
    pPanda->setExternM(M);
    pPanda->setExternc(c);
    pPanda->setExternG(G);
    pPanda->setExternJ(J);
}