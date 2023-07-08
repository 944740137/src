#include "pandaController.h"

Robot7 *pPanda = nullptr;
Robot7Controller *pController = nullptr;

void pandaRun(const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq, const Eigen::Matrix<double, DIM, 1> &theta, const Eigen::Matrix<double, DIM, 1> &tau, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E, Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug)
{
    pController->updateTime();

    pPanda->updateJointData(q, theta, dq, tau);

    pPanda->updateEndeffectorData(position, orientation, TO2E);

    pPanda->calculation(pController->ddq_d);

    pController->calDesire(pPanda);

    pController->calError(pPanda);
    pController->setControllerLaw(pPanda, tau_d);

    pController->recordData(pPanda);

    pController->controllerParamRenew();

    pController->pubData(param_debug, pPanda);
}

void pandaInit()
{
    if (pPandaDynLibManager == nullptr)
    {
        pPandaDynLibManager = new pandaDynLibManager();
    }
    if (pController == nullptr)
    {
        // pController = new panda_controller::ComputedTorqueMethod(TaskSpace::jointSpace);
        pController = new panda_controller::Backstepping(TaskSpace::jointSpace);
        // pController = new panda_controller::PD(TaskSpace::jointSpace);
    }
    if (pPanda == nullptr)
    {
        pPanda = new my_robot::Robot<DIM>();
    }
}

void pandaStart(const Eigen::Matrix<double, DIM, 1> &q0, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, int recordPeriod)
{
    pController->setRecord(recordPeriod);
    pPanda->setq0(q0);
    pPanda->setPosAndOri0(position, orientation);
}

void pandaGetDyn(const Eigen::Matrix<double, 7, 7> &M, const Eigen::Matrix<double, 7, 1> &c, const Eigen::Matrix<double, 7, 1> &G, const Eigen::Matrix<double, 6, 7> &J)
{
    pPanda->setExternM(M);
    pPanda->setExternc(c);
    pPanda->setExternG(G);
    pPanda->setExternJ(J);
}