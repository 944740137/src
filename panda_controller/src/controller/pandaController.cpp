#include "pandaController.h"

Robot7 *pPanda = nullptr;
Robot7Controller *pController = nullptr;

void pandaRun(Eigen::Matrix<double, DIM, 1> q, Eigen::Matrix<double, DIM, 1> dq, Eigen::Matrix<double, DIM, 1> tau, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Affine3d TO2E, Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug)
{
    pController->updateTime();

    pPanda->updateJointData(q, dq, tau);

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
        pController = new panda_controller::ComputedTorqueMethod();
    }
    if (pPanda == nullptr)
    {
        pPanda = new my_robot::Robot<DIM>();
    }
}

void pandaStart(Eigen::Matrix<double, DIM, 1> q0, int recordPeriod)
{
    pController->setRecord(recordPeriod);
    pPanda->setq0(q0);
}

void pandaGetDyn(Eigen::Matrix<double, 7, 7> M, Eigen::Matrix<double, 7, 1> c, Eigen::Matrix<double, 7, 1> G, Eigen::Matrix<double, 6, 7> J)
{
    pPanda->setExternM(M);
    pPanda->setExternc(c);
    pPanda->setExternG(G);
    pPanda->setExternJ(J);
}