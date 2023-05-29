#include <algorithm/pinocchino_interactive.h>
#include "controller/controller.h"

extern pinLibInteractive *pinInteractive;
PandaController *pController;
Panda *pPanda;

namespace my_controller
{
    // void Controller::startControllerManager()
    // {
    //     if (pinInteractive == nullptr)
    //         pinInteractive = new pinLibInteractive();
    // }

    template <int _Dofs>
    bool ComputedTorqueMethod<_Dofs>::setControllerLaw(my_robot::Robot<_Dofs> *robot)
    {
        this->qc = /* ddq_d */ +Kp * this->jointError + Kv * this->jointError;
        this->tau_d << robot->getM() * (this->qc) + robot->getC() /* + G */;
        return true;
    }
};

template <int _Dofs>
void robotRun(Eigen::Matrix<double, _Dofs, 1> q, Eigen::Matrix<double, _Dofs, 1> dq, Eigen::Matrix<double, _Dofs, 1> tau, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Matrix<double, 4, 4> TO2E)
{
    pPanda->updateJointData(q, dq, tau);
    pPanda->updateEndeffectorData(position, orientation, TO2E);

    pController->updateError(pPanda);
    pController->setControllerLaw(pPanda);
}
