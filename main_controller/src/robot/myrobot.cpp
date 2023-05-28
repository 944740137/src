#include <algorithm/pinocchino_interactive.h>
#include "robot/myrobot.h"
// #include <controllerLaw/controllerLaw.h>

extern pinLibInteractive *pinInteractive;

namespace my_robot
{
    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, _Dofs> Robot<_Dofs>::getM()
    {
        return this->M;
    }

    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, _Dofs> Robot<_Dofs>::getC()
    {
        return this->C;
    }

    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getG()
    {
        return this->G;
    }

    template <int _Dofs>
    bool Robot<_Dofs>::calculation(Eigen::Matrix<double, _Dofs, 1> ddq_d)
    {
        pinocchio::Data data = pinInteractive->getpData();
        pinocchio::Model model = pinInteractive->getpModel();

        pinocchio::forwardKinematics(model, data, this->q);
        pinocchio::updateFramePlacements(model, data);

        pinocchio::computeJointJacobians(model, data, this->q);
        this->J = data.J;
        pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
        this->dJ = data.dJ;

        pinocchio::rnea(model, data, this->q, this->dq, ddq_d);
        pinocchio::computeGeneralizedGravity(model, data, this->q);
        this->G = data.g;
        pinocchio::computeCoriolisMatrix(model, data, this->q, this->dq);
        this->C = data.C;
        pinocchio::crba(model, data, this->q);
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        this->M = data.M;

        weightedPseudoInverse(J, J_inv, this->M);

        return true;
    }

    template <int _Dofs>
    bool Robot<_Dofs>::updateJointData(Eigen::Matrix<double, _Dofs, 1> q, Eigen::Matrix<double, _Dofs, 1> dq, Eigen::Matrix<double, _Dofs, 1> tau)
    {
        this->q = q;
        this->dq = dq;
        return true;
    }

    template <int _Dofs>
    bool Robot<_Dofs>::updateEndeffectorData(Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Matrix<double, 4, 4> TO2E)
    {
        this->position = position;
        this->orientation = orientation;
        this->TO2E = TO2E;
        return true;
    }
}