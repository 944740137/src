#pragma once
#include <algorithm/pinocchino_interactive.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm/pseudo_inversion.h>
extern pinLibInteractive *pinInteractive;
namespace my_robot
{
    template <int _Dofs = 7>
    class Robot
    {

    private:
        Eigen::Matrix<double, _Dofs, 1> q0;
        Eigen::Matrix<double, _Dofs, 1> q;
        Eigen::Matrix<double, _Dofs, 1> dq;
        Eigen::Matrix<double, _Dofs, 1> tau;

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Affine3d TO2E;

        Eigen::Matrix<double, _Dofs, _Dofs> C;
        Eigen::Matrix<double, _Dofs, _Dofs> M;
        Eigen::Matrix<double, _Dofs, 1> G;
        Eigen::Matrix<double, _Dofs * 10, _Dofs> Y;

        Eigen::Matrix<double, 6, _Dofs> J;
        Eigen::Matrix<double, _Dofs, 6> J_inv;
        Eigen::Matrix<double, 6, _Dofs> dJ;

    public:
        Eigen::Matrix<double, 6, _Dofs> getJ();
        Eigen::Matrix<double, _Dofs, _Dofs> getM();
        Eigen::Matrix<double, _Dofs, _Dofs> getC();
        Eigen::Matrix<double, _Dofs, 1> getG();

        Eigen::Matrix<double, _Dofs, 1> getq0();
        Eigen::Matrix<double, _Dofs, 1> getq();
        Eigen::Matrix<double, _Dofs, 1> getdq();
        Eigen::Matrix<double, _Dofs, 1> getTorque();
        Eigen::Vector3d getPosition();
        Eigen::Quaterniond getOrientation();
        Eigen::Affine3d getT();
        bool setq0(Eigen::Matrix<double, _Dofs, 1> q);

        // franka api
        // bool setM(Eigen::Matrix<double, _Dofs, _Dofs> M);
        // bool setC(Eigen::Matrix<double, _Dofs, _Dofs> C);
        // bool setG(Eigen::Matrix<double, _Dofs, _Dofs> G);
        bool updateJointData(Eigen::Matrix<double, _Dofs, 1> q, Eigen::Matrix<double, _Dofs, 1> dq, Eigen::Matrix<double, _Dofs, 1> tau);
        bool updateEndeffectorData(Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Affine3d TO2E);

        // pinocchino api
        bool calculation(Eigen::Matrix<double, _Dofs, 1> ddq_d);
    };

}



namespace my_robot
{
    template <int _Dofs>
    Eigen::Matrix<double, 6, _Dofs> Robot<_Dofs>::getJ()
    {
        return this->J;
    }
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
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getq0()
    {
        return this->q0;
    }
    template <int _Dofs>
    bool Robot<_Dofs>::setq0(Eigen::Matrix<double, _Dofs, 1> q)
    {
        this->q0 = q;
        return true;
    }
    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getq()
    {
        return this->q;
    }

    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getdq()
    {
        return this->dq;
    }

    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getTorque()
    {
        return this->tau;
    }
    template <int _Dofs>
    Eigen::Vector3d Robot<_Dofs>::getPosition()
    {
        return this->position;
    }
    template <int _Dofs>
    Eigen::Quaterniond Robot<_Dofs>::getOrientation()
    {
        return this->orientation;
    }
    template <int _Dofs>
    Eigen::Affine3d Robot<_Dofs>::getT()
    {
        return this->TO2E;
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

        weightedPseudoInverse(this->J, this->J_inv, this->M);

        return true;
    }

    template <int _Dofs>
    bool Robot<_Dofs>::updateJointData(Eigen::Matrix<double, _Dofs, 1> q, Eigen::Matrix<double, _Dofs, 1> dq, Eigen::Matrix<double, _Dofs, 1> tau)
    {
        this->q = q;
        this->dq = dq;
        this->tau = tau;
        return true;
    }

    template <int _Dofs>
    bool Robot<_Dofs>::updateEndeffectorData(Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Affine3d TO2E)
    {
        this->position = position;
        this->orientation = orientation;
        this->TO2E = TO2E;
        return true;
    }
}
const int robotDim = 7;
typedef my_robot::Robot<robotDim> Robot7;
Robot7 *pPanda = nullptr;
