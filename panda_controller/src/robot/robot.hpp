#pragma once

#include <algorithm/pandaDynLibManager.h>
#include <algorithm/calPseudoInverseMatrix.hpp>
// extern pinLibInteractive *pinInteractive;

namespace my_robot
{
    template <int _Dofs = 7>
    class Robot
    {

    private:
        // sensor
        Eigen::Matrix<double, _Dofs, 1> q0;
        Eigen::Matrix<double, _Dofs, 1> q;
        Eigen::Matrix<double, _Dofs, 1> dq;
        Eigen::Matrix<double, _Dofs, 1> theta;

        Eigen::Vector3d position0;
        Eigen::Quaterniond orientation0;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d dposition;
        Eigen::Quaterniond dorientation;

        Eigen::Affine3d TO2E;
        Eigen::Matrix<double, _Dofs, 1> tau;

        // pino
        Eigen::Matrix<double, _Dofs, _Dofs> C;
        Eigen::Matrix<double, _Dofs, _Dofs> M;
        Eigen::Matrix<double, _Dofs, 1> G;
        Eigen::Matrix<double, _Dofs * 10, _Dofs> Y;
        Eigen::Matrix<double, 6, _Dofs> J;
        Eigen::Matrix<double, _Dofs, 6> J_inv;

        // panda
        Eigen::Matrix<double, 7, 1> externc; // 科氏项,非矩阵
        Eigen::Matrix<double, 7, 7> externM;
        Eigen::Matrix<double, 7, 1> externG;
        Eigen::Matrix<double, 6, 7> externJ;

        // 加权伪逆
        Eigen::Matrix<double, 6, _Dofs> dJ;

    public:
        // get
        Eigen::Matrix<double, _Dofs, 1> getq0();
        Eigen::Matrix<double, _Dofs, 1> getq();
        Eigen::Matrix<double, _Dofs, 1> getdq();
        Eigen::Matrix<double, _Dofs, 1> getTheta();

        Eigen::Vector3d getPosition0();
        Eigen::Quaterniond getOrientation0();
        Eigen::Vector3d getPosition();
        Eigen::Quaterniond getOrientation();
        Eigen::Vector3d getdPosition();
        Eigen::Quaterniond getdOrientation();
        Eigen::Affine3d getT();
        Eigen::Matrix<double, _Dofs, 1> getTorque();

        // set
        bool setq0(const Eigen::Matrix<double, _Dofs, 1> &q);
        bool setPosAndOri0(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);
        bool updateJointData(const Eigen::Matrix<double, _Dofs, 1> &q, const Eigen::Matrix<double, _Dofs, 1> &theta, const Eigen::Matrix<double, _Dofs, 1> &dq, const Eigen::Matrix<double, _Dofs, 1> &tau);
        bool updateEndeffectorData(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E);

        // other dyn api
        bool setExternM(const Eigen::Matrix<double, _Dofs, _Dofs> &externM);
        bool setExternc(const Eigen::Matrix<double, _Dofs, 1> &externc);
        bool setExternG(const Eigen::Matrix<double, _Dofs, 1> &externG);
        bool setExternJ(const Eigen::Matrix<double, 6, _Dofs> &externJ);
        Eigen::Matrix<double, _Dofs, _Dofs> getExternM();
        Eigen::Matrix<double, _Dofs, 1> getExternc(); // 科氏项
        Eigen::Matrix<double, _Dofs, 1> getExternG();
        Eigen::Matrix<double, 6, _Dofs> getExternJ();

        // pinocchino dyn api
        bool calculation(Eigen::Matrix<double, _Dofs, 1> ddq_d);
        Eigen::Matrix<double, 6, _Dofs> getJ();
        Eigen::Matrix<double, _Dofs, 6> getJ_inv();
        Eigen::Matrix<double, _Dofs, _Dofs> getM();
        Eigen::Matrix<double, _Dofs, _Dofs> getC();
        Eigen::Matrix<double, _Dofs, 1> getG();
    };

}

namespace my_robot
{
    // sensor
    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getq0()
    {
        return this->q0;
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
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getTheta()
    {
        return this->theta;
    }

    template <int _Dofs>
    Eigen::Vector3d Robot<_Dofs>::getPosition0()
    {
        return this->position0;
    }
    template <int _Dofs>
    Eigen::Quaterniond Robot<_Dofs>::getOrientation0()
    {
        return this->orientation0;
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
    Eigen::Vector3d Robot<_Dofs>::getdPosition()
    {
        return this->dposition;
    }
    template <int _Dofs>
    Eigen::Quaterniond Robot<_Dofs>::getdOrientation()
    {
        return this->dorientation;
    }
    template <int _Dofs>
    Eigen::Affine3d Robot<_Dofs>::getT()
    {
        return this->TO2E;
    }
    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getTorque()
    {
        return this->tau;
    }

    template <int _Dofs>
    bool Robot<_Dofs>::setq0(const Eigen::Matrix<double, _Dofs, 1> &q)
    {
        this->q0 = q;
        return true;
    }
    template <int _Dofs>
    bool Robot<_Dofs>::setPosAndOri0(const Eigen::Vector3d &position0, const Eigen::Quaterniond &orientation0)
    {
        this->position0 = position0;
        this->orientation0 = orientation0;
        return true;
    }
    template <int _Dofs>
    bool Robot<_Dofs>::updateJointData(const Eigen::Matrix<double, _Dofs, 1> &q, const Eigen::Matrix<double, _Dofs, 1> &theta, const Eigen::Matrix<double, _Dofs, 1> &dq, const Eigen::Matrix<double, _Dofs, 1> &tau)
    {
        this->q = q;
        this->theta = theta;
        this->dq = dq;
        this->tau = tau;
        return true;
    }
    template <int _Dofs>
    bool Robot<_Dofs>::updateEndeffectorData(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E)
    {
        this->position = position;
        this->orientation = orientation;

        this->dposition = (this->externJ * dq).head(3);
        static Eigen::Matrix<double, 3, 1> tmp = (this->externJ * dq).tail(3); // 欧拉角
        this->dorientation = Eigen::AngleAxisd(tmp[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(tmp[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(tmp[2], Eigen::Vector3d::UnitZ());

        this->TO2E = TO2E;
        return true;
    }

    // pino
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
    Eigen::Matrix<double, 6, _Dofs> Robot<_Dofs>::getJ()
    {
        return this->J;
    }
    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 6> Robot<_Dofs>::getJ_inv()
    {
        return this->J_inv;
    }

    template <int _Dofs>
    bool Robot<_Dofs>::calculation(Eigen::Matrix<double, _Dofs, 1> ddq_d)
    {
        pinocchio::Data data = pPandaDynLibManager->getpData();
        pinocchio::Model model = pPandaDynLibManager->getpModel();

        pPandaDynLibManager->forwardKinematics(model, data, this->q);
        pPandaDynLibManager->updateFramePlacements(model, data);

        pPandaDynLibManager->computeJointJacobians(model, data, this->q);
        this->J = data.J;
        pPandaDynLibManager->computeJointJacobiansTimeVariation(model, data, q, dq);
        this->dJ = data.dJ;

        pPandaDynLibManager->rnea(model, data, this->q, this->dq, ddq_d);
        pPandaDynLibManager->computeGeneralizedGravity(model, data, this->q);
        this->G = data.g;
        pPandaDynLibManager->computeCoriolisMatrix(model, data, this->q, this->dq);
        this->C = data.C;
        pPandaDynLibManager->crba(model, data, this->q);
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        this->M = data.M;

        weightedPseudoInverse(this->J, this->J_inv, this->M);

        return true;
    }

    // other
    template <int _Dofs>
    bool Robot<_Dofs>::setExternM(const Eigen::Matrix<double, _Dofs, _Dofs> &externM)
    {
        this->externM = externM;
        return true;
    }
    template <int _Dofs>
    bool Robot<_Dofs>::setExternc(const Eigen::Matrix<double, _Dofs, 1> &externc)
    {
        this->externc = externc;
        return true;
    }
    template <int _Dofs>
    bool Robot<_Dofs>::setExternG(const Eigen::Matrix<double, _Dofs, 1> &externG)
    {
        this->externG = externG;
        return true;
    }
    template <int _Dofs>
    bool Robot<_Dofs>::setExternJ(const Eigen::Matrix<double, 6, _Dofs> &externJ)
    {
        this->externJ = externJ;
        return true;
    }
    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, _Dofs> Robot<_Dofs>::getExternM()
    {
        return this->externM;
    }
    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getExternc()
    {
        return this->externc;
    }
    template <int _Dofs>
    Eigen::Matrix<double, _Dofs, 1> Robot<_Dofs>::getExternG()
    {
        return this->externG;
    }
    template <int _Dofs>
    Eigen::Matrix<double, 6, _Dofs> Robot<_Dofs>::getExternJ()
    {
        return this->externJ;
    }
}
