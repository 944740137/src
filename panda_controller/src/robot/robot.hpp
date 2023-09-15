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
        // limit
        const double dposMax[3] = {1.7};
        const double doriMax[3] = {2.5};

        // sensor
        Eigen::Matrix<double, _Dofs, 1> q0;
        Eigen::Vector3d position0;
        Eigen::Quaterniond orientation0;

        Eigen::Matrix<double, _Dofs, 1> theta;
        Eigen::Matrix<double, _Dofs, 1> q;
        Eigen::Matrix<double, _Dofs, 1> dq;

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation; // todo 换成欧拉角
        Eigen::Vector3d dposition;
        Eigen::Quaterniond dorientation;

        Eigen::Affine3d TO2E;
        Eigen::Matrix<double, _Dofs, 1> tau;

        // pino
        Eigen::Matrix<double, _Dofs, _Dofs> C;
        Eigen::Matrix<double, _Dofs, _Dofs> M;
        Eigen::Matrix<double, _Dofs, 1> G;
        // Eigen::Matrix<double, _Dofs * 10, _Dofs> Y;
        Eigen::Matrix<double, 6, _Dofs> J;
        Eigen::Matrix<double, 6, _Dofs> dJ;
        Eigen::Matrix<double, _Dofs, 6> J_inv;

        // panda
        Eigen::Matrix<double, 7, 1> externc; // 科氏项,非矩阵
        Eigen::Matrix<double, 7, 7> externM;
        Eigen::Matrix<double, 7, 1> externG;
        Eigen::Matrix<double, 6, 7> externJ;
        Eigen::Matrix<double, 6, _Dofs> externdJ;
        Eigen::Matrix<double, _Dofs, 6> externdJ_inv;

    public:
        double qMax[7] = {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};
        double qMin[7] = {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
        double dqLimit[7] = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
        double ddqLimit[7] = {15, 7.5, 10, 12.5, 15, 10, 10};

        Robot(const Robot &) = delete;
        void operator=(const Robot &) = delete;

        Robot();
        virtual ~Robot();

        // gei limit
        const double *const getdposMax() const;
        const double *const getdoriMax() const;
        const double *const getqMax() const;
        const double *const getqMin() const;
        const double *const getdqLimit() const;
        const double *const getddqLimit() const;

        // get kinematics
        const Eigen::Matrix<double, _Dofs, 1> &getq0();
        const Eigen::Matrix<double, _Dofs, 1> &getq();
        const Eigen::Matrix<double, _Dofs, 1> &getdq();

        const Eigen::Vector3d &getPosition0();
        const Eigen::Quaterniond &getOrientation0();
        const Eigen::Vector3d &getPosition();
        const Eigen::Quaterniond &getOrientation();
        const Eigen::Vector3d &getdPosition();
        const Eigen::Quaterniond &getdOrientation();
        const Eigen::Affine3d &getT();
        const Eigen::Matrix<double, _Dofs, 1> &getTorque();

        // set kinematics
        void setq0(const Eigen::Matrix<double, _Dofs, 1> &q);
        void setPosAndOri0(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);
        void updateJointData(const Eigen::Matrix<double, _Dofs, 1> &q, const Eigen::Matrix<double, _Dofs, 1> &theta, const Eigen::Matrix<double, _Dofs, 1> &dq, const Eigen::Matrix<double, _Dofs, 1> &tau);
        void updateEndeffectorData(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E);

        // other dyn api
        void setExternM(const Eigen::Matrix<double, _Dofs, _Dofs> &externM);
        void setExternc(const Eigen::Matrix<double, _Dofs, 1> &externc);
        void setExternG(const Eigen::Matrix<double, _Dofs, 1> &externG);
        void setExternJ(const Eigen::Matrix<double, 6, _Dofs> &externJ);
        const Eigen::Matrix<double, _Dofs, _Dofs> &getExternM();
        const Eigen::Matrix<double, _Dofs, 1> &getExternc(); // 科氏项
        const Eigen::Matrix<double, _Dofs, 1> &getExternG();
        const Eigen::Matrix<double, 6, _Dofs> &getExternJ();

        // pinocchino dyn api
        void calculation(Eigen::Matrix<double, _Dofs, 1> ddq_d);
        const Eigen::Matrix<double, 6, _Dofs> &getJ();
        const Eigen::Matrix<double, _Dofs, 6> &getJ_inv();
        const Eigen::Matrix<double, _Dofs, _Dofs> &getM();
        const Eigen::Matrix<double, _Dofs, _Dofs> &getC();
        const Eigen::Matrix<double, _Dofs, 1> &getG();
    };

}

namespace my_robot
{
    template <int _Dofs>
    Robot<_Dofs>::~Robot()
    {
    }
    template <int _Dofs>
    Robot<_Dofs>::Robot() : q0(Eigen::Matrix<double, _Dofs, 1>::Zero()), position0(Eigen::Matrix<double, 3, 1>::Zero()),
                            orientation0(Eigen::Quaterniond::Identity()), theta(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                            q(Eigen::Matrix<double, _Dofs, 1>::Zero()), dq(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                            position(Eigen::Matrix<double, 3, 1>::Zero()), orientation(Eigen::Quaterniond::Identity()),
                            dposition(Eigen::Matrix<double, 3, 1>::Zero()), dorientation(Eigen::Quaterniond::Identity()),
                            TO2E(Eigen::Affine3d::Identity()), tau(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                            C(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()), M(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                            G(Eigen::Matrix<double, _Dofs, 1>::Zero()), J(Eigen::Matrix<double, 6, _Dofs>::Zero()),
                            dJ(Eigen::Matrix<double, 6, _Dofs>::Zero()), J_inv(Eigen::Matrix<double, _Dofs, 6>::Zero()),
                            externc(Eigen::Matrix<double, _Dofs, 1>::Zero()), externM(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                            externG(Eigen::Matrix<double, _Dofs, 1>::Zero()), externJ(Eigen::Matrix<double, 6, _Dofs>::Zero()),
                            externdJ(Eigen::Matrix<double, 6, _Dofs>::Zero()), externdJ_inv(Eigen::Matrix<double, _Dofs, 6>::Zero())
    {
        // 全部数据初始化为0
    }

    template <int _Dofs>
    const double *const Robot<_Dofs>::getdposMax() const
    {
        return this->dposMax;
    }
    template <int _Dofs>
    const double *const Robot<_Dofs>::getdoriMax() const
    {
        return this->doriMax;
    }
    template <int _Dofs>
    const double *const Robot<_Dofs>::getqMax() const
    {
        return this->qMax;
    }
    template <int _Dofs>
    const double *const Robot<_Dofs>::getqMin() const
    {
        return this->qMin;
    }
    template <int _Dofs>
    const double *const Robot<_Dofs>::getdqLimit() const
    {
        return this->dqLimit;
    }
    template <int _Dofs>
    const double *const Robot<_Dofs>::getddqLimit() const
    {
        return this->ddqLimit;
    }
    // 初值设置和获取
    template <int _Dofs>
    void Robot<_Dofs>::setq0(const Eigen::Matrix<double, _Dofs, 1> &q)
    {
        this->q0 = q;
    }
    template <int _Dofs>
    void Robot<_Dofs>::setPosAndOri0(const Eigen::Vector3d &position0, const Eigen::Quaterniond &orientation0)
    {
        this->position0 = position0;
        this->orientation0 = orientation0;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, 1> &Robot<_Dofs>::getq0()
    {
        return this->q0;
    }
    template <int _Dofs>
    const Eigen::Vector3d &Robot<_Dofs>::getPosition0()
    {
        return this->position0;
    }
    template <int _Dofs>
    const Eigen::Quaterniond &Robot<_Dofs>::getOrientation0()
    {
        return this->orientation0;
    }

    // 获取传感器数据
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, 1> &Robot<_Dofs>::getq()
    {
        return this->q;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, 1> &Robot<_Dofs>::getdq()
    {
        return this->dq;
    }
    template <int _Dofs>
    const Eigen::Vector3d &Robot<_Dofs>::getPosition()
    {
        return this->position;
    }
    template <int _Dofs>
    const Eigen::Quaterniond &Robot<_Dofs>::getOrientation()
    {
        return this->orientation;
    }
    template <int _Dofs>
    const Eigen::Vector3d &Robot<_Dofs>::getdPosition()
    {
        return this->dposition;
    }
    template <int _Dofs>
    const Eigen::Quaterniond &Robot<_Dofs>::getdOrientation()
    {
        return this->dorientation;
    }
    template <int _Dofs>
    const Eigen::Affine3d &Robot<_Dofs>::getT()
    {
        return this->TO2E;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, 1> &Robot<_Dofs>::getTorque()
    {
        return this->tau;
    }

    // 传感器数据更新
    template <int _Dofs>
    void Robot<_Dofs>::updateJointData(const Eigen::Matrix<double, _Dofs, 1> &q, const Eigen::Matrix<double, _Dofs, 1> &theta, const Eigen::Matrix<double, _Dofs, 1> &dq, const Eigen::Matrix<double, _Dofs, 1> &tau)
    {
        this->q = q;
        this->theta = theta;
        this->dq = dq;
        this->tau = tau;
    }
    template <int _Dofs>
    void Robot<_Dofs>::updateEndeffectorData(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, const Eigen::Affine3d &TO2E)
    {
        this->position = position;
        this->orientation = orientation;

        this->dposition = (this->externJ * dq).head(3);
        Eigen::Matrix<double, 3, 1> tmp = (this->externJ * dq).tail(3); // 欧拉角
        this->dorientation = Eigen::AngleAxisd(tmp[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(tmp[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(tmp[2], Eigen::Vector3d::UnitZ());

        this->TO2E = TO2E;
    }

    // pino dyn
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, _Dofs> &Robot<_Dofs>::getM()
    {
        return this->M;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, _Dofs> &Robot<_Dofs>::getC()
    {
        return this->C;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, 1> &Robot<_Dofs>::getG()
    {
        return this->G;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, 6, _Dofs> &Robot<_Dofs>::getJ()
    {
        return this->J;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, 6> &Robot<_Dofs>::getJ_inv()
    {
        return this->J_inv;
    }
    template <int _Dofs>
    void Robot<_Dofs>::calculation(Eigen::Matrix<double, _Dofs, 1> ddq_d)
    {
        pPandaDynLibManager->upDataModel(this->q);
        pPandaDynLibManager->computeKinData(this->J, this->dJ, this->q, this->dq);
        pPandaDynLibManager->computeDynData(this->M, this->C, this->G, this->q, this->dq);
        weightedPseudoInverse(this->J, this->J_inv, this->M);
    }

    // other
    template <int _Dofs>
    void Robot<_Dofs>::setExternM(const Eigen::Matrix<double, _Dofs, _Dofs> &externM)
    {
        this->externM = externM;
    }
    template <int _Dofs>
    void Robot<_Dofs>::setExternc(const Eigen::Matrix<double, _Dofs, 1> &externc)
    {
        this->externc = externc;
    }
    template <int _Dofs>
    void Robot<_Dofs>::setExternG(const Eigen::Matrix<double, _Dofs, 1> &externG)
    {
        this->externG = externG;
    }
    template <int _Dofs>
    void Robot<_Dofs>::setExternJ(const Eigen::Matrix<double, 6, _Dofs> &externJ)
    {
        this->externJ = externJ;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, _Dofs> &Robot<_Dofs>::getExternM()
    {
        return this->externM;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, 1> &Robot<_Dofs>::getExternc()
    {
        return this->externc;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, _Dofs, 1> &Robot<_Dofs>::getExternG()
    {
        return this->externG;
    }
    template <int _Dofs>
    const Eigen::Matrix<double, 6, _Dofs> &Robot<_Dofs>::getExternJ()
    {
        return this->externJ;
    }
}
