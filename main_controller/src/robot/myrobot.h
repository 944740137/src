#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace my_robot
{
    template <int _Dofs = 7>
    class Robot
    {

    private:
        Eigen::Matrix<double, _Dofs, 1> q;
        Eigen::Matrix<double, _Dofs, 1> dq;
        Eigen::Matrix<double, _Dofs, 1> tau;

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Matrix<double, 4, 4> TO2E;

        Eigen::Matrix<double, _Dofs, _Dofs> C;
        Eigen::Matrix<double, _Dofs, _Dofs> M;
        Eigen::Matrix<double, _Dofs, 1> G;
        Eigen::Matrix<double, _Dofs * 10, _Dofs> Y;

        Eigen::Matrix<double, 6, _Dofs> J;
        Eigen::Matrix<double, _Dofs, 6> J_inv;
        Eigen::Matrix<double, 6, _Dofs> dJ;

    public:
        Eigen::Matrix<double, _Dofs, _Dofs> getM();
        Eigen::Matrix<double, _Dofs, _Dofs> getC();
        Eigen::Matrix<double, _Dofs, 1> getG();

        // franka api
        // bool setM(Eigen::Matrix<double, _Dofs, _Dofs> M);
        // bool setC(Eigen::Matrix<double, _Dofs, _Dofs> C);
        // bool setG(Eigen::Matrix<double, _Dofs, _Dofs> G);
        bool updateJointData(Eigen::Matrix<double, _Dofs, 1> q, Eigen::Matrix<double, _Dofs, 1> dq, Eigen::Matrix<double, _Dofs, 1> tau);
        bool updateEndeffectorData(Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Matrix<double, 4, 4> TO2E);

        // pinocchino api
        bool calculation(Eigen::Matrix<double, _Dofs, 1> ddq_d);
    };
    // typedef Robot<7> Robot7d;
}

