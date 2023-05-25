
// #include <fstream>
// #include <iostream>
// #include <memory>
// #include <mutex>
// #include <string>
// #include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
namespace my_robot
{
    template <typename _Scalar, int _Dofs = 7>
    class Robot
    {
    private:
        Eigen::Matrix<_Scalar, _Dofs, _Dofs> C;
        Eigen::Matrix<_Scalar, _Dofs, _Dofs> M;
        Eigen::Matrix<_Scalar, _Dofs, 1> G;

        Eigen::Matrix<_Scalar, _Dofs, 1> tau_d;
        Eigen::Matrix<_Scalar, _Dofs, 1> tau;

        Eigen::Matrix<_Scalar, 6, _Dofs> J;
        Eigen::Matrix<_Scalar, _Dofs, 6> J_inv;
        Eigen::Matrix<_Scalar, 6, _Dofs> dJ;
        Eigen::Matrix<_Scalar, 4, 4> TO2E;

        Eigen::Matrix<_Scalar, _Dofs, 1> q;
        Eigen::Matrix<_Scalar, _Dofs, 1> dq; 
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

    public:
        Eigen::Matrix<_Scalar, _Dofs, _Dofs> getM();
        Eigen::Matrix<_Scalar, _Dofs, _Dofs> getC();
        Eigen::Matrix<_Scalar, _Dofs, 1> getG();
        bool setM(Eigen::Matrix<_Scalar, _Dofs, _Dofs> M);
        bool setC(Eigen::Matrix<_Scalar, _Dofs, _Dofs> C);
        bool setG(Eigen::Matrix<_Scalar, _Dofs, _Dofs> G);

        bool updateData(Robot<_Scalar,_Dofs> &Robot);
        bool setTau();
    };
    typedef Robot<double,7> Robot7d;
}

template <typename _Scalar, int _Dofs>
Eigen::Matrix<_Scalar, _Dofs, _Dofs> my_robot::Robot<_Scalar, _Dofs>::getM()
{
    return this->M;
};

template <typename _Scalar, int _Dofs>
Eigen::Matrix<_Scalar, _Dofs, _Dofs> my_robot::Robot<_Scalar, _Dofs>::getC()
{
    return this->C;
};

template <typename _Scalar, int _Dofs>
Eigen::Matrix<_Scalar, _Dofs, 1> my_robot::Robot<_Scalar, _Dofs>::getG()
{
    return this->G;
};

template <typename _Scalar, int _Dofs>
bool setM(Eigen::Matrix<_Scalar, _Dofs, _Dofs> M)
{
    this->M = M;
    return true;
}

template <typename _Scalar, int _Dofs>
bool setC(Eigen::Matrix<_Scalar, _Dofs, _Dofs> C)
{
    this->C = C;
    return true;
}

template <typename _Scalar, int _Dofs>
bool setG(Eigen::Matrix<_Scalar, _Dofs, _Dofs> G)
{
    this->G = G;
    return true;
}