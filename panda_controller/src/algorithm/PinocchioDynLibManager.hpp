#pragma once

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

template <int _Dofs>
class PinocchioDynLibManager
{
public:
    pinocchio::Model *pModel;
    pinocchio::Data *pData;

public:
    // void myforwardKinematics(pinocchio::Model model_, pinocchio::Data data_, VectorXd q_pin_, VectorXd v_pin_, VectorXd a_pin_);
    PinocchioDynLibManager();
    virtual pinocchio::Model &getpModel();
    virtual pinocchio::Data &getpData();

    virtual void forwardKinematics(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, _Dofs, 1> &q) = 0;
    virtual void updateFramePlacements(pinocchio::Model &model, pinocchio::Data &data) = 0;
    virtual void computeJointJacobians(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, _Dofs, 1> &q) = 0;
    virtual void computeJointJacobiansTimeVariation(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, _Dofs, 1> &q, Eigen::Matrix<double, _Dofs, 1> &dq) = 0;
    virtual void rnea(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, _Dofs, 1> &q, Eigen::Matrix<double, _Dofs, 1> &dq, Eigen::Matrix<double, _Dofs, 1> &ddq_d) = 0;
    virtual void computeGeneralizedGravity(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, _Dofs, 1> &q) = 0;
    virtual void computeCoriolisMatrix(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, _Dofs, 1> &q, Eigen::Matrix<double, _Dofs, 1> &dq) = 0;
    virtual void crba(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, _Dofs, 1> &q) = 0;
};

template <int _Dofs>
PinocchioDynLibManager<_Dofs>::PinocchioDynLibManager()
{
    static pinocchio::Model model;
    static pinocchio::Data data;
    std::string urdf = std::string("/home/wd/workSpace/ROS/franka_ros/3_workSpaceMain/src/franka_description/robots/panda/panda_withoutHand.urdf");
    std::cout << "[robotController] pinocchino动力学库加载urdf：" << urdf << " 文件" << std::endl;
    pinocchio::urdf::buildModel(urdf, model);
    pModel = &model;
    data = pinocchio::Data(model);
    pData = &data;
}

template <int _Dofs>
pinocchio::Model &PinocchioDynLibManager<_Dofs>::getpModel()
{
    return *pModel;
}

template <int _Dofs>
pinocchio::Data &PinocchioDynLibManager<_Dofs>::getpData()
{
    return *pData;
}
