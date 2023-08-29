#pragma once
#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"


class pinLibInteractive
{
public:
    pinLibInteractive();

public:
    // void myforwardKinematics(pinocchio::Model model_, pinocchio::Data data_, VectorXd q_pin_, VectorXd v_pin_, VectorXd a_pin_);

    // pinocchio::Model &getpModel();
    // pinocchio::Data &getpData();
    void forwardKinematics(const Eigen::Matrix<double, 7, 1> &q);
    void updateFramePlacements();
    void computeJointJacobians(Eigen::Matrix<double, 6, 7> &J, const Eigen::Matrix<double, 7, 1> &q);
    void computeJointJacobiansTimeVariation(const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq);
    void rnea(const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq, const Eigen::Matrix<double, 7, 1> &ddq_d);
    void computeGeneralizedGravity(const Eigen::Matrix<double, 7, 1> &q);
    void computeCoriolisMatrix(Eigen::Matrix<double, 7, 7> &C, const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq);
    void crba(const Eigen::Matrix<double, 7, 1> &q);
};
