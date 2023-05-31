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
    pinocchio::Model *pModel;
    pinocchio::Data *pData;
    pinocchio::Model &getpModel();
    pinocchio::Data &getpData();
};


template <int _Dofs>
void weightedPseudoInverse(Eigen::Matrix<double, 6, _Dofs> M, Eigen::Matrix<double, _Dofs, 6>& M_pinv, Eigen::Matrix<double, _Dofs, _Dofs> W) 
{
  M_pinv = W.inverse() * M.transpose() * (M * W.inverse() * M.transpose()).inverse();
}