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
    PinocchioDynLibManager(const PinocchioDynLibManager &) = delete;
    void operator=(const PinocchioDynLibManager &) = delete;

    PinocchioDynLibManager() = delete;
    virtual ~PinocchioDynLibManager();

    PinocchioDynLibManager(std::string urdf);

    virtual pinocchio::Model* getpModel();
    virtual pinocchio::Data* getpData();

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
PinocchioDynLibManager<_Dofs>::~PinocchioDynLibManager()
{
}
template <int _Dofs>
PinocchioDynLibManager<_Dofs>::PinocchioDynLibManager(std::string urdf)
{
    static pinocchio::Model model;
    static pinocchio::Data data;

    std::cout << "[robotController] pinocchino动力学库加载urdf:" << urdf << " 文件" << std::endl;
    pinocchio::urdf::buildModel(urdf, model);
    data = pinocchio::Data(model);
    pModel = &model;
    pData = &data;
}

template <int _Dofs>
pinocchio::Model *PinocchioDynLibManager<_Dofs>::getpModel()
{
    return pModel;
}

template <int _Dofs>
pinocchio::Data *PinocchioDynLibManager<_Dofs>::getpData()
{
    return pData;
}
