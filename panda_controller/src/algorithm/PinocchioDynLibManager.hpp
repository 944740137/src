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
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex frameId;

public:
    PinocchioDynLibManager(const PinocchioDynLibManager &) = delete;
    void operator=(const PinocchioDynLibManager &) = delete;

    PinocchioDynLibManager() = delete;
    virtual ~PinocchioDynLibManager();
    explicit PinocchioDynLibManager(const std::string urdf, const std::string TcpName);//禁止隐性转换

    // kin
    virtual void upDataModel(Eigen::Matrix<double, _Dofs, 1> &q) = 0;
    virtual void computeTcpJacobian(Eigen::Matrix<double, 6, _Dofs> &J,
                                    Eigen::Matrix<double, 6, _Dofs> &dJ,
                                    const Eigen::Matrix<double, _Dofs, 1> &q,
                                    const Eigen::Matrix<double, _Dofs, 1> &dq) = 0;
    virtual void computeKinData(Eigen::Matrix<double, 6, _Dofs> &J,
                                Eigen::Matrix<double, 6, _Dofs> &dJ,
                                const Eigen::Matrix<double, _Dofs, 1> &q,
                                const Eigen::Matrix<double, _Dofs, 1> &dq) = 0;

    // dyn
    virtual void computeGeneralizedGravity(Eigen::Matrix<double, _Dofs, 1> &G,
                                           const Eigen::Matrix<double, _Dofs, 1> &q) = 0;
    virtual void computeCoriolisMatrix(Eigen::Matrix<double, _Dofs, _Dofs> &C,
                                       const Eigen::Matrix<double, _Dofs, 1> &q,
                                       const Eigen::Matrix<double, _Dofs, 1> &dq) = 0;
    virtual void crba(Eigen::Matrix<double, _Dofs, _Dofs> &M, const Eigen::Matrix<double, _Dofs, 1> &q) = 0;
    virtual void computeDynData(Eigen::Matrix<double, _Dofs, _Dofs> &M,
                                Eigen::Matrix<double, _Dofs, _Dofs> &C,
                                Eigen::Matrix<double, _Dofs, 1> &G,
                                const Eigen::Matrix<double, _Dofs, 1> &q,
                                const Eigen::Matrix<double, _Dofs, 1> &dq) = 0;
};
template <int _Dofs>
PinocchioDynLibManager<_Dofs>::~PinocchioDynLibManager()
{
}
template <int _Dofs>
PinocchioDynLibManager<_Dofs>::PinocchioDynLibManager(const std::string urdf, const std::string TcpName)
{
    std::cout << "[robotController] pinocchino动力学库加载urdf:" << urdf << " 文件" << std::endl;
    std::cout << "[robotController] TcpName:" << TcpName << std::endl;
    pinocchio::urdf::buildModel(urdf, this->model);
    this->data = pinocchio::Data(this->model);
    this->frameId = this->model.getFrameId(TcpName);
}
