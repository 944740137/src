#pragma once
#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"

class PandaDynLibManager
{
public:
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex frameId; // 末端

public:
    PandaDynLibManager(const PandaDynLibManager &) = delete;
    void operator=(const PandaDynLibManager &) = delete;

    PandaDynLibManager() = delete;
    virtual ~PandaDynLibManager();
    explicit PandaDynLibManager(const std::string urdf, const std::string TcpName); // 禁止隐性转换

    // kin
    void upDataModel(Eigen::Matrix<double, 7, 1> &q);
    void computeTcpJacobian(Eigen::Matrix<double, 6, 7> &J,
                            Eigen::Matrix<double, 6, 7> &dJ,
                            const Eigen::Matrix<double, 7, 1> &q,
                            const Eigen::Matrix<double, 7, 1> &dq);
    void computeKinData(Eigen::Matrix<double, 6, 7> &J,
                        Eigen::Matrix<double, 6, 7> &dJ,
                        const Eigen::Matrix<double, 7, 1> &q,
                        const Eigen::Matrix<double, 7, 1> &dq);
    // dyn
    void computeGeneralizedGravity(Eigen::Matrix<double, 7, 1> &G, const Eigen::Matrix<double, 7, 1> &q);
    void computeCoriolisMatrix(Eigen::Matrix<double, 7, 7> &C,
                               const Eigen::Matrix<double, 7, 1> &q,
                               const Eigen::Matrix<double, 7, 1> &dq);
    void crba(Eigen::Matrix<double, 7, 7> &M, const Eigen::Matrix<double, 7, 1> &q);
    void computeDynData(Eigen::Matrix<double, 7, 7> &M,
                        Eigen::Matrix<double, 7, 7> &C,
                        Eigen::Matrix<double, 7, 1> &G,
                        const Eigen::Matrix<double, 7, 1> &q,
                        const Eigen::Matrix<double, 7, 1> &dq);
};
