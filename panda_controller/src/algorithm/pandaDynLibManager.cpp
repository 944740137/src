#include <algorithm/pandaDynLibManager.h>

PandaDynLibManager *pPandaDynLibManager = nullptr;

PandaDynLibManager::~PandaDynLibManager()
{
}
PandaDynLibManager::PandaDynLibManager(const std::string urdf, const std::string TcpName) : PinocchioDynLibManager(urdf, TcpName) // 显式调用父类有参构造
{
}

void PandaDynLibManager::upDataModel(Eigen::Matrix<double, DIM, 1> &q)
{
    pinocchio::forwardKinematics(this->model, this->data, q);
    pinocchio::updateFramePlacements(this->model, this->data);
}

// kin
void PandaDynLibManager::computeTcpJacobian(Eigen::Matrix<double, 6, DIM> &J,
                                            Eigen::Matrix<double, 6, DIM> &dJ,
                                            const Eigen::Matrix<double, DIM, 1> &q,
                                            const Eigen::Matrix<double, DIM, 1> &dq)
{
    pinocchio::computeJointJacobians(this->model, this->data);
    pinocchio::computeJointJacobiansTimeVariation(this->model, this->data, q, dq);
    //J0
    pinocchio::getFrameJacobian(this->model, this->data, this->frameId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
    pinocchio::getFrameJacobianTimeVariation(this->model, this->data, this->frameId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
    //JE
    // pinocchio::getFrameJacobian(this->model, this->data, frameId, pinocchio::ReferenceFrame::LOCAL, J_pin2);
    // pinocchio::getFrameJacobianTimeVariation(this->model, this->data, frameId, pinocchio::ReferenceFrame::LOCAL, dJ_pin2);
}
void PandaDynLibManager::computeKinData(Eigen::Matrix<double, 6, DIM> &J,
                                        Eigen::Matrix<double, 6, DIM> &dJ,
                                        const Eigen::Matrix<double, DIM, 1> &q,
                                        const Eigen::Matrix<double, DIM, 1> &dq)
{
    this->computeTcpJacobian(J, dJ, q, dq);
}

// dyn
void PandaDynLibManager::computeGeneralizedGravity(Eigen::Matrix<double, DIM, 1> &G, const Eigen::Matrix<double, DIM, 1> &q)
{
    G = pinocchio::computeGeneralizedGravity(this->model, this->data, q);
}
void PandaDynLibManager::computeCoriolisMatrix(Eigen::Matrix<double, DIM, DIM> &C,
                                               const Eigen::Matrix<double, DIM, 1> &q,
                                               const Eigen::Matrix<double, DIM, 1> &dq)
{
    C = pinocchio::computeCoriolisMatrix(this->model, this->data, q, dq);
}
void PandaDynLibManager::crba(Eigen::Matrix<double, DIM, DIM> &M, const Eigen::Matrix<double, DIM, 1> &q)
{
    pinocchio::crba(this->model, this->data, q);
    this->data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    M = this->data.M;
}
void PandaDynLibManager::computeDynData(Eigen::Matrix<double, DIM, DIM> &M,
                                        Eigen::Matrix<double, DIM, DIM> &C,
                                        Eigen::Matrix<double, DIM, 1> &G,
                                        const Eigen::Matrix<double, DIM, 1> &q,
                                        const Eigen::Matrix<double, DIM, 1> &dq)
{
    computeGeneralizedGravity(G, q);
    computeCoriolisMatrix(C, q, dq);
    crba(M, q);
}