#include <algorithm/pandaDynLibManager.h>

pandaDynLibManager *pPandaDynLibManager = nullptr;

pandaDynLibManager::pandaDynLibManager()
{
}

void pandaDynLibManager::forwardKinematics(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q)
{
    pinocchio::forwardKinematics(model, data, q);
}

void pandaDynLibManager::updateFramePlacements(pinocchio::Model &model, pinocchio::Data &data)
{
    pinocchio::updateFramePlacements(model, data);
}

void pandaDynLibManager::computeJointJacobians(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q)
{
    pinocchio::computeJointJacobians(model, data, q);
}

void pandaDynLibManager::computeJointJacobiansTimeVariation(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q, Eigen::Matrix<double, DIM, 1> &dq)
{
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
}

void pandaDynLibManager::rnea(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q, Eigen::Matrix<double, DIM, 1> &dq, Eigen::Matrix<double, DIM, 1> &ddq_d)
{
    pinocchio::rnea(model, data, q, dq, ddq_d);
}

void pandaDynLibManager::computeGeneralizedGravity(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q)
{
    pinocchio::computeGeneralizedGravity(model, data, q);
}

void pandaDynLibManager::computeCoriolisMatrix(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q, Eigen::Matrix<double, DIM, 1> &dq)
{
    pinocchio::computeCoriolisMatrix(model, data, q, dq);
}

void pandaDynLibManager::crba(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q)
{
    pinocchio::crba(model, data, q);
}
