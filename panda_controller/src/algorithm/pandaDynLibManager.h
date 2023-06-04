#include "PinocchioDynLibManager.hpp"

#define DIM 7

class pandaDynLibManager :  public PinocchioDynLibManager<DIM>
{

public:
  pandaDynLibManager();
  void forwardKinematics(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q);
  void updateFramePlacements(pinocchio::Model &model, pinocchio::Data &data);
  void computeJointJacobians(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q);
  void computeJointJacobiansTimeVariation(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q, Eigen::Matrix<double, DIM, 1> &dq);
  void rnea(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q, Eigen::Matrix<double, DIM, 1> &dq, Eigen::Matrix<double, DIM, 1> &ddq_d);
  void computeGeneralizedGravity(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q);
  void computeCoriolisMatrix(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q, Eigen::Matrix<double, DIM, 1> &dq);
  void crba(pinocchio::Model &model, pinocchio::Data &data, Eigen::Matrix<double, DIM, 1> &q);
};
//
extern pandaDynLibManager *pPandaDynLibManager;
