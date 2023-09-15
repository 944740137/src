#include "PinocchioDynLibManager.hpp"

#define DIM 7

class PandaDynLibManager : public PinocchioDynLibManager<DIM>
{

public:
  PandaDynLibManager(const PandaDynLibManager &) = delete;
  void operator=(const PandaDynLibManager &) = delete;

  PandaDynLibManager() = delete;
  ~PandaDynLibManager();

  explicit PandaDynLibManager(const std::string urdf, const std::string TcpName);

  void upDataModel(Eigen::Matrix<double, DIM, 1> &q);
  void computeTcpJacobian(Eigen::Matrix<double, 6, DIM> &J,
                          Eigen::Matrix<double, 6, DIM> &dJ,
                          const Eigen::Matrix<double, DIM, 1> &q,
                          const Eigen::Matrix<double, DIM, 1> &dq);
  void computeKinData(Eigen::Matrix<double, 6, DIM> &J,
                      Eigen::Matrix<double, 6, DIM> &dJ,
                      const Eigen::Matrix<double, DIM, 1> &q,
                      const Eigen::Matrix<double, DIM, 1> &dq);

  void computeGeneralizedGravity(Eigen::Matrix<double, DIM, 1> &G, const Eigen::Matrix<double, DIM, 1> &q);
  void computeCoriolisMatrix(Eigen::Matrix<double, DIM, DIM> &C,
                             const Eigen::Matrix<double, DIM, 1> &q,
                             const Eigen::Matrix<double, DIM, 1> &dq);
  void crba(Eigen::Matrix<double, DIM, DIM> &M, const Eigen::Matrix<double, DIM, 1> &q);
  void computeDynData(Eigen::Matrix<double, DIM, DIM> &M,
                      Eigen::Matrix<double, DIM, DIM> &C,
                      Eigen::Matrix<double, DIM, 1> &G,
                      const Eigen::Matrix<double, DIM, 1> &q,
                      const Eigen::Matrix<double, DIM, 1> &dq);
};
//
extern PandaDynLibManager *pPandaDynLibManager;
