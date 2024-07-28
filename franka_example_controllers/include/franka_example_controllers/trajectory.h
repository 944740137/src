#include <Eigen/Dense>
#define DIM 7
// void JointCosTrajectory(const Eigen::Matrix<double, DIM, 1> &selectAxis, double nowTime, double posRatio, double velRatio,
//                         const Eigen::Matrix<double, DIM, 1> &q0, Eigen::Matrix<double, DIM, 1> &q_d,
//                         Eigen::Matrix<double, DIM, 1> &dq_d, Eigen::Matrix<double, DIM, 1> &ddq_d);

// void cartesianTrajectory1(double nowTime, double posRatio, double velRatio,
//                           const Eigen::Vector3d &position0, const Eigen::Vector3d &orientation0,
//                           Eigen::Vector3d &position_d, Eigen::Vector3d &orientation_d,
//                           Eigen::Vector3d &dposition_d, Eigen::Vector3d &dorientation_d,
//                           Eigen::Vector3d &ddposition_d, Eigen::Vector3d &ddorientation_d);

// posRatio 位置比例参数 0～1
// velRatio 位置比例参数 0～1
void JointCosTrajectory(Eigen::Matrix<double, DIM, 1> &selectAxis, double nowTime, double posRatio, double velRatio,
                        const Eigen::Matrix<double, DIM, 1> &q0, const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq,
                        Eigen::Matrix<double, DIM, 1> &q_d, Eigen::Matrix<double, DIM, 1> &dq_d, Eigen::Matrix<double, DIM, 1> &ddq_d,
                        Eigen::Matrix<double, DIM, 1> &qerror, Eigen::Matrix<double, DIM, 1> &dqerror);
void Joint0Trajectory(Eigen::Matrix<double, DIM, 1> &selectAxis, double nowTime, double posRatio, double velRatio,
                      const Eigen::Matrix<double, DIM, 1> &q0, const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq,
                      Eigen::Matrix<double, DIM, 1> &q_d, Eigen::Matrix<double, DIM, 1> &dq_d, Eigen::Matrix<double, DIM, 1> &ddq_d,
                      Eigen::Matrix<double, DIM, 1> &qerror, Eigen::Matrix<double, DIM, 1> &dqerror);

void cartesianTrajectory0(double nowTime, double posRatio, double velRatio, const Eigen::Affine3d &T, const Eigen::Affine3d &T0,
                          const Eigen::Matrix<double, 6, 1> &X0, const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                          Eigen::Matrix<double, 6, 1> &X_d, Eigen::Matrix<double, 6, 1> &dX_d, Eigen::Matrix<double, 6, 1> &ddX_d,
                          Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror);

void cartesianTrajectoryXZ1(double nowTime, double posRatio, double velRatio, const Eigen::Affine3d &T, const Eigen::Affine3d &T0,
                            const Eigen::Matrix<double, 6, 1> &X0, const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                            Eigen::Matrix<double, 6, 1> &X_d, Eigen::Matrix<double, 6, 1> &dX_d, Eigen::Matrix<double, 6, 1> &ddX_d,
                            Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror);

void cartesianTrajectoryXZ2(double nowTime, double posRatio, double velRatio, const Eigen::Affine3d &T, const Eigen::Affine3d &T0,
                            const Eigen::Matrix<double, 6, 1> &X0, const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                            Eigen::Matrix<double, 6, 1> &X_d, Eigen::Matrix<double, 6, 1> &dX_d, Eigen::Matrix<double, 6, 1> &ddX_d,
                            Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror);

void cartesianTrajectoryXZ3(double nowTime, double posRatio, double velRatio, const Eigen::Affine3d &T, const Eigen::Affine3d &T0,
                            const Eigen::Matrix<double, 6, 1> &X0, const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                            Eigen::Matrix<double, 6, 1> &X_d, Eigen::Matrix<double, 6, 1> &dX_d, Eigen::Matrix<double, 6, 1> &ddX_d,
                            Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror);
void cartesianPosTrajectoryX1(double nowTime, double posRatio, double velRatio,
                              const Eigen::Matrix<double, 3, 1> &pos0, const Eigen::Matrix<double, 3, 1> &pos, const Eigen::Matrix<double, 3, 1> &dpos,
                              Eigen::Matrix<double, 3, 1> &pos_d, Eigen::Matrix<double, 3, 1> &dpos_d, Eigen::Matrix<double, 3, 1> &ddpos_d,
                              Eigen::Matrix<double, 3, 1> &pos_error, Eigen::Matrix<double, 3, 1> &dpos_error);
void cartesianPosTrajectory0(double nowTime, double posRatio, double velRatio,
                              const Eigen::Matrix<double, 3, 1> &pos0, const Eigen::Matrix<double, 3, 1> &pos, const Eigen::Matrix<double, 3, 1> &dpos,
                              Eigen::Matrix<double, 3, 1> &pos_d, Eigen::Matrix<double, 3, 1> &dpos_d, Eigen::Matrix<double, 3, 1> &ddpos_d,
                              Eigen::Matrix<double, 3, 1> &pos_error, Eigen::Matrix<double, 3, 1> &dpos_error);