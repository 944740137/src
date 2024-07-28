#include <Eigen/Dense>
#include "franka_example_controllers/trajectory.h"
#include <iostream>
void calCartesianError(const Eigen::Affine3d &T,
                       const Eigen::Matrix<double, 6, 1> &X_d, const Eigen::Matrix<double, 6, 1> &dX_d,
                       const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                       Eigen::Quaterniond &orientation_d, Eigen::Quaterniond &dorientation_d,
                       Eigen::Quaterniond &orientation, Eigen::Quaterniond &dorientation,
                       Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror)
{
    // 笛卡尔误差计算比较特殊

    // 误差
    Xerror.head(3) << (X_d - X).head(3);
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
    {
        orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond errorOrientation(orientation.inverse() * orientation_d);
    Xerror.tail(3) << errorOrientation.x(), errorOrientation.y(), errorOrientation.z();
    Xerror.tail(3) << T.rotation() * Xerror.tail(3); // 提取后三个元素

    // 误差导数
    dXerror.head(3) << (dX_d - dX).head(3);
    if (dorientation_d.coeffs().dot(dorientation.coeffs()) < 0.0)
    {
        dorientation.coeffs() << -dorientation.coeffs();
    }
    Eigen::Quaterniond derrorOrientation(dorientation.inverse() * dorientation_d);
    dXerror.tail(3) << derrorOrientation.x(), derrorOrientation.y(), derrorOrientation.z();
    dXerror.tail(3) << T.rotation() * dXerror.tail(3); // 提取后三个元素
}

void cartesianTrajectory0(double nowTime, double posRatio, double velRatio, const Eigen::Affine3d &T, const Eigen::Affine3d &T0,
                          const Eigen::Matrix<double, 6, 1> &X0, const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                          Eigen::Matrix<double, 6, 1> &X_d, Eigen::Matrix<double, 6, 1> &dX_d, Eigen::Matrix<double, 6, 1> &ddX_d,
                          Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror)
{
    if (nowTime == 0)
        std::cout << "[---------------] cartesianTrajectory0" << std::endl;
    // 初始化（用不到的自由度初始成与当前广义坐标一样，速度加速度为0）
    X_d = X0;
    dX_d.setZero();
    ddX_d.setZero();

    // 期望位置函数

    // 期望位置赋值

    // 期望姿态函数
    Eigen::Quaterniond orientation0(T0.rotation());
    Eigen::Quaterniond orientation(T.rotation());
    Eigen::Quaterniond orientation_d = orientation0;
    Eigen::Quaterniond dorientation = Eigen::AngleAxisd(dX[3], Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(dX[4], Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(dX[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond dorientation_d = Eigen::AngleAxisd(dX_d[3], Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(dX_d[4], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(dX_d[5], Eigen::Vector3d::UnitZ());

    // 期望姿态赋值

    // 误差计算
    calCartesianError(T, X_d, dX_d, X, dX, orientation_d, dorientation_d, orientation, dorientation, Xerror, dXerror);
}

void JointCosTrajectory(Eigen::Matrix<double, DIM, 1> &selectAxis, double nowTime, double posRatio, double velRatio,
                        const Eigen::Matrix<double, DIM, 1> &q0, const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq,
                        Eigen::Matrix<double, DIM, 1> &q_d, Eigen::Matrix<double, DIM, 1> &dq_d, Eigen::Matrix<double, DIM, 1> &ddq_d,
                        Eigen::Matrix<double, DIM, 1> &qerror, Eigen::Matrix<double, DIM, 1> &dqerror)
{
    static double maxPos = M_PI / 4 * 0.5; // 单向
    static double maxVel = M_PI / 2 * 0.5; // 单向
    static Eigen::Matrix<double, DIM, 1> deltaAngle;
    static Eigen::Matrix<double, DIM, 1> dDeltaAngle;
    static Eigen::Matrix<double, DIM, 1> ddDeltaAngle;
    for (size_t i = 0; i < DIM; i++)
    {
        if (selectAxis[i] != 0)
            selectAxis[i] = 1;

        deltaAngle[i] = selectAxis[i] * maxPos * (1 - std::cos(maxVel * velRatio * nowTime)) * posRatio;
        dDeltaAngle[i] = selectAxis[i] * maxPos * (maxVel * velRatio) * (std::sin((maxVel * velRatio) * nowTime)) * posRatio;
        ddDeltaAngle[i] = selectAxis[i] * maxPos * (maxVel * velRatio) * (maxVel * velRatio) * (std::cos((maxVel * velRatio) * nowTime)) * posRatio;
    }
    q_d = q0 + deltaAngle;
    dq_d = dDeltaAngle;
    ddq_d = ddDeltaAngle;

    qerror = q_d - q;
    dqerror = dq_d - dq;
}

void Joint0Trajectory(Eigen::Matrix<double, DIM, 1> &selectAxis, double nowTime, double posRatio, double velRatio,
                      const Eigen::Matrix<double, DIM, 1> &q0, const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq,
                      Eigen::Matrix<double, DIM, 1> &q_d, Eigen::Matrix<double, DIM, 1> &dq_d, Eigen::Matrix<double, DIM, 1> &ddq_d,
                      Eigen::Matrix<double, DIM, 1> &qerror, Eigen::Matrix<double, DIM, 1> &dqerror)
{
    q_d = q0;
    dq_d = Eigen::MatrixXd::Zero(7, 1);
    ddq_d = Eigen::MatrixXd::Zero(7, 1);

    qerror = q_d - q;
    dqerror = dq_d - dq;
}

void cartesianTrajectoryXZ1(double nowTime, double posRatio, double velRatio, const Eigen::Affine3d &T, const Eigen::Affine3d &T0,
                            const Eigen::Matrix<double, 6, 1> &X0, const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                            Eigen::Matrix<double, 6, 1> &X_d, Eigen::Matrix<double, 6, 1> &dX_d, Eigen::Matrix<double, 6, 1> &ddX_d,
                            Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror)
{
    if (nowTime == 0)
        std::cout << "[---------------] cartesianTrajectoryXZ: 1" << std::endl;
    // 初始化（用不到的自由度初始成与当前广义坐标一样，速度加速度为0）
    X_d = X0;
    dX_d.setZero();
    ddX_d.setZero();

    // 期望位置函数
    double alpha = 2 * M_PI / 5 * velRatio;
    double radius = 0.11;
    double deltaX = radius * (std::cos(alpha * nowTime) - 1) * posRatio;
    double dDeltaX = radius * alpha * std::sin(alpha * nowTime) * posRatio;
    double ddDeltaX = radius * alpha * alpha * std::cos(alpha * nowTime) * posRatio;
    double deltaZ = radius * std::sin(alpha * nowTime) * posRatio;
    double dDeltaZ = radius * alpha * std::cos(alpha * nowTime) * posRatio;
    double ddDeltaZ = -radius * alpha * alpha * std::sin(alpha * nowTime) * posRatio;

    // 期望位置赋值
    X_d[0] = X0[0] + deltaX;
    dX_d[0] = dDeltaX;
    ddX_d[0] = ddDeltaX;
    X_d[2] = X0[2] + deltaZ;
    dX_d[2] = dDeltaZ;
    ddX_d[2] = ddDeltaZ;

    // 期望姿态函数
    Eigen::Quaterniond orientation0(T0.rotation());
    Eigen::Quaterniond orientation(T.rotation());
    Eigen::Quaterniond orientation_d = orientation0;
    Eigen::Quaterniond dorientation = Eigen::AngleAxisd(dX[3], Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(dX[4], Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(dX[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond dorientation_d = Eigen::AngleAxisd(dX_d[3], Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(dX_d[4], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(dX_d[5], Eigen::Vector3d::UnitZ());

    // 期望姿态赋值
    X_d.tail(3) << orientation_d.toRotationMatrix().eulerAngles(2, 1, 0);
    dX_d.tail(3) << dorientation_d.toRotationMatrix().eulerAngles(2, 1, 0);
    // ddX_d.tail(3) << ddorientation_d.toRotationMatrix().eulerAngles(2, 1, 0); //todo 插值：贝塞尔 四元数

    // 误差计算
    calCartesianError(T, X_d, dX_d, X, dX, orientation_d, dorientation_d, orientation, dorientation, Xerror, dXerror);
}

void cartesianTrajectoryXZ2(double nowTime, double posRatio, double velRatio, const Eigen::Affine3d &T, const Eigen::Affine3d &T0,
                            const Eigen::Matrix<double, 6, 1> &X0, const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                            Eigen::Matrix<double, 6, 1> &X_d, Eigen::Matrix<double, 6, 1> &dX_d, Eigen::Matrix<double, 6, 1> &ddX_d,
                            Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror)
{
    if (nowTime == 0)
        std::cout << "[---------------] cartesianTrajectoryXZ: 2" << std::endl;
    // 初始化（用不到的自由度初始成与当前广义坐标一样，速度加速度为0）
    X_d = X0;
    dX_d.setZero();
    ddX_d.setZero();

    // 期望位置函数
    double alpha = 2 * M_PI / 5 * velRatio;
    double radius = 0.15;
    double deltaX = radius * std::sin(alpha * nowTime) * posRatio;
    double dDeltaX = radius * alpha * std::cos(alpha * nowTime) * posRatio;
    double ddDeltaX = -radius * alpha * alpha * std::sin(alpha * nowTime) * posRatio;
    double deltaZ = radius * (1 - std::cos(alpha * nowTime)) * posRatio;
    double dDeltaZ = -radius * alpha * std::sin(alpha * nowTime) * posRatio;
    double ddDeltaZ = -radius * alpha * alpha * std::cos(alpha * nowTime) * posRatio;

    // 期望位置赋值
    X_d[0] = X0[0] + deltaX;
    dX_d[0] = dDeltaX;
    ddX_d[0] = ddDeltaX;
    X_d[2] = X0[2] + deltaZ;
    dX_d[2] = dDeltaZ;
    ddX_d[2] = ddDeltaZ;

    // 期望姿态函数
    Eigen::Quaterniond orientation0(T0.rotation());
    Eigen::Quaterniond orientation(T.rotation());
    Eigen::Quaterniond orientation_d = orientation0;
    Eigen::Quaterniond dorientation = Eigen::AngleAxisd(dX[3], Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(dX[4], Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(dX[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond dorientation_d = Eigen::AngleAxisd(dX_d[3], Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(dX_d[4], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(dX_d[5], Eigen::Vector3d::UnitZ());

    // 期望姿态赋值
    X_d.tail(3) << orientation_d.toRotationMatrix().eulerAngles(2, 1, 0);
    dX_d.tail(3) << dorientation_d.toRotationMatrix().eulerAngles(2, 1, 0);
    // ddX_d.tail(3) << ddorientation_d.toRotationMatrix().eulerAngles(2, 1, 0); //todo 插值：贝塞尔 四元数

    // 误差计算
    calCartesianError(T, X_d, dX_d, X, dX, orientation_d, dorientation_d, orientation, dorientation, Xerror, dXerror);
}

void cartesianTrajectoryXZ3(double nowTime, double posRatio, double velRatio, const Eigen::Affine3d &T, const Eigen::Affine3d &T0,
                            const Eigen::Matrix<double, 6, 1> &X0, const Eigen::Matrix<double, 6, 1> &X, const Eigen::Matrix<double, 6, 1> &dX,
                            Eigen::Matrix<double, 6, 1> &X_d, Eigen::Matrix<double, 6, 1> &dX_d, Eigen::Matrix<double, 6, 1> &ddX_d,
                            Eigen::Matrix<double, 6, 1> &Xerror, Eigen::Matrix<double, 6, 1> &dXerror)
{
    if (nowTime == 0)
        std::cout << "[---------------] cartesianTrajectoryXZ: 3" << std::endl;

    // 初始化（用不到的自由度初始成与当前广义坐标一样，速度加速度为0）
    X_d = X0;
    dX_d.setZero();
    ddX_d.setZero();

    // 期望位置函数
    double alpha = 2 * M_PI / 5 * velRatio;
    double radius = 0.16;
    double deltaX = radius * (1 - std::cos(alpha * nowTime)) * posRatio;
    double dDeltaX = -radius * alpha * std::sin(alpha * nowTime) * posRatio;
    double ddDeltaX = -radius * alpha * alpha * std::cos(alpha * nowTime) * posRatio;
    double deltaZ = radius * std::sin(alpha * nowTime) * posRatio;
    double dDeltaZ = radius * alpha * std::cos(alpha * nowTime) * posRatio;
    double ddDeltaZ = -radius * alpha * alpha * std::sin(alpha * nowTime) * posRatio;

    // 期望位置赋值
    X_d[0] = X0[0] + deltaX;
    dX_d[0] = dDeltaX;
    ddX_d[0] = ddDeltaX;
    X_d[2] = X0[2] + deltaZ;
    dX_d[2] = dDeltaZ;
    ddX_d[2] = ddDeltaZ;

    // 期望姿态函数
    Eigen::Quaterniond orientation0(T0.rotation());
    Eigen::Quaterniond orientation(T.rotation());
    Eigen::Quaterniond orientation_d = orientation0;
    Eigen::Quaterniond dorientation = Eigen::AngleAxisd(dX[3], Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(dX[4], Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(dX[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond dorientation_d = Eigen::AngleAxisd(dX_d[3], Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(dX_d[4], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(dX_d[5], Eigen::Vector3d::UnitZ());

    // 期望姿态赋值
    X_d.tail(3) << orientation_d.toRotationMatrix().eulerAngles(2, 1, 0);
    dX_d.tail(3) << dorientation_d.toRotationMatrix().eulerAngles(2, 1, 0);
    // ddX_d.tail(3) << ddorientation_d.toRotationMatrix().eulerAngles(2, 1, 0); //todo 插值：贝塞尔 四元数

    // 误差计算
    calCartesianError(T, X_d, dX_d, X, dX, orientation_d, dorientation_d, orientation, dorientation, Xerror, dXerror);
}

void cartesianPosTrajectoryX1(double nowTime, double posRatio, double velRatio,
                              const Eigen::Matrix<double, 3, 1> &pos0, const Eigen::Matrix<double, 3, 1> &pos, const Eigen::Matrix<double, 3, 1> &dpos,
                              Eigen::Matrix<double, 3, 1> &pos_d, Eigen::Matrix<double, 3, 1> &dpos_d, Eigen::Matrix<double, 3, 1> &ddpos_d,
                              Eigen::Matrix<double, 3, 1> &pos_error, Eigen::Matrix<double, 3, 1> &dpos_error)
{
    if (nowTime == 0)
        std::cout << "[---------------] cartesianTrajectoryXZ: 3" << std::endl;

    // 初始化（用不到的自由度初始成与当前广义坐标一样，速度加速度为0）
    pos_d = pos0;
    dpos_d.setZero();
    ddpos_d.setZero();

    // 期望位置函数
    double alpha = 2 * M_PI / 5 * velRatio;
    double radius = 0.16;
    double deltaX = radius * (1 - std::cos(alpha * nowTime)) * posRatio;
    double dDeltaX = -radius * alpha * std::sin(alpha * nowTime) * posRatio;
    double ddDeltaX = -radius * alpha * alpha * std::cos(alpha * nowTime) * posRatio;
    double deltaZ = radius * std::sin(alpha * nowTime) * posRatio;
    double dDeltaZ = radius * alpha * std::cos(alpha * nowTime) * posRatio;
    double ddDeltaZ = -radius * alpha * alpha * std::sin(alpha * nowTime) * posRatio;

    // 期望位置赋值
    pos_d[0] = pos0[0] + deltaX;
    dpos_d[0] = dDeltaX;
    ddpos_d[0] = ddDeltaX;

    // 误差计算
    pos_error = pos_d - pos;
    dpos_error = dpos_d - dpos;
}

void cartesianPosTrajectory0(double nowTime, double posRatio, double velRatio,
                             const Eigen::Matrix<double, 3, 1> &pos0, const Eigen::Matrix<double, 3, 1> &pos, const Eigen::Matrix<double, 3, 1> &dpos,
                             Eigen::Matrix<double, 3, 1> &pos_d, Eigen::Matrix<double, 3, 1> &dpos_d, Eigen::Matrix<double, 3, 1> &ddpos_d,
                             Eigen::Matrix<double, 3, 1> &pos_error, Eigen::Matrix<double, 3, 1> &dpos_error)
{
    if (nowTime == 0)
        std::cout << "[---------------] cartesianTrajectoryXZ: 3" << std::endl;

    // 初始化（用不到的自由度初始成与当前广义坐标一样，速度加速度为0）
    pos_d = pos0;
    dpos_d.setZero();
    ddpos_d.setZero();

    // 期望位置函数
    double alpha = 2 * M_PI / 5 * velRatio;
    double radius = 0.16;
    double deltaX = radius * (1 - std::cos(alpha * nowTime)) * posRatio;
    double dDeltaX = -radius * alpha * std::sin(alpha * nowTime) * posRatio;
    double ddDeltaX = -radius * alpha * alpha * std::cos(alpha * nowTime) * posRatio;
    double deltaZ = radius * std::sin(alpha * nowTime) * posRatio;
    double dDeltaZ = radius * alpha * std::cos(alpha * nowTime) * posRatio;
    double ddDeltaZ = -radius * alpha * alpha * std::sin(alpha * nowTime) * posRatio;

    // 误差计算
    pos_error = pos_d - pos;
    dpos_error = dpos_d - dpos;
}