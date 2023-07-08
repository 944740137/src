#include <Eigen/Dense>

template <int _Dofs>
void JointSinTrajectory(Eigen::Matrix<double, _Dofs, 1> selectAxis, double nowTime, double posRatio, double velRatio, Eigen::Matrix<double, _Dofs, 1> &deltaAngle, Eigen::Matrix<double, _Dofs, 1> &dDeltaAngle, Eigen::Matrix<double, _Dofs, 1> &ddDeltaAngle)
{
    double maxPos = M_PI / 4; // 单向
    double maxVel = M_PI / 2; // 单向

    for (size_t i = 0; i < _Dofs; i++)
    {
        if (selectAxis[i] != 0)
            selectAxis[i] = 1;

        deltaAngle[i] = selectAxis[i] * maxPos * std::sin(maxVel * velRatio * nowTime) * posRatio;
        dDeltaAngle[i] = selectAxis[i] * maxPos * (maxVel * velRatio) * (std::cos((maxVel * velRatio) * nowTime)) * posRatio;
        ddDeltaAngle[i] = selectAxis[i] * -maxPos * (maxVel * velRatio) * (maxVel * velRatio) * (std::sin((maxVel * velRatio) * nowTime)) * posRatio;
    }
}
template <int _Dofs>
void JointCosTrajectory(Eigen::Matrix<double, _Dofs, 1> selectAxis, double nowTime, double posRatio, double velRatio, Eigen::Matrix<double, _Dofs, 1> &deltaAngle, Eigen::Matrix<double, _Dofs, 1> &dDeltaAngle, Eigen::Matrix<double, _Dofs, 1> &ddDeltaAngle)
{
    double maxPos = M_PI / 4 * 0.5; // 单向
    double maxVel = M_PI / 2; // 单向

    for (size_t i = 0; i < _Dofs; i++)
    {
        if (selectAxis[i] != 0)
            selectAxis[i] = 1;

        deltaAngle[i] = selectAxis[i] * maxPos * (1 - std::cos(maxVel * velRatio * nowTime)) * posRatio;
        dDeltaAngle[i] = selectAxis[i] * maxPos * (maxVel * velRatio) * (std::sin((maxVel * velRatio) * nowTime)) * posRatio;
        ddDeltaAngle[i] = selectAxis[i] * maxPos * (maxVel * velRatio) * (maxVel * velRatio) * (std::cos((maxVel * velRatio) * nowTime)) * posRatio;
    }
}