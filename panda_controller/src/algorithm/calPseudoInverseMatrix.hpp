#include <Eigen/Dense>

template <int _Dofs>
void weightedPseudoInverse(const Eigen::Matrix<double, 6, _Dofs> &M, Eigen::Matrix<double, _Dofs, 6> &M_pinv, const Eigen::Matrix<double, _Dofs, _Dofs> &W)
{
    M_pinv = W.inverse() * M.transpose() * (M * W.inverse() * M.transpose()).inverse();
}