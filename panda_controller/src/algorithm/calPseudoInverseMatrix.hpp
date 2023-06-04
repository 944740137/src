#include <Eigen/Dense>

template <int _Dofs>
void weightedPseudoInverse(Eigen::Matrix<double, 6, _Dofs> M, Eigen::Matrix<double, _Dofs, 6> &M_pinv, Eigen::Matrix<double, _Dofs, _Dofs> W)
{
    M_pinv = W.inverse() * M.transpose() * (M * W.inverse() * M.transpose()).inverse();
}