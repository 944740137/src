#include <franka_example_controllers/pinocchino_interactive.h>


pinLibInteractive *pinInteractive = nullptr;

pinocchio::Model *pModel;
pinocchio::Data *pData;

pinLibInteractive::pinLibInteractive()
{
    static pinocchio::Model model;
    static pinocchio::Data data;
    std::string urdf = std::string("/home/wd/workSpace/ROS/franka_ros/4_workSpaceNullSpace/src/franka_description/robots/panda/panda_withoutHand.urdf");
    std::cout << "------------------" << urdf << "------------------" << std::endl;
    pinocchio::urdf::buildModel(urdf, model);

    pModel = &model;
    data = pinocchio::Data(model);
    pData = &data;
}
// pinocchio::Model &pinLibInteractive::getpModel()
// {
//     return *pModel;
// }
// pinocchio::Data &pinLibInteractive::getpData()
// {
//     return *pData;
// }
void pinLibInteractive::forwardKinematics(const Eigen::Matrix<double, 7, 1> &q)
{
    pinocchio::forwardKinematics(*(pModel), *(pData), q);
}

void pinLibInteractive::updateFramePlacements()
{
    pinocchio::updateFramePlacements(*(pModel), *(pData));
}

void pinLibInteractive::computeJointJacobians(Eigen::Matrix<double, 6, 7> &J, const Eigen::Matrix<double, 7, 1> &q)
{
    pinocchio::computeJointJacobians(*(pModel), *(pData), q);
    J = pData->J;
}

void pinLibInteractive::computeJointJacobiansTimeVariation(const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq)
{
    pinocchio::computeJointJacobiansTimeVariation(*(pModel), *(pData), q, dq);
}

void pinLibInteractive::rnea(const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq, const Eigen::Matrix<double, 7, 1> &ddq_d)
{
    pinocchio::rnea(*(pModel), *(pData), q, dq, ddq_d);
}

void pinLibInteractive::computeGeneralizedGravity(const Eigen::Matrix<double, 7, 1> &q)
{
    pinocchio::computeGeneralizedGravity(*(pModel), *(pData), q);
}

void pinLibInteractive::computeCoriolisMatrix(Eigen::Matrix<double, 7, 7> &C, const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq)
{
    pinocchio::computeCoriolisMatrix(*(pModel), *(pData), q, dq);
    C = pData->C;
}

void pinLibInteractive::crba(const Eigen::Matrix<double, 7, 1> &q)
{
    pinocchio::crba(*(pModel), *(pData), q);
}