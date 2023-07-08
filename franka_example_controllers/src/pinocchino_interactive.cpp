#include <algorithm/pinocchino_interactive.h>

pinLibInteractive *pinInteractive = nullptr;
pinLibInteractive::pinLibInteractive()
{
    static pinocchio::Model model;
    static pinocchio::Data data;
    std::string urdf = std::string("/home/wd/franka_ros/3_workspace/src/franka_description/robots/panda/panda_withHand.urdf");
    std::cout << "------------------" << urdf << "------------------" << std::endl;
    pinocchio::urdf::buildModel(urdf, model);
    pModel = &model;
    data = pinocchio::Data(model);
    pData = &data;
}
pinocchio::Model &pinLibInteractive::getpModel()
{
    return *pModel;
}
pinocchio::Data &pinLibInteractive::getpData()
{
    return *pData;
}
// void pinLibInteractive::myforwardKinematics(pinocchio::Model model_, pinocchio::Data data_, VectorXd q_pin_, VectorXd v_pin_, VectorXd a_pin_)
// {
//     pinocchio::forwardKinematics(model_, data_, q_pin_, v_pin_, a_pin_);
// }
// void tmp()
// {
//     pinInteractive = new pinLibInteractive();
//     pinocchio::Data *data = pinInteractive->getpData();
//     pinocchio::Model *model = pinInteractive->getpModel();
//     Eigen::VectorXd q_pin(model->nq), v_pin(model->nv), a_pin(model->nv), tau_pin(model->nq);
//     pinocchio::forwardKinematics(*model, *data, q_pin, v_pin, a_pin);
// }