#include <algorithm/pinocchino_interactive.h>

pinLibInteractive *pinInteractive = nullptr;
pinLibInteractive::pinLibInteractive()
{
    static pinocchio::Model model;
    static pinocchio::Data data;
    std::string urdf = std::string("/home/wd/Ros_franka/catkin_franka_230512/src/franka_description/robots/panda/panda_withoutHand.urdf");
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
