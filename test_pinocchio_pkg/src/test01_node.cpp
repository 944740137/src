#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "Pinocchio_test");
    // ros::NodeHandle nh;
    using namespace pinocchio;

    // You should change here to set up your own URD F file or just pass it as an argument of this example.
    const std::string urdf_filename = (argc <= 1) ? std::string("/home/wd/Ros_franka/catkin_franka_230512/src/test_pinocchio_pkg/urdf/panda/panda_withoutHand.urdf") : argv[1];

    // Load the urdf model
    Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    std::cout << "-----------" << std::endl;
    std::cout << "model name: " << model.name << std::endl;

    Data data(model);
    // Create data required by the algorithms

    // Sample a random configuration
    Eigen::VectorXd q = randomConfiguration(model);
    std::cout << "q: " << q.transpose() << std::endl;

    // Perform the forward kinematics over the kinematic tree
    forwardKinematics(model, data, q);

    // Print out the placement of each joint of the kinematic tree
    for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left
                  << model.names[joint_id] << ": "
                  << std::fixed << std::setprecision(2)
                  << data.oMi[joint_id].translation().transpose()
                  << std::endl;

                   
    std::cout << "-----------"<< std::endl;
}