// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <panda_controller/panda_controller_paramConfig.h>
#include <panda_controller/paramForDebug.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace panda_controller
{
  // 关节空间的动力学控制
  class PandaController : public controller_interface::MultiInterfaceController<
                              franka_hw::FrankaModelInterface,
                              hardware_interface::EffortJointInterface,
                              franka_hw::FrankaStateInterface>
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; // 机器人全部状态
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; // 机器人的动力学和运动学模型
    std::vector<hardware_interface::JointHandle> joint_handles_; // 关节状态类

    // 平滑力矩
    const double delta_tau_max{1.0};                                                                                                                 // 最大力矩变化值
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    // 动态配置参数
    double filter_params{0.005}; // 滤波参数，用于动态调参
    std::unique_ptr<dynamic_reconfigure::Server<panda_controller::panda_controller_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void controlParamCallback(panda_controller::panda_controller_paramConfig &config, uint32_t level);

    // 发布数据
    ros::Publisher paramForDebug;
    panda_controller::paramForDebug param_debug;

    // 获取传感器数据
    franka::RobotState robot_state;
    Eigen::Matrix<double, 7, 1> q;
    Eigen::Matrix<double, 7, 1> theta;
    Eigen::Matrix<double, 7, 1> dq;
    Eigen::Matrix<double, 7, 1> tau_J_d;
    Eigen::Matrix<double, 7, 1> tau_d;
    Eigen::Affine3d transform; // 齐次变换矩阵
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;

    // 获取动力学数据
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> c;
    Eigen::Matrix<double, 7, 1> G;
    Eigen::Matrix<double, 6, 7> J;
  };

} // namespace franka_example_controllers
