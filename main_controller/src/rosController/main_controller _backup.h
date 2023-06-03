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

#include <main_controller/main_controller_paramConfig.h>
#include <main_controller/paramForDebug.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace main_controller
{

  // 关节空间的动力学控制
  class MainController : public controller_interface::MultiInterfaceController<
                             franka_hw::FrankaModelInterface,
                             hardware_interface::EffortJointInterface,
                             franka_hw::FrankaStateInterface>
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    // 平滑变化率
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; // 机器人全部状态
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; // 机器人的动力学和运动学模型
    std::vector<hardware_interface::JointHandle> joint_handles_; // 关节状态类

    double filter_params{0.005};     // 滤波参数，用于动态调参
    const double delta_tau_max{1.0}; // 最大力矩变化值
    bool firstUpdate = true;         // 用于判断是不是第一个控制周期，计算雅可比导数。

    // 用于计算关节空间期望轨迹
    ros::Duration elapsed_time;

    // 动态参数中调整的目标控制参数
    Eigen::Matrix<double, 7, 7> Kp_target;
    Eigen::Matrix<double, 7, 7> Kv_target;
    Eigen::Matrix<double, 7, 7> K1_target;
    Eigen::Matrix<double, 7, 7> K2_target;
    // 实际控制参数
    Eigen::Matrix<double, 7, 7> Kp;
    Eigen::Matrix<double, 7, 7> Kv;
    Eigen::Matrix<double, 7, 7> K1;
    Eigen::Matrix<double, 7, 7> K2;
    // 关节空间期望轨迹
    Eigen::Matrix<double, 7, 1> q_d;
    Eigen::Matrix<double, 7, 1> dq_d;
    Eigen::Matrix<double, 7, 1> ddq_d;
    Eigen::Matrix<double, 7, 1> q_initial;

    // 动态配置参数
    std::unique_ptr<dynamic_reconfigure::Server<main_controller::main_controller_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void controlParamCallback(main_controller::main_controller_paramConfig &config, uint32_t level);

    // 参数更新函数
    void controllerParamRenew();

    // debug，绘图
    double time = 0;
    std::ofstream myfile;
    ros::Publisher paramForDebug;
  };

} // namespace franka_example_controllers
