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

#include <franka_example_controllers/dynamic_control_paramConfig.h>
#include <franka_example_controllers/paramForDebug.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers
{

  // 关节空间的动力学控制
  class JointDynamicControlController : public controller_interface::MultiInterfaceController<
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
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::dynamic_control_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void controlParamCallback(franka_example_controllers::dynamic_control_paramConfig &config, uint32_t level);

    // 参数更新函数
    void controllerParamRenew();

    // debug，绘图
    int time = 0;
    std::ofstream myfile;
    ros::Publisher paramForDebug;

    // pin
    // pinocchioLib lib_;
    // pinocchio::Model pinModel;
    // pinocchio::Data pPinData;
  };

  // 笛卡尔空间的动力学控制
  class CartesianDynamicControlController : public controller_interface::MultiInterfaceController<
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
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d);

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; // 机器人全部状态
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; // 机器人的动力学和运动学模型
    std::vector<hardware_interface::JointHandle> joint_handles_; // 关节状态类

    double filter_params{0.005};     // 滤波参数，调整目标位置与阻抗变化速率
    const double delta_tau_max{1.0}; // 最大力矩变化值
    bool firstUpdate = true;         // 用于判断是不是第一个控制周期，计算雅可比导数。

    // 用于计算笛卡尔空间期望轨迹
    ros::Duration elapsed_time;

    // 主任务
    Eigen::Matrix<double, 6, 6> Kp_target;
    Eigen::Matrix<double, 6, 6> Kv_target;
    Eigen::Matrix<double, 6, 6> Kp;
    Eigen::Matrix<double, 6, 6> Kv;

    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;
    Eigen::Vector3d position_initial;
    Eigen::Quaterniond orientation_initial;

    // 用于数值微分
    std::array<double, 42> jacobian_array_old;
    Eigen::Matrix<double, 6, 7> S1;
    Eigen::Matrix<double, 6, 7> S1_dot;

    // 动态配置参数
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::dynamic_control_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void controlParamCallback(franka_example_controllers::dynamic_control_paramConfig &config, uint32_t level);

    // 参数更新函数
    void controllerParamRenew();

    // debug，绘图
    int time = 0;
    std::ofstream myfile;
    ros::Publisher paramForDebug;
  };

} // namespace franka_example_controllers
