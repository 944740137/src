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

#include <franka_example_controllers/nullspace_impedance_paramConfig.h>
#include <franka_example_controllers/paramForDebug.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers
{

  class NullSpaceImpedanceController : public controller_interface::MultiInterfaceController<
                                           franka_hw::FrankaModelInterface,
                                           hardware_interface::EffortJointInterface,
                                           franka_hw::FrankaStateInterface>
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    // 平滑
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; // 机器人全部状态
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; // 机器人的动力学和运动学模型
    std::vector<hardware_interface::JointHandle> joint_handles_; // 关节状态类

    double filter_params{0.005};     // 滤波参数，调整目标位置与阻抗变化速率
    const double delta_tau_max{1.0}; // 最大力矩变化值
    bool firstUpdate = true;         // 用于判断是不是第一个控制周期，计算雅可比导数。

    // 主任务控制参数
    Eigen::Matrix<double, 3, 3> task1_Kp;
    Eigen::Matrix<double, 3, 3> task1_Kv;
    Eigen::Matrix<double, 3, 3> task1_Kp_target;
    Eigen::Matrix<double, 3, 3> task1_Kv_target;

    // 主任务目标位置（无姿态）
    Eigen::Vector3d position_d;

    // 零空间阻抗参数
    Eigen::Matrix<double, 7, 7> task2_Md;
    Eigen::Matrix<double, 7, 7> task2_Bd;
    Eigen::Matrix<double, 7, 7> task2_Kd;
    Eigen::Matrix<double, 7, 1> task2_q_d;
    Eigen::Matrix<double, 7, 1> task2_dq_d;
    Eigen::Matrix<double, 7, 1> task2_ddq_d;

    // 用于数值微分
    std::array<double, 42> jacobian_array_old;
    Eigen::Matrix<double, 3, 1> error_old;

    // 动态配置参数
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(franka_example_controllers::nullspace_impedance_paramConfig &config, uint32_t level);

    // 参数更新函数
    void controllerParamRenew();

    // debug，绘图
    int time = 0;
    std::ofstream myfile;
    ros::Publisher paramForDebug;
  };

  class NullSpaceImpedanceEBObserverController
      : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface>
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    // 平滑
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; // 机器人全部状态
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; // 机器人的动力学和运动学模型
    std::vector<hardware_interface::JointHandle> joint_handles_; // 关节状态类

    double filter_params{0.05};      // 滤波参数，调整目标位置与阻抗变化速率
    const double delta_tau_max{1.0}; // 最大力矩变化值
    bool firstUpdate = true;         // 用于判断是不是第一个控制周期，计算雅可比导数。

    // 主任务
    int K_pos = 10;
    int K_ori = 6;
    int P_pos = 1;
    int P_ori = 1;
    Eigen::Matrix<double, 6, 6> K;
    Eigen::Matrix<double, 6, 6> P;
    Eigen::Matrix<double, 7, 7> gamma; // Γf
    Eigen::Matrix<double, 6, 6> K_target;
    Eigen::Matrix<double, 6, 6> P_target;
    Eigen::Matrix<double, 7, 7> gamma_target; // Γf
    Eigen::Vector3d position_d;               // xd
    Eigen::Vector3d d_position_d;
    Eigen::Vector3d dd_position_d;    // ddxd
    Eigen::Quaterniond orientation_d; // xd
    Eigen::Quaterniond d_orientation_d;
    Eigen::Quaterniond dd_orientation_d; // ddxd
    Eigen::Vector3d position_d_target;
    Eigen::Vector3d position_d_target1;
    Eigen::Vector3d position_d_target2;
    Eigen::Quaterniond orientation_d_target;
    Eigen::Matrix<double, 7, 1> tau_estimated;

    // 零空间任务
    Eigen::Matrix<double, 7, 7> Kd;
    Eigen::Matrix<double, 7, 7> Md;
    Eigen::Matrix<double, 7, 7> Bd;
    Eigen::Matrix<double, 7, 7> Kd_target;
    Eigen::Matrix<double, 7, 7> Md_target;
    Eigen::Matrix<double, 7, 7> Bd_target;
    Eigen::Matrix<double, 7, 1> q_d;
    Eigen::Matrix<double, 7, 1> dq_d;
    Eigen::Matrix<double, 7, 1> ddq_d;

    // 用于数值微分 滤波
    std::array<double, 42> jacobian_array_old;
    Eigen::Matrix<double, 6, 6> Lambda_old;
    Eigen::Matrix<double, 6, 7> S1;
    Eigen::Matrix<double, 6, 6> S2;
    Eigen::Matrix<double, 6, 7> S1_dot;
    Eigen::Matrix<double, 6, 6> S2_dot;

    // 用于轨迹生成
    ros::Duration elapsed_time;

    // 动态配置参数
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(franka_example_controllers::nullspace_impedance_paramConfig &config, uint32_t level);

    // 参数更新函数
    void controllerParamRenew();

    // debug，绘图
    int time = 0;
    std::ofstream myfile;
    ros::Publisher paramForDebug;
  };

  class NullSpaceImpedanceMBObserverController
      : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface>
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    //
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; // 机器人全部状态
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; // 机器人的动力学和运动学模型
    std::vector<hardware_interface::JointHandle> joint_handles_; // 关节状态类

    double filter_params{0.005};     // 滤波参数，调整目标位置与阻抗变化速率
    const double delta_tau_max{1.0}; // 最大力矩变化值
    bool firstUpdate = true;         // 用于判断是不是第一个控制周期，计算雅可比导数。

    // 主任务
    Eigen::Matrix<double, 7, 7> KI; // 观测器
    Eigen::Matrix<double, 6, 6> Kp;
    Eigen::Matrix<double, 6, 6> Kv;
    Eigen::Matrix<double, 6, 6> KI_target;
    Eigen::Matrix<double, 6, 6> Kp_target;
    Eigen::Matrix<double, 6, 6> Kv_target;
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;
    Eigen::Vector3d position_d_target;
    Eigen::Quaterniond orientation_d_target;
    Eigen::Matrix<double, 7, 1> tau_last;

    // 零空间任务
    Eigen::Matrix<double, 7, 7> Md;
    Eigen::Matrix<double, 7, 7> Bd;
    Eigen::Matrix<double, 7, 7> Kd;
    Eigen::Matrix<double, 7, 1> q_d;
    Eigen::Matrix<double, 7, 1> dq_d;
    Eigen::Matrix<double, 7, 1> ddq_d;

    // 用于数值微分
    std::array<double, 42> jacobian_array_old;
    Eigen::Matrix<double, 6, 7> S1;
    Eigen::Matrix<double, 6, 7> S1_dot;

    // 动态配置参数
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(franka_example_controllers::nullspace_impedance_paramConfig &config, uint32_t level);

    // 参数更新函数
    void controllerParamRenew();

    //
    int time = 0;
    std::ofstream myfile;
    ros::Publisher paramForDebug;
  };
} // namespace franka_example_controllers
