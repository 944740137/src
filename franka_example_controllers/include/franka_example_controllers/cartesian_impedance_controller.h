// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

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

#include <franka_example_controllers/impedance_controller_paramConfig.h>
#include <franka_example_controllers/paramForDebug.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include "franka_example_controllers/trajectory.h"

namespace franka_example_controllers
{

  class CartesianImpedanceController : public controller_interface::MultiInterfaceController<
                                       franka_hw::FrankaModelInterface,
                                       hardware_interface::EffortJointInterface,
                                       franka_hw::FrankaStateInterface>
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    /********************************************franka********************************************/
    // 命令力矩平滑和滤波
    const double delta_tau_max{1.0}; // 最大力矩变化值
    double filter_params{0.001};     // 滤波参数，调整目标位置与阻抗变化速率
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d);
    // 硬件交互
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; // 机器人全部状态
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; // 机器人的动力学和运动学模型
    std::vector<hardware_interface::JointHandle> joint_handles_; // 关节状态类
    double comRatio = 0;

    // 记录和时刻
    // bool firstUpdate = true; // 用于判断是不是第一个控制周期，计算雅可比导数。
    double time = 0;
    std::ofstream myfile;

    // 动态配置参数
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::impedance_controller_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(franka_example_controllers::impedance_controller_paramConfig &config, uint32_t level);
    void controllerParamRenew();

    // 发布和记录数据
    ros::Publisher paramForDebug;
    franka_example_controllers::paramForDebug param_debug;
    void recordData();

    // 初始值
    Eigen::Matrix<double, 7, 1> q0 = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 1> X0 = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Affine3d T0;

    // 获取传感器数据
    void upDateParam();
    franka::RobotState robot_state;
    Eigen::Matrix<double, 7, 1> q = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 1> dq = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 1> tau_J_d = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 1> tau_J = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 1> tau_d = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Affine3d T;
    Eigen::Matrix<double, 6, 1> X = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> dX = Eigen::MatrixXd::Zero(6, 1);

    // 获取动力学/运动学数据
    Eigen::Matrix<double, 7, 7> M = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 1> c = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 1> G = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 7> J = Eigen::MatrixXd::Zero(6, 7);

    Eigen::Matrix<double, 7, 7> M_pin = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 7> C_pin = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 1> G_pin = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 7> J_pin = Eigen::MatrixXd::Zero(6, 7);

    // 计算雅克比
    Eigen::Matrix<double, 6, 7> dJ = Eigen::MatrixXd::Zero(6, 7);     // 未滤波
    Eigen::Matrix<double, 6, 7> dJ_pin = Eigen::MatrixXd::Zero(6, 7); // 未滤波
    Eigen::Matrix<double, 6, 7> J_old = Eigen::MatrixXd::Zero(6, 7);
    Eigen::Matrix<double, 6, 7> S1 = Eigen::MatrixXd::Zero(6, 7);
    Eigen::Matrix<double, 6, 7> S1_dot = Eigen::MatrixXd::Zero(6, 7);

    // franka估计
    Eigen::Matrix<double, 7, 1> tau_ext = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 1> F_ext0 = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> F_extK = Eigen::MatrixXd::Zero(6, 1);

    // 主任务
    double r = 0.1;
    double r_d = 0.1;
    Eigen::Matrix<double, 7, 1> qc = Eigen::MatrixXd::Identity(7, 1);

    Eigen::Matrix<double, 3, 1> xc1 = Eigen::MatrixXd::Identity(3, 1);
    Eigen::Matrix<double, 3, 3> Md = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Dd = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Kd = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Matrix<double, 3, 3> Md_d = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Dd_d = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Kd_d = Eigen::MatrixXd::Identity(3, 3);

    // 零空间任务
    Eigen::Matrix<double, 7, 1> task2_q_d = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 7> task2_K = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 7> task2_D = Eigen::MatrixXd::Identity(7, 7);
  };

} // namespace franka_example_controllers
