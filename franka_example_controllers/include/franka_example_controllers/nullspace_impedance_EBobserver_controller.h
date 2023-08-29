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

#include <franka_example_controllers/nullspace_impedance_controller_paramConfig.h>
#include <franka_example_controllers/paramForDebug.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <fstream>
#include <iostream>

#include "franka_example_controllers/trajectory.h"

namespace franka_example_controllers
{

  class NullSpaceImpedanceEBObserverController : public controller_interface::MultiInterfaceController<
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

    // 记录和时刻
    // bool firstUpdate = true; // 用于判断是不是第一个控制周期，计算雅可比导数。
    double time = 0;
    std::ofstream myfile;
    std::ofstream ddxc1file;
    std::ofstream ddxc2file;
    std::ofstream dvcfile;
    std::ofstream ddqc1file;
    std::ofstream ddqc2file;

    // 动态配置参数
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_controller_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(franka_example_controllers::nullspace_impedance_controller_paramConfig &config, uint32_t level);
    void controllerParamRenew();

    // 发布和记录数据
    ros::Publisher paramForDebug;
    franka_example_controllers::paramForDebug param_debug;
    void recordData();

    // 初始值
    Eigen::Matrix<double, 7, 1> q0;
    Eigen::Matrix<double, 6, 1> X0;
    Eigen::Affine3d T0;

    // 获取传感器数据
    void upDateParam();
    franka::RobotState robot_state;
    Eigen::Matrix<double, 7, 1> q;
    Eigen::Matrix<double, 7, 1> dq;
    Eigen::Matrix<double, 7, 1> tau_J_d;
    Eigen::Matrix<double, 7, 1> tau_d;
    Eigen::Affine3d T;
    Eigen::Matrix<double, 6, 1> X;
    Eigen::Matrix<double, 6, 1> dX;

    // 获取动力学/运动学数据
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> c;
    Eigen::Matrix<double, 7, 1> G;
    Eigen::Matrix<double, 6, 7> J;

    Eigen::Matrix<double, 7, 7> M_pin;
    Eigen::Matrix<double, 7, 7> C_pin;
    Eigen::Matrix<double, 7, 1> G_pin;
    Eigen::Matrix<double, 6, 7> J_pin;
    // 计算雅克比
    Eigen::Matrix<double, 6, 7> dJ; // 未滤波
    Eigen::Matrix<double, 6, 7> J_old;
    Eigen::Matrix<double, 6, 7> S1;
    Eigen::Matrix<double, 6, 7> S1_dot;

    /********************************************控制器********************************************/
    Eigen::Matrix<double, 3, 7> J1; // 论文里的J
    Eigen::Matrix<double, 3, 7> dJ1;
    Eigen::Matrix<double, 7, 3> J1_pinv;

    Eigen::Matrix<double, 7, 4> Z;
    Eigen::Matrix<double, 4, 7> Z_inv; // v = Z_inv * q
    Eigen::Matrix<double, 4, 7> dZ_inv;
    Eigen::Matrix<double, 4, 7> S2;
    Eigen::Matrix<double, 4, 7> S2_dot;
    
    Eigen::Matrix<double, 3, 1> dx;
    Eigen::Matrix<double, 4, 1> v;

    Eigen::Matrix<double, 3, 1> s;

    Eigen::Matrix<double, 3, 3> Lambdax_inv;
    Eigen::Matrix<double, 4, 4> Lambdav;
    Eigen::Matrix<double, 3, 3> ux;
    Eigen::Matrix<double, 4, 4> uv;

    Eigen::Matrix<double, 7, 1> tau_msr;
    Eigen::Matrix<double, 7, 1> dtau_msr;

    Eigen::Matrix<double, 3, 1> ddxc;
    Eigen::Matrix<double, 4, 1> dvc;
    Eigen::Matrix<double, 7, 1> ddqc;

    Eigen::Matrix<double, 7, 1> tau_task;
    Eigen::Matrix<double, 7, 1> tau_null;

    // 主任务
    Eigen::Matrix<double, 3, 3> P;
    Eigen::Matrix<double, 3, 3> K;
    Eigen::Matrix<double, 3, 3> P_d;
    Eigen::Matrix<double, 3, 3> K_d;

    // 零空间任务
    Eigen::Matrix<double, 4, 4> Bv;
    Eigen::Matrix<double, 7, 7> Kd;
    Eigen::Matrix<double, 4, 4> Bv_d;
    Eigen::Matrix<double, 7, 7> Kd_d;

    Eigen::Matrix<double, 7, 1> task2_q_d;
    Eigen::Matrix<double, 7, 1> task2_dq_d;
    Eigen::Matrix<double, 7, 1> task2_ddq_d;

    // 观测器
    Eigen::Matrix<double, 7, 7> Gamma_inv;
    Eigen::Matrix<double, 7, 7> Gamma_inv_d;
  };

} // namespace franka_example_controllers
