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

#include "franka_example_controllers/trajectory.h"

namespace franka_example_controllers
{

  class NullSpaceImpedanceMBObserverController : public controller_interface::MultiInterfaceController<
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
    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_controller_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(franka_example_controllers::nullspace_impedance_controller_paramConfig &config, uint32_t level);
    void controllerParamRenew();
    void sixDOF(Eigen::Matrix<double, 6, 1> &X_d,
                Eigen::Matrix<double, 6, 1> &dX_d,
                Eigen::Matrix<double, 6, 1> &ddX_d,
                Eigen::Matrix<double, 6, 1> &Xerror,
                Eigen::Matrix<double, 6, 1> &dXerror);
    void threeDOF(Eigen::Matrix<double, 6, 1> &X_d,
                  Eigen::Matrix<double, 6, 1> &dX_d,
                  Eigen::Matrix<double, 6, 1> &ddX_d,
                  Eigen::Matrix<double, 6, 1> &Xerror,
                  Eigen::Matrix<double, 6, 1> &dXerror,
                  const ros::Duration &t);

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
    Eigen::Matrix<double, 7, 1> S3 = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 1> S3_dot = Eigen::MatrixXd::Zero(7, 1);
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

    Eigen::Matrix<double, 6, 7> J_pin = Eigen::MatrixXd::Zero(6, 7);
    Eigen::Matrix<double, 7, 7> M_pin = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 7> C_pin = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 1> G_pin = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 7> J_pin1 = Eigen::MatrixXd::Zero(6, 7);

    // 计算雅克比
    Eigen::Matrix<double, 6, 7> dJ = Eigen::MatrixXd::Zero(6, 7); // 未滤波
    Eigen::Matrix<double, 6, 7> dJ_pin = Eigen::MatrixXd::Zero(6, 7); // 未滤波
    Eigen::Matrix<double, 6, 7> J_old = Eigen::MatrixXd::Zero(6, 7);
    Eigen::Matrix<double, 6, 7> S1 = Eigen::MatrixXd::Zero(6, 7);
    Eigen::Matrix<double, 6, 7> S1_dot = Eigen::MatrixXd::Zero(6, 7);

    // franka估计
    Eigen::Matrix<double, 7, 1> tau_ext = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 1> F_ext0 = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> F_extK = Eigen::MatrixXd::Zero(6, 1);

    /********************************************3自由度控制器********************************************/
    Eigen::Matrix<double, 3, 3> Jm = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix<double, 3, 1> Ja = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 3, 3> Jb = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix<double, 1, 4> Za = Eigen::MatrixXd::Zero(1, 4);
    Eigen::Matrix<double, 3, 4> Zb = Eigen::MatrixXd::Zero(3, 4);
    Eigen::Matrix<double, 3, 4> Zm = Eigen::MatrixXd::Zero(3, 4);

    Eigen::Matrix<double, 3, 7> J1 = Eigen::MatrixXd::Zero(3, 7); // 论文里的J
    Eigen::Matrix<double, 3, 7> dJ1 = Eigen::MatrixXd::Zero(3, 7);
    Eigen::Matrix<double, 7, 3> J1_pinv = Eigen::MatrixXd::Zero(7, 3);

    Eigen::Matrix<double, 7, 4> Z = Eigen::MatrixXd::Zero(7, 4);
    Eigen::Matrix<double, 4, 7> Z_inv = Eigen::MatrixXd::Zero(4, 7); // v = Z_inv * q
    Eigen::Matrix<double, 4, 7> dZ_inv = Eigen::MatrixXd::Zero(4, 7);
    Eigen::Matrix<double, 4, 7> S2 = Eigen::MatrixXd::Zero(4, 7);
    Eigen::Matrix<double, 4, 7> S2_dot = Eigen::MatrixXd::Zero(4, 7);

    Eigen::Matrix<double, 3, 1> dx = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 4, 1> v = Eigen::MatrixXd::Zero(4, 1);

    Eigen::Matrix<double, 3, 3> Lambdax_inv = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix<double, 4, 4> Lambdav = Eigen::MatrixXd::Zero(4, 4);
    Eigen::Matrix<double, 3, 3> ux = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix<double, 4, 4> uv = Eigen::MatrixXd::Zero(4, 4);

    Eigen::Matrix<double, 6, 1> F_msr = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 7, 1> tau_msr = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 1> dtau_msr = Eigen::MatrixXd::Zero(7, 1);

    Eigen::Matrix<double, 3, 1> ddxc = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 4, 1> dvc = Eigen::MatrixXd::Zero(4, 1);
    Eigen::Matrix<double, 7, 1> ddqc = Eigen::MatrixXd::Zero(7, 1);

    // 主任务
    Eigen::Matrix<double, 3, 3> PD_D = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> PD_D_d = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> PD_K = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> PD_K_d = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Matrix<double, 3, 3> Kv = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Kv_d = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Kp = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Kp_d = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 7, 7> KI = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 7> KI_d = Eigen::MatrixXd::Identity(7, 7);

    // 零空间任务
    Eigen::Matrix<double, 4, 4> Bv = Eigen::MatrixXd::Identity(4, 4);
    Eigen::Matrix<double, 7, 7> Kd = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 4, 4> Bv_d = Eigen::MatrixXd::Identity(4, 4);
    Eigen::Matrix<double, 7, 7> Kd_d = Eigen::MatrixXd::Identity(7, 7);

    Eigen::Matrix<double, 7, 1> task2_q_d = Eigen::MatrixXd::Zero(7, 1);

    // 观测器
    Eigen::Matrix<double, 7, 1> r = Eigen::MatrixXd::Zero(7, 1);

    /********************************************6自由度控制器********************************************/
  };

} // namespace franka_example_controllers
