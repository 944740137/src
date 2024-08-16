// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <fstream>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_example_controllers/admittance_controller_paramConfig.h>
#include <franka_example_controllers/paramForDebug.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <franka_example_controllers/hps_sensor.h>
namespace franka_example_controllers
{

  class AdmittanceController
      : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                              franka_hw::FrankaModelInterface,
                                                              hardware_interface::EffortJointInterface,
                                                              franka_hw::FrankaStateInterface>
  {
  public:
    AdmittanceController();
    ~AdmittanceController();
    bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    // Saturation
    static constexpr double kDeltaTauMax{1.0};
    std::array<double, 7> saturateTorqueRate(const std::array<double, 7> &tau_d_calculated,const std::array<double, 7> &tau_J_d); // NOLINT (readability-identifier-naming)

    franka_hw::FrankaPoseCartesianInterface *cartesian_pose_interface_;
    std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_; // 机器人全部状态
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_; // 机器人的动力学和运动学模型
    std::vector<hardware_interface::JointHandle> joint_handles_; // 关节状态类
    int time = 0;
    std::ofstream myfile;

    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::admittance_controller_paramConfig>> dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(franka_example_controllers::admittance_controller_paramConfig &config, uint32_t level);
    void controllerParamRenew();
    void recordData();

    ros::Duration elapsed_time_;
    std::array<double, 16> initial_pose_{};
    
    Eigen::Matrix<double, 6, 7> J = Eigen::MatrixXd::Zero(6, 7);
    Eigen::Matrix<double, 7, 1> dq = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 7, 1> dq_d = Eigen::MatrixXd::Zero(7, 1);

    double filter_params{0.0016}; // 滤波参数，调整目标位置与阻抗变化速率
    Eigen::Matrix<double, 3, 3> Md = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Dd = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Kd = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Md_d = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Dd_d = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 3> Kd_d = Eigen::MatrixXd::Identity(3, 3);

    // 发布和记录数据
    ros::Publisher paramForDebug;
    franka_example_controllers::paramForDebug param_debug;

    hps_sensor *hps;
    bool isConnect = false;
    double ft[6] = {0};
    double ft_filter[6] = {0};
    double k_gains_[7] = {600.0,600.0,600.0,600.0,250.0,150.0,50.0};
    double d_gains_[7] = {6.0,6.0,6.0,6.0,2.0,1.0,0.5};
    double filter = 0.1;
    Eigen::Matrix<double, 6, 1> ft_new = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> ft_fil = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 3, 1> x_d = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 3, 1> dx_d = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 3, 1> ddx_d = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 3, 1> x_a = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 3, 1> dx_a = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Matrix<double, 3, 1> ddx_a = Eigen::MatrixXd::Zero(3, 1);

    Eigen::Matrix<double, 6, 6> getAdT(Eigen::Matrix<double, 4, 4> &T);
    Eigen::Matrix<double, 6, 6> AdT = Eigen::MatrixXd::Identity(6, 6);
  };

} // namespace franka_example_controllers
