// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/admittance_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// #define TORQUE_CONTROL

namespace franka_example_controllers
{
  AdmittanceController::AdmittanceController()
  {
    // std::cout << "[------------------] AdmittanceController 构造" << std::endl;
  }

  AdmittanceController::~AdmittanceController()
  {
    std::cout << "[------------------] 关闭传感器" << std::endl;
    hps->ftc_close();
  }
  Eigen::Matrix<double, 6, 6> AdmittanceController::getAdT(Eigen::Matrix<double, 4, 4> &Tab)
  {
    Eigen::Matrix<double, 3, 3> Rab = Tab.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> Pab = Tab.block(0, 3, 3, 1);
    Eigen::Matrix<double, 3, 3> S;
    S << 0, -Pab(2), -Pab(1), Pab(2), 0, -Pab(0), -Pab(1), Pab(0), 0;
    this->AdT.block(0, 0, 3, 3) = Rab.transpose();
    this->AdT.block(0, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
    this->AdT.block(3, 0, 3, 3) = -Rab.transpose() * S;
    this->AdT.block(3, 3, 3, 3) = Rab.transpose();
    return this->AdT;
  }
  bool AdmittanceController::init(hardware_interface::RobotHW *robot_hardware,
                                  ros::NodeHandle &node_handle)
  {
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("AdmittanceController: Could not get parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
    if (cartesian_pose_interface_ == nullptr)
    {
      ROS_ERROR(
          "AdmittanceController: Could not get Cartesian Pose "
          "interface from hardware");
      return false;
    }
    try
    {
      cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
          cartesian_pose_interface_->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "AdmittanceController: Exception getting Cartesian handle: " << e.what());
      return false;
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR("AdmittanceController: Could not get state interface from hardware");
      return false;
    }
    try
    {
      auto state_handle = state_interface->getHandle(arm_id + "_robot");
      std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      for (size_t i = 0; i < q_start.size(); i++)
      {
        if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1)
        {
          ROS_ERROR_STREAM(
              "AdmittanceController: Robot is not in the expected starting position for "
              "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
              "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
          return false;
        }
      }
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "AdmittanceController: Exception getting state handle: " << e.what());
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("CartesianImpedanceController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
#ifdef TORQUE_CONTROL
    auto *effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("CartesianImpedanceController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM("CartesianImpedanceController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }
#endif
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::admittance_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&AdmittanceController::complianceParamCallback, this, _1, _2));

    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    double ft[6] = {0};
    this->hps = new hps_sensor();
    this->isConnect = this->hps->connect_sensor(8080, "172.16.0.3");

    return true;
  }

  void AdmittanceController::starting(const ros::Time & /* time */)
  {
    std::cout << "[------------------] start1:AdmittanceController\n";
    std::cout << "[------------------] start2:AdmittanceController\n";
    std::cout << "[------------------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------------------] 编译时刻:" << __TIME__ << "\n";
#ifdef TORQUE_CONTROL
    std::cout << "[------------------] 导纳控制:力矩控制" << std::endl;
#else
    std::cout << "[------------------] 导纳控制:纯位置控制" << std::endl;
#endif
    if (!this->myfile.is_open())
    {
      this->myfile.open("/home/wd/log/franka/master/AdmittanceController.txt");
      this->myfile << "AdmittanceController" << std::endl;
      this->myfile << "编译日期:" << __DATE__ << "\n";
      this->myfile << "编译时刻:" << __TIME__ << "\n";
      this->myfile << "导纳阻抗" << std::endl;
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      this->x_d[0] = initial_pose_[12];
      this->x_d[1] = initial_pose_[13];
      this->x_d[2] = initial_pose_[14];
    }

    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    elapsed_time_ = ros::Duration(0.0);
    this->x_a[0] = initial_pose_[12];
    this->x_a[1] = initial_pose_[13];
    this->x_a[2] = initial_pose_[14];
    this->dx_a = Eigen::MatrixXd::Zero(3, 1);
    this->ddx_a = Eigen::MatrixXd::Zero(3, 1);
  }

  void AdmittanceController::update(const ros::Time & /* time */,
                                    const ros::Duration &period)
  {
    elapsed_time_ += period;
    this->J = Eigen::Map<Eigen::Matrix<double, 6, 7>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());

    franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
    this->dq = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

    std::array<double, 16> new_pose = initial_pose_;

    if (this->isConnect)
    {
      hps->get_ftData(this->ft);
      for (int i = 0; i < 6; i++)
      {
        // if (this->ft[i] > 0.5)
        this->ft_new[i] = this->ft[i]; // 换方向
        //   else
        //     this->ft_new[i] = 0;
      }
      // 变换力矢量的表示到基坐标系下
      this->ft_new[0] = this->ft[0] * std::sqrt(2) / 2 - this->ft[1] * std::sqrt(2) / 2;
      this->ft_new[1] = -(this->ft[0] * std::sqrt(2) / 2 + this->ft[1] * std::sqrt(2) / 2);
      this->ft_new[2] = -this->ft_new[2];
    }
    // double radius = 0.10;
    // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
    // double delta_x = radius * std::sin(angle);
    // double delta_z = radius * (std::cos(angle) - 1);
    // x_d[0] -= delta_x;

    this->ft_fil = filter_params * this->ft_new + (1.0 - filter_params) * this->ft_fil;
    // for (int i = 0; i < 6; i++)
    //   this->ft_fil[i] = 0.0;
    ddx_a = -Md.inverse() * (-this->ft_fil.block(0, 0, 3, 1) - Kd * (x_d - x_a) - Dd * (dx_d - dx_a)) + ddx_d;
    dx_a = dx_a + ddx_a * 0.001;
    x_a = x_a + dx_a * 0.001;

    new_pose[12] = x_a[0];
    new_pose[13] = x_a[1];
    new_pose[14] = x_a[2];
    cartesian_pose_handle_->setCommand(new_pose);

    // 发布
    for (int i = 0; i < 7; i++)
    {
      // this->param_debug.tau_d[i] = this->tau_d[i];
      this->param_debug.dq_d[i] = robot_state.dq_d[i];
      this->param_debug.q_d[i] = robot_state.q_d[i];
    }
    for (int i = 0; i < 6; i++)
    {
      this->param_debug.F_sensor[i] = this->ft_new[i];
      this->param_debug.F_sensor_fil[i] = this->ft_fil[i];
    }
    for (int i = 0; i < 3; i++)
    {
      this->param_debug.pos_d[i] = this->x_d[i];
      this->param_debug.pos[i] = this->x_a[i];
      this->param_debug.dpos[i] = this->dx_a[i];
      this->param_debug.ddpos[i] = this->ddx_a[i];
    }
    this->paramForDebug.publish(this->param_debug);

    // 清零
    for (int i = 0; i < 6; i++)
      this->ft_new[i] = 0.0;

#ifdef TORQUE_CONTROL
    // franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();
    std::array<double, 7> tau_d_calculated;
    for (size_t i = 0; i < 7; ++i)
    {
      tau_d_calculated[i] = coriolis[i] + k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) + d_gains_[i] * (robot_state.dq_d[i] - robot_state.dq[i]);
    }
    std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d_saturated[i]);
    }
#endif
    // 记录数据
    this->time++;
    controllerParamRenew();
    recordData();
  }
  std::array<double, 7> AdmittanceController::saturateTorqueRate(
      const std::array<double, 7> &tau_d_calculated,
      const std::array<double, 7> &tau_J_d)
  { // NOLINT (readability-identifier-naming)
    std::array<double, 7> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
    }
    return tau_d_saturated;
  }
  void AdmittanceController::complianceParamCallback(franka_example_controllers::admittance_controller_paramConfig &config, uint32_t /*level*/)
  {
    if (time == 0)
    {
      this->Kd = config.AC_Kd * Eigen::MatrixXd::Identity(3, 3);
      this->Dd = config.AC_Dd * Eigen::MatrixXd::Identity(3, 3);
      this->Md = config.AC_Md * Eigen::MatrixXd::Identity(3, 3);
    }
    this->Kd_d = config.AC_Kd * Eigen::MatrixXd::Identity(3, 3);
    this->Dd_d = config.AC_Dd * Eigen::MatrixXd::Identity(3, 3);
    this->Md_d = config.AC_Md * Eigen::MatrixXd::Identity(3, 3);
  }

  void AdmittanceController::controllerParamRenew()
  {
    this->Kd = filter_params * this->Kd_d + (1.0 - filter_params) * this->Kd;
    this->Dd = filter_params * this->Dd_d + (1.0 - filter_params) * this->Dd;
    this->Md = filter_params * this->Md_d + (1.0 - filter_params) * this->Md;
  }
  void AdmittanceController::recordData()
  {
    // this->myfile << "time: " << this->time << "_\n";
    // this->myfile << "comRatio: " << this->comRatio << "_\n";
    // this->myfile << "J_pin1: \n";
    // this->myfile << this->J_pin1 << "\n";
    // this->myfile << "J: \n";
    // this->myfile << this->J << "\n";
    // this->myfile << "J_pin: \n";
    // this->myfile << this->J_pin << "\n";
    // this->myfile << "dJ: \n";
    // this->myfile << this->dJ << "\n";
    // this->myfile << "dJ_pin: \n";
    // this->myfile << this->dJ_pin << "\n";
    // this->myfile << "M: \n";
    // this->myfile << this->M << "\n";
    // this->myfile << "M_pin: \n";
    // this->myfile << this->M_pin << "\n";
    // this->myfile << "c: \n";
    // this->myfile << this->c.transpose() << "\n";
    // this->myfile << "c_pin*dq: \n";
    // this->myfile << (this->C_pin * this->dq).transpose() << "\n";
    // this->myfile << "G: \n";
    // this->myfile << this->G.transpose() << "\n";
    // this->myfile << "G_pin: \n";
    // this->myfile << this->G_pin.transpose() << "\n";

    this->myfile << "Kd: \n";
    this->myfile << this->Kd << "\n";
    this->myfile << "Dd: \n";
    this->myfile << this->Dd << "\n";
    this->myfile << "Md: \n";
    this->myfile << this->Md << "\n";
  }
} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::AdmittanceController,
                       controller_interface::ControllerBase)
