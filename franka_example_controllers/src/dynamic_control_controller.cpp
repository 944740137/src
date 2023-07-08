// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm/pinocchino_interactive.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>

#include <franka_example_controllers/dynamic_control_controller.h>
#include <algorithm/pseudo_inversion.h>

#include <ros/ros.h>
#include <time.h>
extern pinLibInteractive *pinInteractive;
namespace franka_example_controllers
{

  bool JointDynamicControlController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "--------------init1:JointDynamicControlController--------------" << std::endl;
    std::cout << "--------------init2:JointDynamicControlController--------------" << std::endl;
    // 参数服务器
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("JointDynamicControlController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("JointDynamicControlController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("JointDynamicControlController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("JointDynamicControlController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("JointDynamicControlController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("JointDynamicControlController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("JointDynamicControlController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("JointDynamicControlController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // 动态参数服务
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::dynamic_control_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&JointDynamicControlController::controlParamCallback, this, _1, _2));

    // 发布者对象
    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    return true;
  }
  void JointDynamicControlController::starting(const ros::Time & /*time*/)
  {
    std::cout << "--------------start1:JointDynamicControlController--------------" << std::endl;
    std::cout << "--------------start2:JointDynamicControlController--------------" << std::endl;

    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    // 获取当前关节位置
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial_(initial_state.q.data());

    // 控制参数赋初值
    // int tmp = 1;
    // Kp.setIdentity();
    // Kp = tmp * Eigen::MatrixXd::Identity(7, 7);
    // Kv.setIdentity();
    // Kv = tmp * Eigen::MatrixXd::Identity(7, 7);
    // K1.setIdentity();
    // K1 = tmp * Eigen::MatrixXd::Identity(7, 7);
    // K2.setIdentity();
    // K2 = tmp * Eigen::MatrixXd::Identity(7, 7);

    q_d.setZero();
    dq_d.setZero();
    ddq_d.setZero();
    q_initial = q_initial_;
    elapsed_time = ros::Duration(0.0);
    std::cout << "pinLibInteractive start" << std::endl;
    if (pinInteractive == nullptr)
      pinInteractive = new pinLibInteractive();
    std::cout << "pinLibInteractive over" << std::endl;
  }
  void JointDynamicControlController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    // std::cout << "update" << std::endl;

    // 记录数据
    if (firstUpdate)
    {
      // debug
      myfile.open("/home/wd/JointDynamicControlController.txt");
      myfile << "JointDynamicControlController\n"
             << std::endl;
      firstUpdate = false;
    }

    // 期望轨迹生成
    elapsed_time += t;
    double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * elapsed_time.toSec())) * 0.2;
    double dot_delta_angle = M_PI / 8 * M_PI / 5 * (std::sin(M_PI / 5.0 * elapsed_time.toSec())) * 0.2;
    double ddot_delta_angle = M_PI / 8 * M_PI / 5 * M_PI / 5 * (std::cos(M_PI / 5.0 * elapsed_time.toSec())) * 0.2;
    for (size_t i = 0; i < 7; ++i)
    {
      if (i == 4)
      {
        q_d[i] = q_initial[i] - delta_angle;
        dq_d[i] = -dot_delta_angle;
        ddq_d[i] = -ddot_delta_angle;
      }
      else
      {
        q_d[i] = q_initial[i] + delta_angle;
        dq_d[i] = dot_delta_angle;
        ddq_d[i] = ddot_delta_angle;
      }
    }

    // 发布数据
    franka_example_controllers::paramForDebug param_debug;

    // franka 获取状态,C,M，q,dq的array类
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 7> g_array = model_handle_->getGravity();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolisTerm(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> inertiaMatrix1(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> G(g_array.data());

    Eigen::Matrix<double, 9, 1> q_tmp;
    Eigen::Matrix<double, 9, 1> dq_tmp;

    q_tmp << q, 0.02, 0.02;
    dq_tmp << q, 0, 0;
    // pinocchino
    // std::cout << "111111111111" << std::endl;

    pinocchio::Data data = pinInteractive->getpData();
    pinocchio::Model model = pinInteractive->getpModel();

    pinocchio::forwardKinematics(model, data, q_tmp);
    pinocchio::updateFramePlacements(model, data);
    // std::cout << "2222222222" << std::endl;

    pinocchio::computeJointJacobians(model, data, q_tmp);
    Eigen::MatrixXd J_pin1 = data.J;
    pinocchio::computeJointJacobiansTimeVariation(model, data, q_tmp, dq_tmp);
    Eigen::MatrixXd dJ_pin = data.dJ;
    // std::cout << "333333" << std::endl;

    Eigen::MatrixXd J_pin2 = pinocchio::computeJointJacobians(model, data);

    Eigen::Matrix<double, 6, 7> J_pin3;
    Eigen::Matrix<double, 6, 7> J_pin4;
    // Eigen::Matrix<double, 6, 4> J1, J2, dJ;

    // pinocchio::JointIndex joint_id = (pinocchio::JointIndex)(model.njoints - 2);

    // computeJointJacobian(model, data, q, joint_id, J_pin3);
    // pinocchio::getJointJacobian(model, data, joint_id, pinocchio::LOCAL_WORLD_ALIGNED, J_pin4);

    Eigen::Matrix<double, 9, 1> ddq_d_tmp;
    pinocchio::rnea(model, data, q_tmp, dq_tmp, ddq_d_tmp);
    Eigen::MatrixXd G_pin = pinocchio::computeGeneralizedGravity(model, data, q_tmp);
    // Eigen::MatrixXd G_pin = data.g;
    pinocchio::computeCoriolisMatrix(model, data, q_tmp, dq_tmp);
    Eigen::MatrixXd C_pin = data.C;
    pinocchio::getCoriolisMatrix(model, data);
    Eigen::MatrixXd C_pin_ = data.C;

    pinocchio::crba(model, data, q_tmp);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::MatrixXd M_pin = data.M;

    // 误差计算
    Eigen::Matrix<double, 7, 1> error;
    Eigen::Matrix<double, 7, 1> derror;
    Eigen::Matrix<double, 7, 1> r;
    Eigen::Matrix<double, 7, 1> dr;
    Eigen::Matrix<double, 7, 1> error2;
    error = q_d - q;
    derror = dq_d - dq;
    error2 = derror + K1 * error;
    r = dq_d + K1 * error;
    dr = ddq_d + K1 * derror;

    // 命令加速度与输入力矩
    Eigen::VectorXd qc(7), tau_d(7);
    // 纯PD控制----------------------------------------------------------------
    // tau_d << Kp * error + Kv * derror; /* + G */
    // 计算力矩 + PD-----------------------------------------------------------
    // qc = ddq_d + Kp * error + Kv * derror;
    // tau_d << inertiaMatrix1 * (qc) + coriolisTerm; /* + G */
    // 反步控制----------------------------------------------------------------
    // tau_d << inertiaMatrix2 * dr + coriolisMatrix * dr + K2 * error2 + K1 * derror; /* + G */
    // 小练-------------------------------------------------------------------
    // tau_d << inertiaMatrix1 * (Kp * error + Kv * derror); /* + G */

    // debug
    time++;
    if (time % 10 == 0)
    {
      myfile << "--------------------------------------------------------------" << std::endl;
      myfile << "time: " << time << "_" << std::endl;
      myfile << "q:" << std::endl;
      myfile << q.transpose() << std::endl;
      myfile << "franka:M:" << std::endl;
      myfile << inertiaMatrix1 << std::endl;
      myfile << "pinocchino:M_pin:" << std::endl;
      myfile << M_pin << std::endl;
      myfile << "franka:CTerm:" << std::endl;
      myfile << coriolisTerm << std::endl;
      myfile << "pinocchino:CTerm:" << std::endl;
      myfile << C_pin * dq << std::endl;
      // myfile << "pinocchino:CTerm2:" << std::endl;
      // myfile << C_pin_ * dq << std::endl;
      myfile << "franka:G:" << std::endl;
      myfile << G << std::endl;
      myfile << "pinocchino:G:" << std::endl;
      myfile << G_pin << std::endl;
      myfile << "franka:J:" << std::endl;
      myfile << J << std::endl;
      myfile << "pinocchino:J1:" << std::endl;
      myfile << J_pin1 << std::endl;
      // myfile << "pinocchino:J2:" << std::endl;
      // myfile << J_pin2 << std::endl;
      // myfile << "pinocchino:J3:" << std::endl;
      // myfile << J_pin3 << std::endl;
      // myfile << "pinocchino:J4:" << std::endl;
      // myfile << J_pin4 << std::endl;
      // myfile << "pinocchino:C:" << std::endl;
      // myfile << C_pin << std::endl;
      // myfile << "pinocchino:C2:" << std::endl;
      // myfile << C_pin_ << std::endl;
    }

    // 平滑命令
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i)); // 关节句柄设置力矩命令
    }

    // plot
    for (int i = 0; i < 7; i++)
    {
      param_debug.jointError[i] = error[i];
      param_debug.q_d[i] = q_d[i];
      param_debug.q[i] = q[i];
      param_debug.tau_d[i] = tau_d[i];
    }
    paramForDebug.publish(param_debug);

    // 控制参数更新
    controllerParamRenew();
  }
  Eigen::Matrix<double, 7, 1> JointDynamicControlController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }
  void JointDynamicControlController::controlParamCallback(franka_example_controllers::dynamic_control_paramConfig &config, uint32_t /*level*/)
  {
    std::array<double, 7> Kp_values = {config.Kp1, config.Kp2, config.Kp3, config.Kp4, config.Kp5, config.Kp6, config.Kp7};
    std::array<double, 7> Kv_values = {config.Kv1, config.Kv2, config.Kv3, config.Kv4, config.Kv5, config.Kv6, config.Kv7};

    std::array<double, 7> K1_values = {config.K1_1, config.K1_2, config.K1_3, config.K1_4, config.K1_5, config.K1_6, config.K1_7};
    std::array<double, 7> K2_values = {config.K2_1, config.K2_2, config.K2_3, config.K2_4, config.K2_5, config.K2_6, config.K2_7};
    if (time == 0)
    {
      for (int i = 0; i < 7; i++)
      {
        K1(i, i) = K1_values[i];
        K2(i, i) = K2_values[i];
      }
    }
    for (int i = 0; i < 7; i++)
    {
      K1_target(i, i) = K1_values[i];
      K2_target(i, i) = K2_values[i];
    }

    if (time == 0)
    {
      for (int i = 0; i < 7; i++)
      {
        Kp(i, i) = Kp_values[i];
        Kv(i, i) = Kv_values[i];
      }
    }
    for (int i = 0; i < 7; i++)
    {
      Kp_target(i, i) = Kp_values[i];
      Kv_target(i, i) = Kv_values[i];
    }
  }
  void JointDynamicControlController::controllerParamRenew()
  {
    Kp = filter_params * Kp_target + (1.0 - filter_params) * Kp;
    Kv = filter_params * Kv_target + (1.0 - filter_params) * Kv;
  }

  bool CartesianDynamicControlController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "--------------init1:CartesianDynamicControlController_3.23--------------" << std::endl;
    std::cout << "--------------init2:CartesianDynamicControlController_3.23--------------" << std::endl;
    // 发布数据，绘制图像
    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    // 参数服务器
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("CartesianDynamicControlController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("CartesianDynamicControlController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("CartesianDynamicControlController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("CartesianDynamicControlController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("CartesianDynamicControlController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("CartesianDynamicControlController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("CartesianDynamicControlController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM(
            "CartesianDynamicControlController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::dynamic_control_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&CartesianDynamicControlController::controlParamCallback, this, _1, _2));

    // 初始位置姿态赋初值
    position_d.setZero();

    // 控制参数赋初值
    int tmp = 5;
    Kp_target.setIdentity();
    Kp_target = tmp * Eigen::MatrixXd::Identity(7, 7);
    Kv_target.setIdentity();
    Kv_target = tmp * Eigen::MatrixXd::Identity(7, 7);

    return true;
  }
  void CartesianDynamicControlController::starting(const ros::Time & /*time*/)
  {
    std::cout << "--------------start1:CartesianDynamicControlController_3.23--------------" << std::endl;
    std::cout << "--------------start2:CartesianDynamicControlController_3.23--------------" << std::endl;
    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    // 基坐标系下的雅可比（上一时刻，用于数值微分）
    std::array<double, 42> jacobian_array_old = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array_old.data());
    // 当前笛卡尔位置的齐次变换矩阵
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    // 将当前状态设置为平衡点
    position_initial = initial_transform.translation();
    orientation_initial = Eigen::Quaterniond(initial_transform.rotation());

    elapsed_time = ros::Duration(0.0);
  }
  void CartesianDynamicControlController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    // 发布数据
    franka_example_controllers::paramForDebug param_debug;

    // 获取状态,C,M，q,dq的array类
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // 将array类转成矩阵
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolisTerm(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> inertiaMatrix1(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.rotation());

    // 获取外部库的M，C
    // Eigen::Matrix<double, 7, 7> inertiaMatrix2 = MassMatrix(q);
    // Eigen::Matrix<double, 7, 7> coriolisMatrix = CoriolisMatrix(q, dq);

    // 轨迹生成
    elapsed_time += t;
    double radius = 0.15;
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 10 * elapsed_time.toSec()));
    double delta_x = radius * std::sin(angle);
    double delta_z = radius * (std::cos(angle) - 1);
    position_d[0] = position_initial[0] + delta_x;
    position_d[2] = position_initial[2] + delta_z;

    // 误差计算
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> error_dot;

    // 命令加速度与输入力矩
    Eigen::VectorXd qc(7), tau_d(7);
    qc = Kp * error + Kv * error_dot;
    tau_d << inertiaMatrix1 * (qc) + coriolisTerm;

    // debug
    // time++;
    // myfile << " " << std::endl;
    // myfile << "time: " << time << "_"<< std::endl;
    // myfile << "Kp: " << std::endl;
    // myfile << Kp << std::endl;
    // myfile << "Kv: " << std::endl;
    // myfile << Kv << std::endl;

    // 平滑命令
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i)); // 关节句柄设置力矩命令
    }

    // 目标位置，控制参数更新
    controllerParamRenew();
  }
  Eigen::Matrix<double, 7, 1> CartesianDynamicControlController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }
  void CartesianDynamicControlController::controlParamCallback(franka_example_controllers::dynamic_control_paramConfig &config, uint32_t /*level*/)
  {
    // Kp_target.setIdentity();
    // Kp_target = config.Kp * Eigen::MatrixXd::Identity(7, 7);
    // Kv_target.setIdentity();
    // Kv_target = config.Kv * Eigen::MatrixXd::Identity(7, 7);
  }
  void CartesianDynamicControlController::controllerParamRenew()
  {
    // Kp = filter_params * Kp_target + (1.0 - filter_params) * Kp;
    // Kv = filter_params * Kv_target + (1.0 - filter_params) * Kv;
  }

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointDynamicControlController,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianDynamicControlController,
                       controller_interface::ControllerBase)