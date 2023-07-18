// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/nullspace_impedance_MBobserver_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

#include <fstream>
#include <iostream>

namespace franka_example_controllers {

bool NullSpaceImpedanceMBObserverController::init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& node_handle) 
{
  paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug",20);

  //参数服务器
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "NullSpaceImpedanceMBObserverController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  //运动学/动力学模型类：实例化
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "NullSpaceImpedanceMBObserverController: Exception getting model handle from interface: "<< ex.what());
    return false;
  }

  //机器人完整状态类：实例化
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Exception getting state handle from interface: "<< ex.what());
    return false;
  }
  
  //关节控制类（ROS自带）：实例化
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
  dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_MBobserver_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(boost::bind(&NullSpaceImpedanceMBObserverController::complianceParamCallback, this, _1, _2));

  //初始位置姿态赋初值
  position_d.setZero();
  position_d_target.setZero();

  // 控制参数赋初值
  KI << Ki * Eigen::MatrixXd::Identity(7, 7);
  Kp.topLeftCorner(3, 3) << Kp_pos * Eigen::Matrix3d::Identity();
  Kp.bottomRightCorner(3, 3) << Kp_ori * Eigen::Matrix3d::Identity();
  Kv.topLeftCorner(3, 3) << Kv_pos * Eigen::Matrix3d::Identity();
  Kv.bottomRightCorner(3, 3) << Kv_ori * Eigen::Matrix3d::Identity();

  Md.setIdentity();
  Bd.setIdentity();
  Kd.setIdentity();
  return true;
}

// debug
int time3 = 0;
std::ofstream myfile3;

void NullSpaceImpedanceMBObserverController::starting(const ros::Time& /*time*/) 
{
  std::cout << "--------------start:NullSpaceImpedanceMBObserverController_3.23--------------"<< std::endl;
  std::cout << "--------------start:NullSpaceImpedanceMBObserverController_3.23--------------" << std::endl;
  // 获取机器人初始状态
  franka::RobotState initial_state = state_handle_->getRobotState();
  // 基坐标系下的雅可比（上一时刻，用于数值微分）
  std::array<double, 42> jacobian_array_old = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array_old.data());
  // 获取当前关节位置
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  // 当前笛卡尔位置的齐次变换矩阵
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // 将当前状态设置为平衡点
  position_d = initial_transform.translation();
  orientation_d = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target = initial_transform.translation();
  orientation_d_target = Eigen::Quaterniond(initial_transform.rotation());

  // 零空间期望位置设为当其位置
  q_d = q_initial;
  dq_d.setZero();
  ddq_d.setZero();
}

void NullSpaceImpedanceMBObserverController::update(const ros::Time& /*time*/,const ros::Duration& t) 
{
  //画图
  franka_example_controllers::paramForDebug param_debug;

  // 获取状态,C,M，q,dq
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // 将array类转成矩阵
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));  // 齐次变换矩阵
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.rotation());

  // 计算雅可比
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());//完整雅可比
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_array_old.data());  
  Eigen::Matrix<double, 6, 7> jacobian_dot;
  double r1 = 0.01;
  if (firstUpdate)
  {
    // debug
    myfile3.open("/home/wd/NullSpaceImpedanceMBObserverController.txt");
    myfile3 << "NullSpaceImpedanceMBObserverController.15——109\n" << std::endl;

    S1 = jacobian;   //0时刻
    jacobian_dot.setZero();
    S1_dot.setZero();
  }
  else
  {
    /* jacobian_dot= */ S1_dot = (jacobian - S1) / r1;
    S1 = S1_dot * t.toSec() + S1;
  }
  jacobian_array_old = jacobian_array;

  // 误差计算
  Eigen::Matrix<double, 6, 1> s;
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> error_dot;
  error.head(3) << position_d - position;  // 提取前三个元素
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << transform.rotation() * error.tail(3);  // 提取后三个元素
  error_dot = -jacobian * dq;                             // 0 - jacobian * dq

  // 伪逆矩阵
  Eigen::MatrixXd jacobian_pinv;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
  weightedPseudoInverse(jacobian, jacobian_pinv, mass);

  //基于动量的观测器
  Eigen::Matrix<double, 7, 1> r;//残差向量
  Eigen::Matrix<double, 7, 1> p;//广义动量
  Eigen::Matrix<double, 7, 1> H;//积分式子
  Eigen::Matrix<double, 7, 1> H_dot;//积分式子

  if (firstUpdate)
  {
    r.setZero();
    p.setZero();
    H.setZero();
    H_dot.setZero();
    firstUpdate = false;
  }
  else
  {
  }
  // 命令加速度与输入力矩
  Eigen::VectorXd qc1(7), qc2(7), tau_d(7), xc(6);
  xc << /*ddq + */ Kp * error + Kv * error_dot /* - jacobian * mass.inverse() * r */;
  qc1 << jacobian_pinv * (xc - S1_dot * dq);
  Eigen::MatrixXd N = I - jacobian_pinv * jacobian;
  qc2 << N * (/* task2_ddq_d = 0 */ mass.inverse() *(Kd * (q_d - q) + Bd * (dq_d - dq)));
  tau_d << mass * (qc1 + qc2) + coriolis;

  //上一个时刻的数据

  // debug
  time3++;
  myfile3 << " " << std::endl;
  myfile3 << "time3: " << time3 << "_"<< std::endl;
  myfile3 << "Kp: " << std::endl;
  myfile3 << Kp << std::endl;
  myfile3 << "Kv: " << std::endl;
  myfile3 << Kv << std::endl;
  myfile3 << "R: " << std::endl;
  myfile3 << transform.rotation() << std::endl;
  myfile3 << "Kd: " << std::endl;
  myfile3 << Kd << std::endl;
  myfile3 << "Bd: " << std::endl;
  myfile3 << Bd << std::endl;
  myfile3 << "Md: " << std::endl;
  myfile3 << Md << std::endl;
  myfile3 << "" << std::endl;
  myfile3 << "q: " << q.transpose() << std::endl;
  myfile3 << "dq: " << dq.transpose() << std::endl;
  myfile3 << "q_d: " << q_d.transpose() << std::endl;
  myfile3 << "dq_d: " << dq_d.transpose() << std::endl;
  myfile3 << "position_d: " << position_d.transpose() << std::endl;
  myfile3 << "position: " << position.transpose() << std::endl;
  myfile3 << "error: " << error.transpose() << std::endl;
  myfile3 << "error_dot: " << error_dot.transpose() << std::endl;
  myfile3 << "jacobian:" << std::endl;
  myfile3 << jacobian << std::endl;
  myfile3 << "S1_dot:" << std::endl;
  myfile3 << S1_dot << std::endl;
  Eigen::MatrixXd II = jacobian * jacobian_pinv;
  myfile3 << "jacobian * jacobian_pinv:" << std::endl;
  myfile3 << II << std::endl;
  myfile3 << "qc1: " << qc1.transpose() << std::endl;
  myfile3 << "qc2: " << qc2.transpose() << std::endl;
  myfile3 << "tau_d: " << tau_d.transpose() << std::endl;

  //画图
  for(int i = 0; i < 7; i++)
  {
    // param_debug.qc1[i] = qc1[i];
    // param_debug.qc2[i] = qc2[i];
    param_debug.tau_d[i] = tau_d[i];
  }
  paramForDebug.publish(param_debug);

  // 平滑命令
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));  // 关节句柄设置力矩命令
  }

  // 目标位置，控制参数更新
  controllerParamRenew();
}

Eigen::Matrix<double, 7, 1> NullSpaceImpedanceMBObserverController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,const Eigen::Matrix<double, 7, 1>& tau_J_d) 
{  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), - delta_tau_max);//6
  }
  return tau_d_saturated;
}

void NullSpaceImpedanceMBObserverController::complianceParamCallback(franka_example_controllers::nullspace_impedance_MBobserver_controller_paramConfig& config,uint32_t /*level*/) 
{
  Kp_target.setIdentity();
  Kp_target.topLeftCorner(3, 3) << config.Kp_pos * Eigen::Matrix3d::Identity();
  Kp_target.bottomRightCorner(3, 3) << config.Kp_ori * Eigen::Matrix3d::Identity();

  Kv_target.setIdentity();
  Kv_target.topLeftCorner(3, 3) << config.Kv_pos * Eigen::Matrix3d::Identity();
  Kv_target.bottomRightCorner(3, 3) << config.Kv_ori * Eigen::Matrix3d::Identity();

  Kd = config.Kd * Eigen::MatrixXd::Identity(7, 7);
  Bd = config.Bd * Eigen::MatrixXd::Identity(7, 7);
  Md = config.Md * Eigen::MatrixXd::Identity(7, 7);
}

void NullSpaceImpedanceMBObserverController::controllerParamRenew() 
{
  Kp = filter_params * Kp_target + (1.0 - filter_params) * Kp;
  Kv = filter_params * Kv_target + (1.0 - filter_params) * Kv;

  Kd = filter_params * Kd + (1.0 - filter_params) * Kd;
  Bd = filter_params * Bd + (1.0 - filter_params) * Bd;
  Md = filter_params * Md + (1.0 - filter_params) * Md;
}

//X-Y-Z固定角
Eigen::Vector3d NullSpaceImpedanceMBObserverController::toEulerAngle(Eigen::Matrix3d R) 
{
  Eigen::Vector3d orientation;
  
  //Y betha
  orientation(1) = atan2(-R(2, 0), std::fabs(std::sqrt(std::pow(R(0, 0), 2) + std::pow(R(1, 0), 2))));

  //Z alpha
  orientation(2) = atan2(R(1, 0) / cos(orientation(1)), R(0, 0) / cos(orientation(1)));

  //X r
  orientation(0) = atan2(R(2, 1) / cos(orientation(1)), R(2, 2) / cos(orientation(1)));

  return orientation;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::NullSpaceImpedanceMBObserverController,
                       controller_interface::ControllerBase)
