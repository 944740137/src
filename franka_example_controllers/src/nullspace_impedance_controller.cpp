// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/nullspace_impedance_controller.h>
#include <algorithm/pseudo_inversion.h>

namespace franka_example_controllers
{

  bool NullSpaceImpedanceController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "--------------init1:NullSpaceImpedanceController_3.10--------------" << std::endl;
    std::cout << "--------------init2:NullSpaceImpedanceController_3.10--------------" << std::endl;
    // 参数服务器
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("NullSpaceImpedanceController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("NullSpaceImpedanceController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // 动态参数服务
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&NullSpaceImpedanceController::complianceParamCallback, this, _1, _2));

    // 发布者对象
    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    return true;
  }
  void NullSpaceImpedanceController::starting(const ros::Time & /*time*/)
  {
    std::cout << "--------------start1:NullSpaceImpedanceController_3.10--------------" << std::endl;
    std::cout << "--------------start2:NullSpaceImpedanceController_3.10--------------" << std::endl;
    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    // 基坐标系下的雅可比array类（上一时刻，用于数值微分）
    std::array<double, 42> jacobian_array_old = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // 获取当前关节位置
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    // 当前笛卡尔位置的齐次变换矩阵
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // 将当前状态设置为平衡点
    position_d = initial_transform.translation();

    // 控制参数赋初值
    task1_Kp.setIdentity();
    task1_Kv.setIdentity();
    task2_Md.setIdentity();
    task2_Bd.setIdentity();
    task2_Kd.setIdentity();

    // 零空间期望位置设为当其位置
    task2_q_d = q_initial;
    task2_dq_d.setZero();
    task2_ddq_d.setZero();
  }
  void NullSpaceImpedanceController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    // 发布数据
    franka_example_controllers::paramForDebug param_debug;

    // 获取状态,C,M，q,dq的array类
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
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    Eigen::Vector3d position(transform.translation());

    // 计算雅可比
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data()); // 完整雅可比
    Eigen::Matrix<double, 3, 7> jacobian_1 = jacobian.block(0, 0, 3, 7);     // 位置雅可比
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_array_old.data());
    Eigen::Matrix<double, 3, 7> jacobian_1_old = jacobian_old.block(0, 0, 3, 7);
    Eigen::Matrix<double, 3, 7> jacobian_1_dot;
    if (firstUpdate)
    {
      // debug
      myfile.open("/home/wd/NullSpaceImpedanceController2.txt");
      myfile << "NullSpaceImpedanceController\n"
             << std::endl;

      jacobian_1_dot.setZero(); // 若在第一次控制周期，将雅可比导数置为0
    }
    else
    {
      jacobian_1_dot = (jacobian_1 - jacobian_1_old) / t.toSec();
    }
    jacobian_array_old = jacobian_array;

    // 误差计算
    Eigen::Matrix<double, 3, 1> error;
    Eigen::Matrix<double, 3, 1> error_dot;
    error << position_d - position;
    if (firstUpdate)
    {
      error_dot.setZero();
      firstUpdate = false;
    }
    else
    {
      error_dot = (error - error_old) / t.toSec();
    }
    error_old = error;

    // 伪逆矩阵
    Eigen::MatrixXd jacobian_1_pinv;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    weightedPseudoInverse(jacobian_1, jacobian_1_pinv, mass);

    // 命令加速度与输入力矩
    Eigen::VectorXd qc1(7), qc2(7), tau_d(7);
    qc1 << jacobian_1_pinv * (/*ddq + */ task1_Kp * error + task1_Kv * error_dot - jacobian_1_dot * dq);
    Eigen::MatrixXd N = I - jacobian_1_pinv * jacobian_1;
    qc2 << N * (/*task2_ddq_d = 0*/ mass.inverse() * (task2_Kd * (task2_q_d - q) + task2_Bd * (task2_dq_d - dq)));
    tau_d << mass * (qc1 + qc2) + coriolis;

    // debug
    // time++;
    // myfile << " " << std::endl;
    // myfile << "time: " << time << "_"<< std::endl;
    // myfile << "mass: " << std::endl;
    // myfile << mass << std::endl;
    // myfile << "task1_Kv: " << std::endl;
    // myfile << task1_Kv << std::endl;
    // myfile << "R: " << std::endl;
    // myfile << transform.rotation() << std::endl;
    // myfile << "" << std::endl;
    // myfile << "q: " << q.transpose() << std::endl;
    // myfile << "dq: " << dq.transpose() << std::endl;
    // myfile << "task2_q_d: " << task2_q_d.transpose() << std::endl;
    // myfile << "task2_dq_d: " << task2_dq_d.transpose() << std::endl;
    // myfile << "task2_ddq_d: " << task2_ddq_d.transpose() << std::endl;
    // myfile << "position_d: " << position_d.transpose() << std::endl;
    // myfile << "position: " << position.transpose() << std::endl;
    // myfile << "orientation_d: " << orientation_d.transpose() << std::endl;
    // myfile << "orientation: " << orientation.transpose() << std::endl;
    // myfile << "orientation error: " << (orientation_d - orientation).transpose() << std::endl;
    // myfile << "error: " << error.transpose() << std::endl;
    // myfile << "error_dot: " << error_dot.transpose() << std::endl;
    // myfile << "jacobian:" << std::endl;
    // myfile << jacobian << std::endl;
    // myfile << "jacobian_1:" << std::endl;
    // myfile << jacobian_1 << std::endl;
    // myfile << "jacobian_1_dot:" << std::endl;
    // myfile << jacobian_1_dot << std::endl;
    // myfile << "jacobian_1_pinv:" << std::endl;
    // myfile << jacobian_1_pinv << std::endl;
    // Eigen::MatrixXd II = jacobian_1 * jacobian_1_pinv;
    // myfile << "jacobian_1 * jacobian_1_pinv:" << std::endl;
    // myfile << II << std::endl;
    // myfile << "qc1: " << qc1.transpose() << std::endl;
    // myfile << "qc2: " << qc2.transpose() << std::endl;
    // myfile << "tau_d: " << tau_d.transpose() << std::endl;

    // 画图
    for (int i = 0; i < 7; i++)
    {
      param_debug.qc1[i] = qc1[i];
      param_debug.qc2[i] = qc2[i];
      param_debug.tau_d[i] = tau_d[i];
    }
    paramForDebug.publish(param_debug);

    // 平滑命令
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i)); // 关节句柄设置力矩命令
    }

    // 控制参数更新
    controllerParamRenew();
  }
  Eigen::Matrix<double, 7, 1> NullSpaceImpedanceController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }
  void NullSpaceImpedanceController::complianceParamCallback(franka_example_controllers::nullspace_impedance_paramConfig &config, uint32_t /*level*/)
  {
    task1_Kp_target.setIdentity();
    task1_Kp_target = config.task1_Kp * Eigen::Matrix3d::Identity();

    task1_Kv_target.setIdentity();
    task1_Kv_target = config.task1_Kv * Eigen::Matrix3d::Identity();

    task2_Kd = config.task2_Kd * Eigen::MatrixXd::Identity(7, 7);
    task2_Bd = config.task2_Bd * Eigen::MatrixXd::Identity(7, 7);
    task2_Md = config.task2_Md * Eigen::MatrixXd::Identity(7, 7);
  }
  void NullSpaceImpedanceController::controllerParamRenew()
  {
    task1_Kp = filter_params * task1_Kp_target + (1.0 - filter_params) * task1_Kp;
    task1_Kv = filter_params * task1_Kp_target + (1.0 - filter_params) * task1_Kv;

    task2_Kd = filter_params * task2_Kd + (1.0 - filter_params) * task2_Kd;
    task2_Bd = filter_params * task2_Bd + (1.0 - filter_params) * task2_Bd;
    task2_Md = filter_params * task2_Md + (1.0 - filter_params) * task2_Md;
  }

  bool NullSpaceImpedanceEBObserverController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "--------------init1:NullSpaceImpedanceEBObserverController_3.21_0.01--------------" << std::endl;
    std::cout << "--------------init2:NullSpaceImpedanceEBObserverController_3.21_0.01--------------" << std::endl;
    // 参数服务器
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceEBObserverController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("NullSpaceImpedanceEBObserverController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceEBObserverController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceEBObserverController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceEBObserverController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceEBObserverController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceEBObserverController: Error getting effort joint interface from "
                       "hardware");
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
        ROS_ERROR_STREAM("NullSpaceImpedanceEBObserverController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // 动态参数
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&NullSpaceImpedanceEBObserverController::complianceParamCallback, this, _1, _2));

    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    return true;
  }
  void NullSpaceImpedanceEBObserverController::starting(const ros::Time & /*time*/)
  {
    std::cout << "--------------start1:NullSpaceImpedanceEBObserverController_3.21_0.01--------------" << std::endl;
    std::cout << "--------------start2:NullSpaceImpedanceEBObserverController_3.21_0.01--------------" << std::endl;
    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    // 基坐标系下的雅可比（上一时刻，用于数值微分）
    std::array<double, 42> jacobian_array_old =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
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

    // 初始位置姿态赋初值
    position_d.setZero();
    d_position_d.setZero();
    dd_position_d.setZero();
    position_d_target.setZero();

    // 主任务相关
    K.topLeftCorner(3, 3) << K_pos * Eigen::Matrix3d::Identity();
    K.bottomRightCorner(3, 3) << K_ori * Eigen::Matrix3d::Identity();
    P.topLeftCorner(3, 3) << P_pos * Eigen::Matrix3d::Identity();
    P.bottomRightCorner(3, 3) << P_ori * Eigen::Matrix3d::Identity();

    // 副任务相关
    Md.setIdentity();
    Bd.setIdentity();
    Kd.setIdentity();

    // 观测器相关
    tau_estimated.setZero();
    gamma = 1 * Eigen::MatrixXd::Identity(7, 7);

    elapsed_time = ros::Duration(0.0);
  }
  void NullSpaceImpedanceEBObserverController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    // 话题发布
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
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.rotation());

    // 轨迹生成
    elapsed_time += t;
    double radius = 0.15;
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 10 * elapsed_time.toSec()));
    double delta_x = radius * std::sin(angle);
    double delta_z = radius * (std::cos(angle) - 1);
    position_d[0] = position_d_target[0] + delta_x;
    position_d[2] = position_d_target[2] + delta_z;

    // 计算雅可比
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data()); // 完整雅可比
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_array_old.data());
    Eigen::Matrix<double, 6, 7> jacobian_dot;
    Eigen::Matrix<double, 6, 7> jacobian_dot_new;
    double r1 = 0.01;
    if (firstUpdate)
    {
      // debug
      myfile.open("/home/wd/NullSpaceImpedanceEBObserverController.txt");
      myfile << "NullSpaceImpedanceEBObserverController_3.15——109\n"
             << std::endl;
      jacobian_dot_new.setZero(); // 若在第一次控制周期，将雅可比导数置为0

      S1 = jacobian; // 0时刻
      jacobian_dot.setZero();
      S1_dot.setZero();
    }
    else
    {
      jacobian_dot_new = (jacobian - jacobian_old) / t.toSec();

      /* jacobian_dot= */ S1_dot = (jacobian - S1) / r1;
      S1 = S1_dot * t.toSec() + S1;
    }
    jacobian_array_old = jacobian_array;

    // 误差计算
    /*   Eigen::Matrix<double, 6, 1> s;
      Eigen::Matrix<double, 6, 1> error;
      Eigen::Matrix<double, 3, 1> error_orientation;
      Eigen::Matrix<double, 6, 1> error_dot;
      error.head(3) << position_d - position;
      for(int i = 0; i < 3; i++)
      {
        if (std::fabs(orientation_d(i) - orientation(i)) <= M_PI)
        {
          error_orientation(i) = orientation_d(i) - orientation(i);
        }
        else
        {
          if (orientation(i) <= 0)
          {
            error_orientation(i) = 2 * M_PI - std::fabs(orientation_d(i) - orientation(i));
          }
          else
          {
            error_orientation(i) = - (2 * M_PI - std::fabs(orientation_d(i) - orientation(i)));
          }
        }
      }
      error.tail(3) << error_orientation;
      error_dot = jacobian * dq;  // 0 - jacobian * dq
      s = error_dot + P * error; */

    Eigen::Matrix<double, 6, 1> s;
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> error_dot;
    error.head(3) << position_d - position; // 提取前三个元素
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << transform.rotation() * error.tail(3); // 提取后三个元素
    error_dot = -jacobian * dq;                            // 0 - jacobian * dq
    s = error_dot + P * error;

    // Lambda计算  Lambda = Λ
    Eigen::Matrix<double, 6, 6> Lambda = (jacobian * mass.inverse() * jacobian.transpose()).inverse();
    Eigen::Matrix<double, 6, 6> Lambda_dot;
    Eigen::Matrix<double, 6, 6> Lambda_dot_new;
    double r2 = 0.01;
    if (firstUpdate)
    {
      Lambda_dot_new.setZero();

      S2 = Lambda;
      Lambda_dot.setZero();
      S2_dot.setZero();
      firstUpdate = false;
    }
    else
    {
      Lambda_dot_new = (Lambda - Lambda_old) / t.toSec();

      /* Lambda_dot= */ S2_dot = (Lambda - S2) / r2;
      S2 = S2_dot * t.toSec() + S2;
    }
    Lambda_old = Lambda;

    // 伪逆矩阵
    Eigen::MatrixXd jacobian_pinv;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    weightedPseudoInverse(jacobian, jacobian_pinv, mass);

    // 命令加速度与输入力矩
    double lambda1 = 0.01;
    Eigen::VectorXd tau_estimated_dot(7);
    Eigen::VectorXd F_estimated(6);
    Eigen::MatrixXd N = I - jacobian_pinv * jacobian; // N
    Eigen::VectorXd term1(7), term2(7), term3(7), term4(7), tau_d(7);
    tau_estimated_dot = -gamma.transpose() * jacobian_pinv * s /* - lambda1 * tau_estimated */;
    tau_estimated << tau_estimated + tau_estimated_dot * t.toSec();
    term1 << jacobian.transpose() * Lambda * (/* ddx_d(0) + */ P * error_dot - S1_dot * dq);
    term2 << jacobian.transpose() * (0.5 * S2_dot + K) * s;
    term3 << jacobian.transpose() * jacobian_pinv.transpose() * tau_estimated; // term3不稳定
    term4 << mass * N * (/*task2_ddq_d(0) + */ mass.inverse() * (Kd * (q_d - q) + Bd * (dq_d - dq)));
    tau_d << term1 + term2 + /* term3 +  */ term4 + coriolis;
    F_estimated = jacobian_pinv.transpose() * tau_estimated;

    // debug
    // time++;
    // myfile << "time: " << time << "_" << std::endl;
    // myfile << "K: " << std::endl;
    // myfile << K << std::endl;
    // myfile << "P: " << std::endl;
    // myfile << P << std::endl;
    // myfile << "gamma: " << std::endl;
    // myfile << gamma << std::endl;
    // myfile << "Kd: " << std::endl;
    // myfile << Kd << std::endl;
    // myfile << "Bd: " << std::endl;
    // myfile << Bd << std::endl;
    // myfile << "Md: " << std::endl;
    // myfile << Md << std::endl;
    // myfile << "R: " << std::endl;
    // myfile << transform.rotation() << std::endl;
    // myfile << "Lambda: " << std::endl;
    // myfile << Lambda << std::endl;
    // myfile << "S2_dot: " << std::endl;
    // myfile << S2_dot << std::endl;
    // myfile << "Lambda_dot_new: " << std::endl;
    // myfile << Lambda_dot_new << std::endl;
    // myfile << "q: " << q.transpose() << std::endl;
    // myfile << "dq: " << dq.transpose() << std::endl;
    // myfile << "q_d: " << q_d.transpose() << std::endl;
    // myfile << "dq_d: " << dq_d.transpose() << std::endl;
    // myfile << "ddq_d: " << ddq_d.transpose() << std::endl;
    // myfile << "position: " << position.transpose() << std::endl;
    // myfile << "position_d: " << position_d.transpose() << std::endl;
    // myfile << "d_position_d: " << d_position_d.transpose() << std::endl;
    // myfile << "dd_position_d: " << dd_position_d.transpose() << std::endl;
    // myfile << "tau_estimated: " << tau_estimated.transpose() << std::endl;
    // myfile << "error: " << error.transpose() << std::endl;
    // myfile << "error_dot: " << error_dot.transpose() << std::endl;
    // myfile << "s: " << s.transpose() << std::endl;
    // myfile << "jacobian:" << std::endl;
    // myfile << jacobian << std::endl;
    // myfile << "S1_dot:" << std::endl;
    // myfile << S1_dot << std::endl;
    // myfile << "jacobian_dot_new:" << std::endl;
    // myfile << jacobian_dot_new << std::endl;
    // myfile << "jacobian_pinv:" << std::endl;
    // myfile << jacobian_pinv << std::endl;
    // myfile << "term2: (0.5 * S2_dot + K).transpose():" << std::endl;
    // myfile << (0.5 * S2_dot + K).transpose() << std::endl;
    // myfile << "term2:jacobian.transpose() * (0.5 * S2_dot + K)" << std::endl;
    // myfile << jacobian.transpose() * (0.5 * S2_dot + K) << std::endl;
    // myfile << "term1:jacobian.transpose() * Lambda: " << std::endl;
    // myfile << jacobian.transpose() * Lambda << std::endl;
    // myfile << "term1:(P * error_dot - S1_dot * dq): " << std::endl;
    // myfile << (P * error_dot - S1_dot * dq) << std::endl;
    // myfile << "term1: " << term1.transpose() << std::endl;
    // myfile << "term2: " << term2.transpose() << std::endl;
    // myfile << "term3: " << term3.transpose() << std::endl;
    // myfile << "term4: " << term4.transpose() << std::endl;
    // myfile << "tau_d: " << tau_d.transpose() << std::endl;
    // myfile << "dq: " << dq.transpose() << std::endl;

    // 发布参数,画图
    for (int i = 0; i < 7; i++)
    {
      param_debug.tau_d[i] = tau_d[i];
      param_debug.term1[i] = term1[i];
      param_debug.term2[i] = term2[i];
      param_debug.term3[i] = term3[i];
      param_debug.term4[i] = term4[i];
    }
    for (int i = 0; i < 6; i++)
    {
      param_debug.cartesianError[i] = error[i];
      param_debug.F_estimated[i] = F_estimated[i];
    }
    paramForDebug.publish(param_debug);

    // 平滑命令
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d[i]); // 关节句柄设置力矩命令
    }

    // 目标位置，控制参数更新
    controllerParamRenew();
  }
  Eigen::Matrix<double, 7, 1> NullSpaceImpedanceEBObserverController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
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
  void NullSpaceImpedanceEBObserverController::complianceParamCallback(franka_example_controllers::nullspace_impedance_paramConfig &config, uint32_t /*level*/)
  {
    K_target.setIdentity();
    K_target.topLeftCorner(3, 3) << config.K_pos * Eigen::Matrix3d::Identity();
    K_target.bottomRightCorner(3, 3) << config.K_ori * Eigen::Matrix3d::Identity();

    P_target.setIdentity();
    P_target.topLeftCorner(3, 3) << config.P_pos * Eigen::Matrix3d::Identity();
    P_target.bottomRightCorner(3, 3) << config.P_ori * Eigen::Matrix3d::Identity();

    gamma_target.setIdentity();
    gamma_target = config.gamma * Eigen::MatrixXd::Identity(7, 7);

    Kd_target = config.Kd * Eigen::MatrixXd::Identity(7, 7);
    Bd_target = config.Bd * Eigen::MatrixXd::Identity(7, 7);
    Md_target = config.Md * Eigen::MatrixXd::Identity(7, 7);
  }
  void NullSpaceImpedanceEBObserverController::controllerParamRenew()
  {
    K = filter_params * K_target + (1.0 - filter_params) * K;
    P = filter_params * P_target + (1.0 - filter_params) * P;
    gamma = filter_params * gamma_target + (1.0 - filter_params) * gamma;

    Kd = filter_params * Kd_target + (1.0 - filter_params) * Kd;
    Bd = filter_params * Bd_target + (1.0 - filter_params) * Bd;
    Md = filter_params * Md_target + (1.0 - filter_params) * Md;
  }

  bool NullSpaceImpedanceMBObserverController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    // 参数服务器
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "NullSpaceImpedanceMBObserverController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "NullSpaceImpedanceMBObserverController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "NullSpaceImpedanceMBObserverController: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "NullSpaceImpedanceMBObserverController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "NullSpaceImpedanceMBObserverController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "NullSpaceImpedanceMBObserverController: Error getting effort joint interface from "
          "hardware");
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
            "NullSpaceImpedanceMBObserverController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&NullSpaceImpedanceMBObserverController::complianceParamCallback, this, _1, _2));

    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    return true;
  }
  void NullSpaceImpedanceMBObserverController::starting(const ros::Time & /*time*/)
  {
    std::cout << "--------------start:NullSpaceImpedanceMBObserverController_3.23--------------" << std::endl;
    std::cout << "--------------start:NullSpaceImpedanceMBObserverController_3.23--------------" << std::endl;
    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    // 基坐标系下的雅可比（上一时刻，用于数值微分）
    std::array<double, 42> jacobian_array_old =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
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

    // 初始位置姿态赋初值
    position_d.setZero();
    position_d_target.setZero();

    // 控制参数赋初值
    int Kp_pos = 50;
    int Kp_ori = 1;
    int Kv_pos = 2;
    int Kv_ori = 1;
    int Ki = 1;
    KI << Ki * Eigen::MatrixXd::Identity(7, 7);
    Kp.topLeftCorner(3, 3) << Kp_pos * Eigen::Matrix3d::Identity();
    Kp.bottomRightCorner(3, 3) << Kp_ori * Eigen::Matrix3d::Identity();
    Kv.topLeftCorner(3, 3) << Kv_pos * Eigen::Matrix3d::Identity();
    Kv.bottomRightCorner(3, 3) << Kv_ori * Eigen::Matrix3d::Identity();

    Md.setIdentity();
    Bd.setIdentity();
    Kd.setIdentity();
  }
  void NullSpaceImpedanceMBObserverController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    // 画图
    franka_example_controllers::paramForDebug param_debug;

    // 获取状态,C,M，q,dq
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // 将array类转成矩阵
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.rotation());

    // 计算雅可比
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data()); // 完整雅可比
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_old(jacobian_array_old.data());
    Eigen::Matrix<double, 6, 7> jacobian_dot;
    double r1 = 0.01;
    if (firstUpdate)
    {
      // debug
      myfile.open("/home/wd/NullSpaceImpedanceMBObserverController.txt");
      myfile << "NullSpaceImpedanceMBObserverController.15——109\n"
             << std::endl;

      S1 = jacobian; // 0时刻
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
    error.head(3) << position_d - position; // 提取前三个元素
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << transform.rotation() * error.tail(3); // 提取后三个元素
    error_dot = -jacobian * dq;                            // 0 - jacobian * dq

    // 伪逆矩阵
    Eigen::MatrixXd jacobian_pinv;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    weightedPseudoInverse(jacobian, jacobian_pinv, mass);

    // 基于动量的观测器
    Eigen::Matrix<double, 7, 1> r;     // 残差向量
    Eigen::Matrix<double, 7, 1> p;     // 广义动量
    Eigen::Matrix<double, 7, 1> H;     // 积分式子
    Eigen::Matrix<double, 7, 1> H_dot; // 积分式子

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
    qc2 << N * (/* task2_ddq_d = 0 */ mass.inverse() * (Kd * (q_d - q) + Bd * (dq_d - dq)));
    tau_d << mass * (qc1 + qc2) + coriolis;

    // 上一个时刻的数据

    // debug
    // time++;
    // myfile << " " << std::endl;
    // myfile << "time: " << time << "_" << std::endl;
    // myfile << "Kp: " << std::endl;
    // myfile << Kp << std::endl;
    // myfile << "Kv: " << std::endl;
    // myfile << Kv << std::endl;
    // myfile << "R: " << std::endl;
    // myfile << transform.rotation() << std::endl;
    // myfile << "Kd: " << std::endl;
    // myfile << Kd << std::endl;
    // myfile << "Bd: " << std::endl;
    // myfile << Bd << std::endl;
    // myfile << "Md: " << std::endl;
    // myfile << Md << std::endl;
    // myfile << "" << std::endl;
    // myfile << "q: " << q.transpose() << std::endl;
    // myfile << "dq: " << dq.transpose() << std::endl;
    // myfile << "q_d: " << q_d.transpose() << std::endl;
    // myfile << "dq_d: " << dq_d.transpose() << std::endl;
    // myfile << "position_d: " << position_d.transpose() << std::endl;
    // myfile << "position: " << position.transpose() << std::endl;
    // myfile << "error: " << error.transpose() << std::endl;
    // myfile << "error_dot: " << error_dot.transpose() << std::endl;
    // myfile << "jacobian:" << std::endl;
    // myfile << jacobian << std::endl;
    // myfile << "S1_dot:" << std::endl;
    // myfile << S1_dot << std::endl;
    // Eigen::MatrixXd II = jacobian * jacobian_pinv;
    // myfile << "jacobian * jacobian_pinv:" << std::endl;
    // myfile << II << std::endl;
    // myfile << "qc1: " << qc1.transpose() << std::endl;
    // myfile << "qc2: " << qc2.transpose() << std::endl;
    // myfile << "tau_d: " << tau_d.transpose() << std::endl;

    // 画图
    for (int i = 0; i < 7; i++)
    {
      param_debug.qc1[i] = qc1[i];
      param_debug.qc2[i] = qc2[i];
      param_debug.tau_d[i] = tau_d[i];
    }
    paramForDebug.publish(param_debug);

    // 平滑命令
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      // joint_handles_[i].setCommand(tau_d(i));  // 关节句柄设置力矩命令
    }

    // 目标位置，控制参数更新
    controllerParamRenew();
  }
  Eigen::Matrix<double, 7, 1> NullSpaceImpedanceMBObserverController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
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
  void NullSpaceImpedanceMBObserverController::complianceParamCallback(franka_example_controllers::nullspace_impedance_paramConfig &config, uint32_t /*level*/)
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

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::NullSpaceImpedanceController,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::NullSpaceImpedanceEBObserverController,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::NullSpaceImpedanceMBObserverController,
                       controller_interface::ControllerBase)