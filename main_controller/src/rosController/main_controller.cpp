// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSEoller
#include <cmath>
#include <memory>

#include <controller/controller.hpp>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>

#include <rosController/main_controller.h>
#include <ros/ros.h>

const char C_Date[12] = __DATE__;
const char C_Time[9] = __TIME__;

// extern Robot7Controller *pController;
// extern Robot7 *pPanda;

namespace main_controller
{

  bool MainController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {

    std::cout << "--------------init1:PandaController--------------" << std::endl;
    std::cout << "--------------init2:PandaController--------------" << std::endl;

    robotInit();

    // 参数服务器
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("MainController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("MainController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("MainController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("MainController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("MainController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("MainController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("MainController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("MainController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // 动态参数服务
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<main_controller::main_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&MainController::controlParamCallback, this, _1, _2));

    // 发布者对象
    paramForDebug = node_handle.advertise<main_controller::paramForDebug>("paramForDebug", 20);

    return true;
  }
  void MainController::starting(const ros::Time & /*time*/)
  {
    std::cout << "--------------start1:PandaController--------------" << std::endl;
    std::cout << "--------------start2:PandaController--------------" << std::endl;
    std::cout << "------编译日期:" << __DATE__ << "------" << std::endl;
    std::cout << "------编译时刻:" << __TIME__ << "------" << std::endl;

    franka::RobotState initial_state = state_handle_->getRobotState();
    Eigen::Matrix<double, 7, 1> q_initial = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    robotStart(q_initial, 1);
  }
  void MainController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    // 发布数据
    main_controller::paramForDebug param_debug;

    franka::RobotState robot_state = state_handle_->getRobotState();

    Eigen::Matrix<double, 7, 1> q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    Eigen::Matrix<double, 7, 1> dq = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
    Eigen::Matrix<double, 7, 1> tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
    Eigen::Matrix<double, 7, 1> tau_d;

    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.rotation());

    robotRun(q, dq, tau_J_d, position, orientation, transform, tau_d);

    // 平滑命令
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i)); // 关节句柄设置力矩命令
    }

    pController->pubData(param_debug, pPanda);

    paramForDebug.publish(param_debug);

    pController->controllerParamRenew();
  }

  Eigen::Matrix<double, 7, 1> MainController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }

  void MainController::controlParamCallback(main_controller::main_controller_paramConfig &config, uint32_t /*level*/)
  {
    pController->dynamicSetParameter(config);
  }

} // namespace main_controller

PLUGINLIB_EXPORT_CLASS(main_controller::MainController,
                       controller_interface::ControllerBase)
