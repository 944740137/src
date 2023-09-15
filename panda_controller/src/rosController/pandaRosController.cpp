// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSEoller
#include <cmath>
#include <memory>

#include <robotController/pandaController.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>

#include <rosController/pandaRosController.h>
#include <ros/ros.h>

namespace panda_controller
{
  bool PandaController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {

    std::cout << "[robotController] init1:panda_controller" << std::endl;
    std::cout << "[robotController] init2:panda_controller" << std::endl;

    std::string urdfPath;
    std::string TcpName;
    if (!node_handle.getParam("/panda_withoutHand_dyn", urdfPath))
    {
      ROS_ERROR_STREAM("panda_controller: Could not read parameter urdfPath");
    }
    if (!node_handle.getParam("/TCP_name", TcpName))
    {
      ROS_ERROR_STREAM("panda_controller: Could not read parameter TcpName");
    }
    pandaInit(urdfPath, TcpName);

    // 参数服务器
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("panda_controller: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("panda_controller: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("panda_controller: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("panda_controller: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("panda_controller: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("panda_controller: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("panda_controller: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("panda_controller: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // 动态参数服务
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<panda_controller::panda_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&PandaController::controlParamCallback, this, _1, _2));

    // 发布者对象
    paramForDebug = node_handle.advertise<panda_controller::paramForDebug>("paramForDebug", 20);

    return true;
  }

  void PandaController::starting(const ros::Time & /*time*/)
  {
    std::cout << "[robotController] start1:panda_controller\n";
    std::cout << "[robotController] start2:panda_controller\n";
    std::cout << "[robotController] 编译日期:" << __DATE__ << "\n";
    std::cout << "[robotController] 编译时刻:" << __TIME__ << std::endl;

    // 初值设置
    franka::RobotState initial_state = state_handle_->getRobotState();
    this->q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    this->theta = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.theta.data());
    this->dq = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.dq.data());
    this->tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.tau_J_d.data());
    this->transform = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())); // 齐次变换矩阵
    this->position = Eigen::Vector3d(transform.translation());
    this->orientation = Eigen::Quaterniond(transform.rotation());
    pandaStart(this->q, this->theta, this->dq, this->tau_J_d, this->position, this->orientation, this->transform, 1);
  }

  void PandaController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    // 获取传感器数据
    this->robot_state = state_handle_->getRobotState();
    this->q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    this->theta = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.theta.data());
    this->dq = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
    this->tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
    this->transform = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    this->position = Eigen::Vector3d(transform.translation());
    this->orientation = Eigen::Quaterniond(transform.rotation());

    // 获取动力学数据
    this->M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(model_handle_->getMass().data());
    this->c = Eigen::Map<Eigen::Matrix<double, 7, 1>>(model_handle_->getCoriolis().data());
    this->G = Eigen::Map<Eigen::Matrix<double, 7, 1>>(model_handle_->getGravity().data());
    this->J = Eigen::Map<Eigen::Matrix<double, 6, 7>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());

    static Eigen::Matrix<double, 7, 1> q_d;
    // 更新franka计算的动力学数据
    pandaGetDyn(this->M, this->c, this->G, this->J);
    // 更新控制力矩或目标位置(只发目标位置时，使用franka的控制器)
    pandaRun(this->q, this->dq, this->theta, this->tau_J_d, this->position, this->orientation, this->transform, this->tau_d, this->param_debug, q_d);

    // 平滑力矩命令并发布
    this->tau_d << saturateTorqueRate(this->tau_d, this->tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(this->tau_d(i)); // 关节句柄设置力矩命令
    }

    // pub
    paramForDebug.publish(this->param_debug);
  }

  Eigen::Matrix<double, 7, 1> PandaController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }

  void PandaController::controlParamCallback(panda_controller::panda_controller_paramConfig &config, uint32_t /*level*/)
  {
    // pController->dynamicSetParameter(config);
  }

} // namespace panda_controller

PLUGINLIB_EXPORT_CLASS(panda_controller::PandaController,
                       controller_interface::ControllerBase)
