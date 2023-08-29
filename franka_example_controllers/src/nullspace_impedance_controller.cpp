// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/nullspace_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers
{

  bool NullSpaceImpedanceController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "[------------------] init1:NullSpaceImpedanceController" << std::endl;
    std::cout << "[------------------] init2:NullSpaceImpedanceController" << std::endl;

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
      ROS_ERROR(
          "NullSpaceImpedanceController: Invalid or no joint_names parameters provided, "
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
      ROS_ERROR_STREAM(
          "NullSpaceImpedanceController: Exception getting model handle from interface: " << ex.what());
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
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&NullSpaceImpedanceController::complianceParamCallback, this, _1, _2));

    // 发布者对象
    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    return true;
  }

  void NullSpaceImpedanceController::starting(const ros::Time & /*time*/)
  {
    std::cout << "[------------------] start1:NullSpaceImpedanceController\n";
    std::cout << "[------------------] start2:NullSpaceImpedanceController\n";
    std::cout << "[------------------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------------------] 编译时刻:" << __TIME__ << "\n";
    std::cout << "[------------------] 无观测器" << std::endl;

    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    this->T0 = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())); // 齐次变换矩阵
    this->q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    this->X0 << Eigen::Vector3d(this->T0.translation()), Eigen::Quaterniond(this->T0.rotation()).toRotationMatrix().eulerAngles(2, 1, 0);

    this->task2_q_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    this->task2_dq_d.setZero();
    this->task2_ddq_d.setZero();

    this->myfile.open("/home/wd/log/franka/NullSpaceImpedanceController.txt");
    this->myfile << "NullSpaceImpedanceController" << std::endl;
  }

  void NullSpaceImpedanceController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    upDateParam();
    // dJ todo：差分改滤波
    double r1 = 0.1;
    if (time == 0)
    {
      S1 = this->J;
      S1_dot.setZero();
    }
    else
    {
      /* dJ= */ S1_dot = (this->J - S1) / r1;
      S1 = S1_dot * t.toSec() + S1;
    }
    this->dJ = S1_dot;

    // 轨迹和误差  3轨迹最差
    static Eigen::Matrix<double, 6, 1> X_d, dX_d, ddX_d, Xerror, dXerror;
    // cartesianTrajectoryXZ1(time / 1000, 0.6, 0.5, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);
    // cartesianTrajectoryXZ2(time / 1000, 0.6, 0.5, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);
    // cartesianTrajectoryXZ3(time / 1000, 0.6, 0.8, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);
    cartesianTrajectory0(time / 1000, 0.8, 0.5, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);

    // 伪逆矩阵计算
    Eigen::MatrixXd J_pinv;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    weightedPseudoInverse(J, J_pinv, this->M);

    // 命令加速度与输入力矩
    Eigen::Matrix<double, 7, 1> qc1, qc2;
    qc1 << J_pinv * (ddX_d + task1_Kp * Xerror + task1_Kv * dXerror - dJ * dq);
    Eigen::MatrixXd N = I - J_pinv * J;
    qc2 << N * (task2_ddq_d + this->M.inverse() * (task2_Kd * (task2_q_d - q) + task2_Bd * (task2_dq_d - dq)));
    this->tau_d << this->M * (qc1 + qc2) + this->c;

    // 记录数据
    this->time++;
    recordData();
    this->myfile << "Xerror: " << Xerror.transpose() << "\n";
    this->myfile << "X_d: " << X_d.transpose() << "\n";
    this->myfile << "X: " << this->X.transpose() << "\n";
    this->myfile << " " << std::endl; // 刷新缓冲区

    // 画图
    for (int i = 0; i < 7; i++)
    {
      this->param_debug.tau_d[i] = this->tau_d[i];
      if (i == 6)
        break;
      this->param_debug.X[i] = this->X[i];
      this->param_debug.X_d[i] = X_d[i];
      this->param_debug.dX[i] = this->dX[i];
      this->param_debug.dX_d[i] = dX_d[i];
      this->param_debug.Xerror[i] = Xerror[i];
      this->param_debug.dXerror[i] = dXerror[i];
    }
    this->paramForDebug.publish(this->param_debug);

    // 目标位置，控制参数更新
    controllerParamRenew();

    // 平滑命令
    tau_d << saturateTorqueRate(this->tau_d, this->tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(this->tau_d(i)); // 关节句柄设置力矩命令
    }
  }

  void NullSpaceImpedanceController::upDateParam()
  {
    // 获取动力学数据
    this->M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(model_handle_->getMass().data());
    this->c = Eigen::Map<Eigen::Matrix<double, 7, 1>>(model_handle_->getCoriolis().data());
    this->G = Eigen::Map<Eigen::Matrix<double, 7, 1>>(model_handle_->getGravity().data());
    this->J = Eigen::Map<Eigen::Matrix<double, 6, 7>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());

    // 获取传感器数据
    this->robot_state = state_handle_->getRobotState();
    this->q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    this->dq = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
    this->tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
    this->T = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    this->X << Eigen::Vector3d(this->T.translation()), Eigen::Quaterniond(this->T.rotation()).toRotationMatrix().eulerAngles(2, 1, 0);
    this->dX = this->J * this->dq;
  }

  void NullSpaceImpedanceController::recordData()
  {
    this->myfile << "time: " << this->time << "_\n";

    // this->myfile << "tau_d: " << this->tau_d.transpose() << "\n";
    // this->myfile << "q: " << this->q.transpose() << "\n";
    // this->myfile << "dq: " << this->dq.transpose() << "\n";
    // this->myfile << "X: " << this->X.transpose() << "\n";
    // this->myfile << "dX: " << this->dX.transpose() << "\n";

    // this->myfile << "J: \n";
    // this->myfile << this->J << "\n";
    // this->myfile << "dJ: \n";
    // this->myfile << this->dJ << "\n";
    // this->myfile << "M: \n";
    // this->myfile << this->M << "\n";
    // this->myfile << "c: \n";
    // this->myfile << this->c.transpose() << "\n";
    // this->myfile << "G: \n";
    // this->myfile << this->G.transpose() << "\n";

    // this->myfile << "R: \n";
    // this->myfile << this->T.rotation() << "\n";
    // this->myfile << "task1_Kp: \n";
    // this->myfile << task1_Kp << "\n";
    // this->myfile << "task1_Kv: \n";
    // this->myfile << task1_Kv << "\n";
    // this->myfile << "task2_q_d: " << task2_q_d.transpose() << "\n";
    // this->myfile << "task2_dq_d: " << task2_dq_d.transpose() << "\n";
    // this->myfile << "task2_ddq_d: " << task2_ddq_d.transpose() << "\n";
  }

  void NullSpaceImpedanceController::complianceParamCallback(franka_example_controllers::nullspace_impedance_controller_paramConfig &config, uint32_t /*level*/)
  {
    if (time == 0)
    {
      task1_Kp.setIdentity();
      task1_Kp.topLeftCorner(3, 3) << config.task1_Kp_pos * Eigen::Matrix3d::Identity();
      task1_Kp.bottomRightCorner(3, 3) << config.task1_Kp_ori * Eigen::Matrix3d::Identity();

      task1_Kv.setIdentity();
      task1_Kv.topLeftCorner(3, 3) << config.task1_Kv_pos * Eigen::Matrix3d::Identity();
      task1_Kv.bottomRightCorner(3, 3) << config.task1_Kv_ori * Eigen::Matrix3d::Identity();
    }

    task1_Kp_target.setIdentity();
    task1_Kp_target.topLeftCorner(3, 3) << config.task1_Kp_pos * Eigen::Matrix3d::Identity();
    task1_Kp_target.bottomRightCorner(3, 3) << config.task1_Kp_ori * Eigen::Matrix3d::Identity();

    task1_Kv_target.setIdentity();
    task1_Kv_target.topLeftCorner(3, 3) << config.task1_Kv_pos * Eigen::Matrix3d::Identity();
    task1_Kv_target.bottomRightCorner(3, 3) << config.task1_Kv_ori * Eigen::Matrix3d::Identity();

    task2_Kd = config.task2_Kd * Eigen::MatrixXd::Identity(7, 7);
    task2_Bd = config.task2_Bd * Eigen::MatrixXd::Identity(7, 7);
    task2_Md = config.task2_Md * Eigen::MatrixXd::Identity(7, 7);
  }
  void NullSpaceImpedanceController::controllerParamRenew()
  {
    task1_Kp = filter_params * task1_Kp_target + (1.0 - filter_params) * task1_Kp;
    task1_Kv = filter_params * task1_Kv_target + (1.0 - filter_params) * task1_Kv;

    task2_Kd = filter_params * task2_Kd + (1.0 - filter_params) * task2_Kd;
    task2_Bd = filter_params * task2_Bd + (1.0 - filter_params) * task2_Bd;
    task2_Md = filter_params * task2_Md + (1.0 - filter_params) * task2_Md;
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
} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::NullSpaceImpedanceController,
                       controller_interface::ControllerBase)
