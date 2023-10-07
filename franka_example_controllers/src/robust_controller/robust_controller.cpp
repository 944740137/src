// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/pinocchino_interactive.h>

#include <franka_example_controllers/robust_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

#include <fstream>
#include <iostream>

extern PandaDynLibManager *pPandaDynLibManager;

namespace franka_example_controllers
{
  bool RobustController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "[------------------] init1:RobustController" << std::endl;
    std::cout << "[------------------] init2:RobustController" << std::endl;

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

    if (pPandaDynLibManager == nullptr)
      pPandaDynLibManager = new PandaDynLibManager(urdfPath, TcpName);

    // 参数服务器
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("RobustController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "RobustController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("RobustController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "RobustController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("RobustController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("RobustController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("RobustController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("RobustController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::robust_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&RobustController::complianceParamCallback, this, _1, _2));
    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    struct sched_param param = {5};
    if (sched_setscheduler(getpid(), SCHED_FIFO, &param) != 0)
    {
      printf("-------------主进程初始化失败-------------\n");
    };
    sched_getparam(0, &param);
    printf("-------------主进程初始化成功，调度策略: %d, 调度优先级: %d-------------\n",
           sched_getscheduler(0), param.sched_priority);

    return true;
  }

  void RobustController::starting(const ros::Time & /*time*/)
  {
    std::cout << "[------------------] start1:RobustController\n";
    std::cout << "[------------------] start2:RobustController\n";
    std::cout << "[------------------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------------------] 编译时刻:" << __TIME__ << "\n";
    std::cout << "[------------------] 鲁棒控制" << std::endl;

    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    this->T0 = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())); // 齐次变换矩阵
    this->q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    this->X0 << Eigen::Vector3d(this->T0.translation()), Eigen::Quaterniond(this->T0.rotation()).toRotationMatrix().eulerAngles(2, 1, 0);
    this->q_observe = this->q0;
    this->dq_observe.setZero();
    this->ddq_observe.setZero();

    if (!this->myfile.is_open())
    {
      this->myfile.open("/home/wd/log/franka/master/RobustController.txt");
      this->myfile << "RobustController" << std::endl;
      this->myfile << "编译日期:" << __DATE__ << "\n";
      this->myfile << "编译时刻:" << __TIME__ << "\n";
      this->myfile << "鲁棒控制" << std::endl;
    }
  }
  void RobustController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    this->time++;
    upDateParam(t);
    recordData();

    static Eigen::Matrix<double, 7, 1> qerror_1 = Eigen::MatrixXd::Zero(7, 1);
    static Eigen::Matrix<double, 7, 1> qerror_2 = Eigen::MatrixXd::Zero(7, 1);
    static Eigen::Matrix<double, 7, 1> u_psi = Eigen::MatrixXd::Zero(7, 1);
    static Eigen::Matrix<double, 7, 1> tmp = Eigen::MatrixXd::Zero(7, 1);

    // 关节：
    static Eigen::Matrix<double, 7, 1> aixs, q_d, dq_d, ddq_d, qerror, dqerror, dqerror_filter, dqerror_observe, u, u_ob, u_fil;
    aixs << 1, 1, 1, /**/ 1, 1, 1, /**/ 1;
    // int waitTime = 15;
    // if (time >= 1000 * waitTime)
    // {
    //   if (time == 1000 * waitTime + 1)
    //     std::cout << "[------------------] 睡眠结束" << std::endl;
    JointCosTrajectory(aixs, (time) / 1000, 0.4, 0.4, this->q0, this->q, this->dq, q_d, dq_d, ddq_d, qerror, dqerror);
    // }
    // else
    // {
    //   if (time == 1)
    //     std::cout << "[------------------] 睡眠开始" << std::endl;
    //   q_d = this->q0;
    //   dq_d.setZero();
    //   ddq_d.setZero();
    //   qerror = q_d - this->q;
    //   dqerror = dq_d - this->dq;
    // }

    // ob
    this->ddq_observe = ddq_d - u_ob + Lv * (this->q - this->q_observe);
    this->dq_observe = this->dq_observe + this->ddq_observe * t.toSec();
    this->dq_observe = this->dq_observe + Lp * (this->q - this->q_observe);
    this->q_observe = this->q_observe + this->dq_observe * t.toSec();

    dqerror_filter = dq_d - dq_filter;
    dqerror_observe = dq_d - dq_observe;
    u = -this->Kp * qerror - this->Kv * dqerror;
    u_fil = -this->Kp * qerror - this->Kv * dqerror_filter;
    u_ob = -this->Kp * qerror - this->Kv * dqerror_observe;

    // 鲁棒
    qerror_2 = this->q - this->q_observe;
    tmp = tmp + 0.5 * (qerror_1 + qerror_2) * t.toSec();
    u_psi = -this->Lu * tmp;
    qerror_1 = qerror_2;

    // 笛卡尔：轨迹和误差  3轨迹最差
    // static Eigen::Matrix<double, 6, 1> X_d, dX_d, ddX_d, Xerror, dXerror;
    // cartesianTrajectoryXZ1(time / 1000, 0.6, 0.5, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);
    // cartesianTrajectory0(time / 1000, 0.8, 0.5, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);

    // con
    // this->tau_d = this->M * (ddq_d - u) + this->C_pin * this->dq; // ------------------------------- CV:传感器速度，无鲁棒
    // if (time == 1)
    //   std::cout << "传感器获取速度，无鲁棒" << std::endl;

    // this->tau_d = this->M * (ddq_d - u + u_psi) + this->C_pin * this->dq; // ------------------------------- CM:传感器速度，有鲁棒
    // if (time == 1)
    //   std::cout << "传感器获取速度，有鲁棒" << std::endl;

    // this->tau_d = this->M * (ddq_d - u_fil) + this->C_pin_filter * this->dq_filter; // -------------- ?:滤波速度，无鲁棒
    // if (time == 1)
    //   std::cout << "传感器获取速度并滤波，无鲁棒" << std::endl;

    // this->tau_d = this->M * (ddq_d - u_ob) + this->C_pin_observe * this->dq_observe; // ------------ OB:观测器速度，无鲁棒
    // if (time == 1)
    //   std::cout << "观测器获取速度，无鲁棒项" << std::endl;

    this->tau_d = this->M * (ddq_d - u_ob + u_psi) + this->C_pin_observe * this->dq_observe; // ---- PM:观测器速度,有鲁棒
    if (time == 1)
      std::cout << "观测器获取速度,有鲁棒项" << std::endl;

    // 画图
    for (int i = 0; i < 7; i++)
    {
      this->param_debug.q[i] = this->q[i];
      this->param_debug.q_observe[i] = this->q_observe[i];
      this->param_debug.q_d[i] = q_d[i];
      this->param_debug.qError[i] = qerror[i];
      this->param_debug.dqError[i] = dqerror[i];

      this->param_debug.dq[i] = this->dq[i];
      this->param_debug.dq_filter[i] = this->dq_filter[i];
      this->param_debug.dq_observe[i] = this->dq_observe[i];
      this->param_debug.dq_d[i] = dq_d[i];

      this->param_debug.tau_d[i] = this->tau_d[i];
      this->param_debug.tau_J[i] = this->tau_J[i];
      this->param_debug.tau_J_d[i] = this->tau_J_d[i];
      this->param_debug.tau_ext[i] = this->tau_ext[i];
      if (i == 6)
        break;
      this->param_debug.F_ext0[i] = this->F_ext0[i];
      this->param_debug.F_extK[i] = this->F_extK[i];
    }
    this->paramForDebug.publish(this->param_debug);

    // 平滑命令
    tau_d << saturateTorqueRate(this->tau_d, this->tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(this->tau_d(i)); // 关节句柄设置力矩命令
    }

    // this->myfile << "q0: " << this->q0.transpose() << "\n";
    // this->myfile << "q: " << this->q.transpose() << "\n";
    // this->myfile << "dq: " << this->dq.transpose() << "\n";
    // this->myfile << "dq_d: " << dq_d.transpose() << "\n";
    // this->myfile << "u_ob: " << u_ob.transpose() << "\n";
    // this->myfile << "q_observe: " << q_observe.transpose() << "\n";
    // this->myfile << "dq_observe: " << dq_observe.transpose() << "\n";
    // this->myfile << "ddq_observe: " << ddq_observe.transpose() << "\n";
    // this->myfile << "dqerror_observe: " << dqerror_observe.transpose() << "\n";
    // this->myfile << "qerror_2: " << qerror_2.transpose() << "\n";

    // this->myfile << "qerror: " << qerror.transpose() << "\n";
    // this->myfile << "dqerror: " << dqerror.transpose() << "\n";
    // this->myfile << "this->tau_d: " << this->tau_d.transpose() << "\n";
    // this->myfile << "------------------------------------\n";
    controllerParamRenew();
  }

  //*//
  //*//

  Eigen::Matrix<double, 7, 1> RobustController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }

  void RobustController::upDateParam(const ros::Duration &t)
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
    this->tau_J = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
    this->tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
    this->T = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    this->X << Eigen::Vector3d(this->T.translation()), Eigen::Quaterniond(this->T.rotation()).toRotationMatrix().eulerAngles(2, 1, 0);
    this->dX = this->J * this->dq;

    // 估计
    this->tau_ext = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_ext_hat_filtered.data());
    this->F_ext0 = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data());
    this->F_extK = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.K_F_ext_hat_K.data());

    static Eigen::Matrix<double, 6, 7> S1 = Eigen::MatrixXd::Zero(6, 7); // J
    static Eigen::Matrix<double, 6, 7> S1_dot = Eigen::MatrixXd::Zero(6, 7);
    static Eigen::Matrix<double, 7, 1> S2 = Eigen::MatrixXd::Zero(7, 1); // dq
    static Eigen::Matrix<double, 7, 1> S2_dot = Eigen::MatrixXd::Zero(7, 1);
    double r1 = 0.1;
    double r2 = 0.2;
    if (time == 0)
    {
      S1 = this->J;
      S2 = this->dq;
      S1_dot.setZero();
      S2_dot.setZero();
    }
    else
    {
      /* dJ= */ S1_dot = (this->J - S1) / r1;
      S1 = S1_dot * t.toSec() + S1;
      /* ddq= */ S2_dot = (this->dq - S2) / r2;
      S2 = S2_dot * t.toSec() + S2;
    }
    this->dJ_diff = S1_dot;
    this->dq_filter = S2;

    //
    pPandaDynLibManager->upDataModel(this->q);
    pPandaDynLibManager->computeKinData(this->J_pin, this->dJ_pin, this->q, this->dq);
    pPandaDynLibManager->computeKinData(this->J_pin, this->dJ_pin_filter, this->q, this->dq_filter);
    pPandaDynLibManager->computeKinData(this->J_pin, this->dJ_pin_observe, this->q, this->dq_observe);

    pPandaDynLibManager->computeDynData(this->M_pin, this->C_pin, this->G_pin, this->q, this->dq);
    pPandaDynLibManager->computeDynData(this->M_pin, this->C_pin_filter, this->G_pin, this->q, this->dq_filter);
    pPandaDynLibManager->computeDynData(this->M_pin, this->C_pin_observe, this->G_pin, this->q, this->dq_observe);
  }

  void RobustController::recordData()
  {
    // this->myfile << "time: " << this->time << "_\n";
    if (this->time == 5)
      return;
    // this->myfile << "J_pin1: \n";
    // this->myfile << this->J_pin1 << "\n";
    // this->myfile << "tau_msr: " << this->tau_msr.transpose() << "\n";
    // this->myfile << "r: " << this->r.transpose() << "\n";
    // this->myfile << "dvc: " << this->dvc.transpose() << "\n";
    // this->myfile << "J: \n";
    // this->myfile << this->J << "\n";
    // this->myfile << "J_pin: \n";
    // this->myfile << this->J_pin << "\n";
    // this->myfile << "dJ_diff: \n";
    // this->myfile << this->dJ_diff << "\n";
    // this->myfile << "dJ_pin: \n";
    // this->myfile << this->dJ_pin << "\n";
    // this->myfile << "dJ_pin_filter: \n";
    // this->myfile << this->dJ_pin_filter << "\n";
    // this->myfile << "dJ_pin_observe: \n";
    // this->myfile << this->dJ_pin_observe << "\n";

    // this->myfile << "M: \n";
    // this->myfile << this->M << "\n";
    // this->myfile << "M_inv: \n";
    // this->myfile << this->M.inverse() << "\n";
    // this->myfile << "M_pin: \n";
    // this->myfile << this->M_pin << "\n";
    // this->myfile << "c: \n";
    // this->myfile << this->c.transpose() << "\n";
    // this->myfile << "C_pin*dq: \n";
    // this->myfile << (this->C_pin * this->dq).transpose() << "\n";
    // this->myfile << "C_pin_filter*dq: \n";
    // this->myfile << (this->C_pin_filter * this->dq_filter).transpose() << "\n";
    // this->myfile << "C_pin_observe*dq: \n";
    // this->myfile << (this->C_pin_observe * this->dq_observe).transpose() << "\n";
    // this->myfile << "G: \n";
    // this->myfile << this->G.transpose() << "\n";
    // this->myfile << "Kp: \n";
    // this->myfile << this->Kp << "\n";
    // this->myfile << "Kp_d: \n";
    // this->myfile << this->Kp_d << "\n";
    // this->myfile << "Kv: \n";
    // this->myfile << this->Kv << "\n";
    // this->myfile << "Kv_d: \n";
    // this->myfile << this->Kv_d << "\n";
  }

  void RobustController::complianceParamCallback(franka_example_controllers::robust_controller_paramConfig &config, uint32_t /*level*/)
  {
    const double wc_values[] = {config.wc1, config.wc2, config.wc3, config.wc4, config.wc5, config.wc6, config.wc7};
    const double wo_values[] = {config.wo1, config.wo2, config.wo3, config.wo4, config.wo5, config.wo6, config.wo7};

    const double Kp_values[] = {config.Kp1, config.Kp2, config.Kp3, config.Kp4, config.Kp5, config.Kp6, config.Kp7};
    const double Kv_values[] = {config.Kv1, config.Kv2, config.Kv3, config.Kv4, config.Kv5, config.Kv6, config.Kv7};

    if (time == 0)
    {
      for (int i = 0; i < 7; ++i)
      {
        // this->Kp(i, i) = wc_values[i] * wc_values[i];
        // this->Kv(i, i) = 2 * wc_values[i];
        this->Kp(i, i) = Kp_values[i];
        this->Kv(i, i) = Kv_values[i];

        this->Lv(i, i) = wo_values[i] * wo_values[i];
        this->Lp(i, i) = 2 * wo_values[i];
      }
      this->Lu = config.L_u * Eigen::MatrixXd::Identity(7, 7);
    }

    for (int i = 0; i < 7; ++i)
    {
      // this->Kp_d(i, i) = wc_values[i] * wc_values[i];
      // this->Kv_d(i, i) = 2 * wc_values[i];
      this->Kp_d(i, i) = Kp_values[i];
      this->Kv_d(i, i) = Kv_values[i];

      this->Lv_d(i, i) = wo_values[i] * wo_values[i];
      this->Lp_d(i, i) = 2 * wo_values[i];
    }
    this->Lu_d = config.L_u * Eigen::MatrixXd::Identity(7, 7);
  }

  void RobustController::controllerParamRenew()
  {
    Kp = filter_params * Kp_d + (1.0 - filter_params) * Kp;
    Kv = filter_params * Kv_d + (1.0 - filter_params) * Kv;
    Lp = filter_params * Lp_d + (1.0 - filter_params) * Lp;
    Lv = filter_params * Lv_d + (1.0 - filter_params) * Lv;
    Lu = filter_params * Lu_d + (1.0 - filter_params) * Lu;
  }

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::RobustController,
                       controller_interface::ControllerBase)
