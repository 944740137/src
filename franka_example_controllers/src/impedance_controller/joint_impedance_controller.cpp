// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/pinocchino_interactive.h>

#include <franka_example_controllers/joint_impedance_controller.h>

#include <math.h>
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
  bool JointImpedanceController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "[------------------] init1:JointImpedanceController" << std::endl;
    std::cout << "[------------------] init2:JointImpedanceController" << std::endl;

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
      ROS_ERROR_STREAM("JointImpedanceController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "JointImpedanceController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("JointImpedanceController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "JointImpedanceController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("JointImpedanceController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("JointImpedanceController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("JointImpedanceController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("JointImpedanceController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::impedance_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&JointImpedanceController::complianceParamCallback, this, _1, _2));

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

  void JointImpedanceController::starting(const ros::Time & /*time*/)
  {
    std::cout << "[------------------] start1:JointImpedanceController\n";
    std::cout << "[------------------] start2:JointImpedanceController\n";
    std::cout << "[------------------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------------------] 编译时刻:" << __TIME__ << "\n";
    std::cout << "[------------------] 关节阻抗" << std::endl;

    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    this->T0 = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())); // 齐次变换矩阵
    this->q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    this->X0 << Eigen::Vector3d(this->T0.translation()), Eigen::Quaterniond(this->T0.rotation()).toRotationMatrix().eulerAngles(2, 1, 0);

    this->task2_q_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    if (!this->myfile.is_open())
    {
      this->myfile.open("/home/wd/log/franka/master/JointImpedanceController.txt");
      this->myfile << "JointImpedanceController" << std::endl;
      this->myfile << "编译日期:" << __DATE__ << "\n";
      this->myfile << "编译时刻:" << __TIME__ << "\n";
      this->myfile << "关节阻抗" << std::endl;
      // this->r << 1, 1, 1, 1, 1, 1, 1;
      // this->r_d << 1, 1, 1, 1, 1, 1, 1;
    }
  }
  double sgn(double s)
  {
    double detla = 0.1;

    s = s / (std::fabs(s) + detla);

    return s;
  }
  void JointImpedanceController::update(const ros::Time & /*time*/, const ros::Duration &t)
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

    // 轨迹和误差
    static Eigen::Matrix<double, 7, 1> aixs, q_d, dq_d, ddq_d, qerror, dqerror;
    aixs << 1, 1, 1, /**/ 1, 0, 0, /**/ 0;
    // JointCosTrajectory(aixs, (time) / 1000, 0.4, 0.6, this->q0, this->q, this->dq, q_d, dq_d, ddq_d, qerror, dqerror);
    Joint0Trajectory(aixs, (time) / 1000, 0.4, 0.4, this->q0, this->q, this->dq, q_d, dq_d, ddq_d, qerror, dqerror);

    // 伪逆矩阵计算
    static Eigen::MatrixXd J_pinv;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    weightedPseudoInverse(J, J_pinv, this->M);

    // 命令加速度与输入力矩
    static Eigen::Matrix<double, 7, 1> tmp1;
    static Eigen::Matrix<double, 7, 1> tmp2;

    // ****************************************************新阻抗***************************************************
    // for (int i = 0; i < 7; i++)
    // {
    //   tmp1[i] = dqerror[i] * std::fabs(dqerror[i]);
    //   tmp2[i] = sgn(qerror[i] + (tmp1[i]) / (2 * r[i]));
    //   this->qc[i] = ddq_d[i] + this->r[i] * tmp2[i];
    // }
    // this->tau_d << this->M * (this->qc) + this->c;
    // if (this->time == 0)
    //   std::cout << "新阻抗" << std::endl;
    // ************************************************************************************************************

    // ****************************************************老阻抗***************************************************
    // this->tau_d << this->M * (ddq_d + this->Md.inverse() * (this->Kd * qerror + this->Dd * dqerror)) + this->c;
    // if (this->time == 0)
    //   std::cout << "老阻抗" << std::endl;
    // *************************************************************************************************************

    // ****************************************************变阻尼阻抗*************************************************
    double bar = 0.5;
    double rd = 2;
    static Eigen::Matrix<double, 7, 7> Dv = Eigen::MatrixXd::Zero(7, 7);
    static Eigen::Matrix<double, 7, 1> fen_mu = Eigen::MatrixXd::Zero(7, 1);
    for (int i = 0; i < 7; i++)
    {
      fen_mu[i] = (bar * bar - dqerror[i] * dqerror[i]);
      Dv(i, i) = rd / fen_mu[i];
    }
    this->tau_d << this->M * (ddq_d + this->Md.inverse() * (this->Kd * qerror + (this->Dd + Dv) * dqerror)) + this->c;
    if (this->time == 0)
      std::cout << "变阻尼阻抗" << std::endl;
    // *************************************************************************************************************

    // 记录数据
    this->time++;
    // recordData();
    // this->myfile << "Dv: " << Dv(1, 1) << "\n";
    // this->myfile << "dqerror: " << dqerror.transpose() << "\n";
    // this->myfile << "fen_mu: " << fen_mu[1] << "\n";
    // this->myfile << "tau_d: " << tau_d.transpose() << "\n";
    // this->myfile << "this->qc:" << this->qc.transpose() << "\n";
    // this->myfile << "tmp1:" << tmp1.transpose() << "\n";
    // this->myfile << "tmp2:" << tmp2.transpose() << "\n";
    // this->myfile << " " << std::endl; // 刷新缓冲区

    // 画图
    for (int i = 0; i < 7; i++)
    {
      this->param_debug.tau_d[i] = this->tau_d[i];
      this->param_debug.tau_J[i] = this->tau_J[i];
      this->param_debug.tau_J_d[i] = this->tau_J_d[i];
      this->param_debug.q[i] = this->q[i];
      this->param_debug.q_d[i] = q_d[i];
      this->param_debug.dq[i] = this->dq[i];
      this->param_debug.dq_d[i] = dq_d[i];
      this->param_debug.qError[i] = qerror[i];
      this->param_debug.dqError[i] = dqerror[i];
      this->param_debug.fen_mu[i] = fen_mu[i];
      this->param_debug.D[i] = (this->Dd + Dv)(i, i);
    }

    // // 平滑命令
    // tau_d << saturateTorqueRate(this->tau_d, this->tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(this->tau_d(i)); // 关节句柄设置力矩命令
    }
    for (int i = 0; i < 7; i++)
    {
      this->param_debug.tau_d_saturate[i] = this->tau_d[i];
    }
    this->paramForDebug.publish(this->param_debug);

    // 目标位置，控制参数更新
    controllerParamRenew();
  }

  Eigen::Matrix<double, 7, 1> JointImpedanceController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }

  void JointImpedanceController::upDateParam()
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

    this->tau_ext = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_ext_hat_filtered.data());
    this->F_ext0 = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data());
    this->F_extK = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.K_F_ext_hat_K.data());

    this->comRatio = robot_state.control_command_success_rate;
    //
    pPandaDynLibManager->upDataModel(this->q);
    pPandaDynLibManager->computeKinData(this->J_pin, this->dJ_pin, this->q, this->dq);
    pPandaDynLibManager->computeDynData(this->M_pin, this->C_pin, this->G_pin, this->q, this->dq);
  }

  void JointImpedanceController::recordData()
  {
    this->myfile << "time: " << this->time << "_\n";
    // this->myfile << "comRatio: " << this->comRatio << "_\n";
    // this->myfile << "J_pin1: \n";
    // this->myfile << this->J_pin1 << "\n";
    // this->myfile << "tau_msr: " << this->tau_msr.transpose() << "\n";
    // this->myfile << "r: " << this->r.transpose() << "\n";
    // this->myfile << "ddxc: " << this->ddxc.transpose() << "\n";
    // this->myfile << "dvc: " << this->dvc.transpose() << "\n";
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
  }

  void JointImpedanceController::complianceParamCallback(franka_example_controllers::impedance_controller_paramConfig &config, uint32_t /*level*/)
  {
    if (time == 0)
    {
      this->r[0] = config.Joint_r1;
      this->r[1] = config.Joint_r2;
      this->r[2] = config.Joint_r3;
      this->r[3] = config.Joint_r4;
      this->r[4] = config.Joint_r4;
      this->r[5] = config.Joint_r4;
      this->r[6] = config.Joint_r4;
      this->Kd = config.Joint_Kd * Eigen::MatrixXd::Identity(7, 7);
      this->Dd = config.Joint_Dd * Eigen::MatrixXd::Identity(7, 7);
      this->Md = config.Joint_Md * Eigen::MatrixXd::Identity(7, 7);
    }
    this->r_d[0] = config.Joint_r1;
    this->r_d[1] = config.Joint_r2;
    this->r_d[2] = config.Joint_r3;
    this->r_d[3] = config.Joint_r4;
    this->r_d[4] = config.Joint_r4;
    this->r_d[5] = config.Joint_r4;
    this->r_d[6] = config.Joint_r4;
    this->Kd_d = config.Joint_Kd * Eigen::MatrixXd::Identity(7, 7);
    this->Dd_d = config.Joint_Dd * Eigen::MatrixXd::Identity(7, 7);
    this->Md_d = config.Joint_Md * Eigen::MatrixXd::Identity(7, 7);
  }

  void JointImpedanceController::controllerParamRenew()
  {
    this->r = filter_params * this->r_d + (1.0 - filter_params) * this->r;
    this->Kd = filter_params * this->Kd_d + (1.0 - filter_params) * this->Kd;
    this->Dd = filter_params * this->Dd_d + (1.0 - filter_params) * this->Dd;
    this->Md = filter_params * this->Md_d + (1.0 - filter_params) * this->Md;
  }

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceController,
                       controller_interface::ControllerBase)
