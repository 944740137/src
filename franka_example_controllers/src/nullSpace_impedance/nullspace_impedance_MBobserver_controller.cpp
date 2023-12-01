// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/pinocchino_interactive.h>

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

extern PandaDynLibManager *pPandaDynLibManager;

namespace franka_example_controllers
{
  bool NullSpaceImpedanceMBObserverController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "[------------------] init1:NullSpaceImpedanceMBObserverController" << std::endl;
    std::cout << "[------------------] init2:NullSpaceImpedanceMBObserverController" << std::endl;

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
      ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "NullSpaceImpedanceMBObserverController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("NullSpaceImpedanceMBObserverController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&NullSpaceImpedanceMBObserverController::complianceParamCallback, this, _1, _2));

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

  void NullSpaceImpedanceMBObserverController::starting(const ros::Time & /*time*/)
  {
    std::cout << "[------------------] start1:NullSpaceImpedanceMBObserverController\n";
    std::cout << "[------------------] start2:NullSpaceImpedanceMBObserverController\n";
    std::cout << "[------------------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------------------] 编译时刻:" << __TIME__ << "\n";
    std::cout << "[------------------] 动量观测器" << std::endl;

    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    this->T0 = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())); // 齐次变换矩阵
    this->q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    this->X0 << Eigen::Vector3d(this->T0.translation()), Eigen::Quaterniond(this->T0.rotation()).toRotationMatrix().eulerAngles(2, 1, 0);

    this->task2_q_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    if (!this->myfile.is_open())
    {
      this->myfile.open("/home/wd/log/franka/master/NullSpaceImpedanceMBObserverController.txt");
      this->myfile << "NullSpaceImpedanceMBObserverController" << std::endl;
      this->myfile << "编译日期:" << __DATE__ << "\n";
      this->myfile << "编译时刻:" << __TIME__ << "\n";
      this->myfile << "动量观测器" << std::endl;
    }
  }

  void NullSpaceImpedanceMBObserverController::threeDOF(Eigen::Matrix<double, 6, 1> &X_d,
                                                        Eigen::Matrix<double, 6, 1> &dX_d,
                                                        Eigen::Matrix<double, 6, 1> &ddX_d,
                                                        Eigen::Matrix<double, 6, 1> &Xerror,
                                                        Eigen::Matrix<double, 6, 1> &dXerror,
                                                        const ros::Duration &t)
  {
    static Eigen::Matrix<double, 7, 1> tmp1 = Eigen::MatrixXd::Zero(7, 1); // 待积分
    static Eigen::Matrix<double, 7, 1> tmp2 = Eigen::MatrixXd::Zero(7, 1); // 待积分
    static Eigen::Matrix<double, 7, 1> tmpI = Eigen::MatrixXd::Zero(7, 1); // 累加

    // this->myfile << "tmp1: " << tmp1.transpose() << "\n";
    // this->myfile << "tmp2: " << tmp2.transpose() << "\n";
    // this->myfile << "tmpI: " << tmpI.transpose() << "\n";
    this->myfile << "------------------" << std::endl;

    // 伪逆矩阵计算
    Eigen::MatrixXd J_pinv;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    weightedPseudoInverse(J, J_pinv, this->M);

    // Z
    this->J1 = this->J.block(0, 0, 3, 7);
    this->dJ1 = dJ.block(0, 0, 3, 7);
    this->J1_pinv = J_pinv.block(0, 0, 7, 3);
    this->Jm = this->J.block(0, 1, 3, 3);
    this->Ja = this->J.block(0, 0, 3, 1);
    this->Jb = this->J.block(0, 4, 3, 3);
    this->Za << 1, 0, 0, 0;
    this->Zb << 0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    this->Zm = this->Jm.inverse() * (-this->Ja * this->Za - this->Jb * this->Zb);
    this->Z << this->Za, this->Zm, this->Zb;
    this->Z_inv = (this->Z.transpose() * this->M * this->Z).inverse() * this->Z.transpose() * this->M;
    double r2 = 0.01;
    if (time == 0)
    {
      this->S2 = this->Z_inv;
      this->S2_dot.setZero();
      this->dZ_inv.setZero();
    }
    else
    {
      /* dZ_inv= */ this->S2_dot = (this->Z_inv - this->S2) / r2;
      this->S2 = this->S2_dot * t.toSec() + this->S2;
    }
    this->dZ_inv = this->S2_dot;

    //
    bool ifPDplus = false;
    if (ifPDplus && time == 0)
      std::cout << "is PDplus" << std::endl;
    if (!ifPDplus && time == 0)
      std::cout << "no PDplus" << std::endl;

    // 命令加速度与输入力矩
    dx = dX.block(0, 0, 3, 1);
    Lambdax_inv = (J1 * M.inverse() * J1.transpose());
    ux = (J1_pinv.transpose() * C_pin - Lambdax_inv.inverse() * dJ1) * J1_pinv;

    Lambdav = (Z.transpose() * M * Z);
    uv = (Z.transpose() * C_pin - Lambdav * dZ_inv) * Z;
    v = Z_inv * dq;

    if (ifPDplus)
    {
      ddxc = Lambdax_inv * ((ux + PD_D) * dXerror.block(0, 0, 3, 1) + PD_K * Xerror.block(0, 0, 3, 1) + J1_pinv.transpose() * r);
    }
    else
    {
      ddxc = ddX_d.block(0, 0, 3, 1) + Kv * dXerror.block(0, 0, 3, 1) + Kp * Xerror.block(0, 0, 3, 1) /* + Lambdax_inv * J1_pinv.transpose() * r */;
    }

    dvc = Lambdav.inverse() * ((uv + Bv) * (-v) + Z.transpose() * Kd * (task2_q_d - q));
    ddqc = J1_pinv * (ddxc - dJ1 * dq) + Z * (dvc - dZ_inv * dq);

    // obsever
    tmp2 = tau_d + C_pin.transpose() * dq + r;
    tmpI = tmpI + 0.5 * (tmp1 + tmp2) * t.toSec();
    tmp1 = tmp2;
    r = KI * (M * dq - tmpI);
    tau_msr = -r;
    F_msr = J_pinv.transpose() * tau_msr;
    this->tau_d << M * ddqc + c;

    // 目标位置，控制参数更新
    controllerParamRenew();

    // 记录数据
    this->time++;
  }

  void NullSpaceImpedanceMBObserverController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    upDateParam();
    recordData();
    double r1 = 0.1;
    double r3 = 0.2;
    if (time == 0)
    {
      this->S1 = this->J;
      this->S3 = this->dq;

      this->S1_dot.setZero();
      this->S3_dot.setZero();

      this->dJ.setZero();
    }
    else
    {
      /* dJ= */ this->S1_dot = (this->J - this->S1) / r1;
      this->S1 = this->S1_dot * t.toSec() + this->S1;

      /* ddq= */ this->S3_dot = (this->dq - this->S3) / r3;
      this->S3 = this->S3_dot * t.toSec() + this->S3;
    }
    this->dJ = S1_dot;
    // this->dq = S3;

    // 轨迹和误差  3轨迹最差
    static Eigen::Matrix<double, 6, 1> X_d, dX_d, ddX_d, Xerror, dXerror;
    // cartesianTrajectoryXZ1(time / 1000, 0.6, 0.5, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);
    // cartesianTrajectoryXZ2(time / 1000, 0.6, 0.5, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);
    // cartesianTrajectoryXZ3(time / 1000, 0.6, 0.8, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);
    cartesianTrajectory0(time / 1000, 0.8, 0.5, this->T, this->T0, this->X0, this->X, this->dX, X_d, dX_d, ddX_d, Xerror, dXerror);

    threeDOF(X_d, dX_d, ddX_d, Xerror, dXerror, t);

    // 画图
    for (int i = 0; i < 7; i++)
    {
      // this->param_debug.tau_d[i] = this->tau_d[i];
      // this->param_debug.tau_J[i] = this->tau_J[i];
      // this->param_debug.tau_J_d[i] = this->tau_J_d[i];
      // this->param_debug.tau_msr[i] = this->tau_msr[i];
      // this->param_debug.dtau_msr[i] = this->dtau_msr[i];
      // this->param_debug.tau_ext[i] = this->tau_ext[i];
      // this->param_debug.S3[i] = this->S3[i];
      // this->param_debug.dq[i] = this->dq[i];
      // if (i == 6)
      //   break;
      // this->param_debug.F_ext0[i] = this->F_ext0[i];
      // this->param_debug.F_extK[i] = this->F_extK[i];
      // this->param_debug.F_msr[i] = this->F_msr[i];
      // this->param_debug.X[i] = this->X[i];
      // this->param_debug.X_d[i] = X_d[i];
      // this->param_debug.dX[i] = this->dX[i];
      // this->param_debug.dX_d[i] = dX_d[i];
      // this->param_debug.Xerror[i] = Xerror[i];
      // this->param_debug.dXerror[i] = dXerror[i];
      // if (i >= 3)
      //   continue;
    }
    this->paramForDebug.publish(this->param_debug);

    // 平滑命令
    tau_d << saturateTorqueRate(this->tau_d, this->tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(this->tau_d(i)); // 关节句柄设置力矩命令
    }
  }

  Eigen::Matrix<double, 7, 1> NullSpaceImpedanceMBObserverController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }

  void NullSpaceImpedanceMBObserverController::upDateParam()
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

  void NullSpaceImpedanceMBObserverController::recordData()
  {
    this->myfile << "time: " << this->time << "_\n";
    this->myfile << "comRatio: " << this->comRatio << "_\n";
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

  void NullSpaceImpedanceMBObserverController::complianceParamCallback(franka_example_controllers::nullspace_impedance_controller_paramConfig &config, uint32_t /*level*/)
  {
    if (time == 0)
    {
      Kp << config.Kp * Eigen::MatrixXd::Identity(3, 3);
      Kv << config.Kv * Eigen::MatrixXd::Identity(3, 3);
      KI << config.KI * Eigen::MatrixXd::Identity(7, 7);

      Bv << config.Bv * Eigen::MatrixXd::Identity(4, 4);
      Kd << config.Kd * Eigen::MatrixXd::Identity(7, 7);

      PD_D << config.mbPD_D * Eigen::MatrixXd::Identity(3, 3);
      PD_K << config.mbPD_K * Eigen::MatrixXd::Identity(3, 3);
    }
    Kp_d << config.Kp * Eigen::MatrixXd::Identity(3, 3);
    Kv_d << config.Kv * Eigen::MatrixXd::Identity(3, 3);
    KI_d << config.KI * Eigen::MatrixXd::Identity(7, 7);

    Bv_d << config.Bv * Eigen::MatrixXd::Identity(4, 4);
    Kd_d << config.Kd * Eigen::MatrixXd::Identity(7, 7);

    PD_D_d << config.mbPD_D * Eigen::MatrixXd::Identity(3, 3);
    PD_K_d << config.mbPD_K * Eigen::MatrixXd::Identity(3, 3);
  }

  void NullSpaceImpedanceMBObserverController::controllerParamRenew()
  {
    Kp = filter_params * Kp_d + (1.0 - filter_params) * Kp;
    Kv = filter_params * Kv_d + (1.0 - filter_params) * Kv;
    KI = filter_params * KI_d + (1.0 - filter_params) * KI;

    Bv = filter_params * Bv_d + (1.0 - filter_params) * Bv;
    Kd = filter_params * Kd_d + (1.0 - filter_params) * Kd;

    PD_D = filter_params * PD_D_d + (1.0 - filter_params) * PD_D;
    PD_K = filter_params * PD_K_d + (1.0 - filter_params) * PD_K;
  }

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::NullSpaceImpedanceMBObserverController,
                       controller_interface::ControllerBase)
