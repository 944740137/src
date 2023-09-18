// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/pinocchino_interactive.h>

#include <franka_example_controllers/nullspace_impedance_EBobserver_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>


namespace franka_example_controllers
{

  bool NullSpaceImpedanceEBObserverController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "[------------------] init1:NullSpaceImpedanceEBObserverController" << std::endl;
    std::cout << "[------------------] init2:NullSpaceImpedanceEBObserverController" << std::endl;

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
      ROS_ERROR(
          "NullSpaceImpedanceEBObserverController: Invalid or no joint_names parameters provided, "
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
      ROS_ERROR_STREAM(
          "NullSpaceImpedanceEBObserverController: Exception getting model handle from interface: " << ex.what());
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
      ROS_ERROR_STREAM("NullSpaceImpedanceEBObserverController: Error getting effort joint interface from hardware");
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

    // 动态参数服务
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::nullspace_impedance_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&NullSpaceImpedanceEBObserverController::complianceParamCallback, this, _1, _2));

    // 发布者对象
    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    return true;
  }

  void NullSpaceImpedanceEBObserverController::starting(const ros::Time & /*time*/)
  {
    std::cout << "[------------------] start1:NullSpaceImpedanceEBObserverController\n";
    std::cout << "[------------------] start2:NullSpaceImpedanceEBObserverController\n";
    std::cout << "[------------------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------------------] 编译时刻:" << __TIME__ << "\n";
    std::cout << "[------------------] 误差观测器" << std::endl;

    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    this->T0 = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())); // 齐次变换矩阵
    this->q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    this->X0 << Eigen::Vector3d(this->T0.translation()), Eigen::Quaterniond(this->T0.rotation()).toRotationMatrix().eulerAngles(2, 1, 0);

    this->task2_q_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());

    this->myfile.open("/home/wd/log/franka/nullSpace/NullSpaceImpedanceEBObserverController.txt");
    this->myfile << "NullSpaceImpedanceEBObserverController" << std::endl;
    this->myfile << "编译日期:" << __DATE__ << "\n";
    this->myfile << "编译时刻:" << __TIME__ << "\n";
    this->myfile << "误差观测器" << std::endl;
  }

  void NullSpaceImpedanceEBObserverController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {

    upDateParam();
    recordData();

    double r1 = 0.1;
    if (time == 0)
    {
      this->S1 = this->J;
      this->S1_dot.setZero();
      this->dJ.setZero();
    }
    else
    {
      /* dJ= */ this->S1_dot = (this->J - this->S1) / r1;
      this->S1 = this->S1_dot * t.toSec() + this->S1;
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

    bool ifPDplus = true; //
    double r = 0.01;

    // 命令加速度与输入力矩
    dx = dX.block(0, 0, 3, 1);
    Lambdax_inv = (J1 * M.inverse() * J1.transpose());
    ux = (J1_pinv.transpose() * C_pin - Lambdax_inv.inverse() * dJ1) * J1_pinv;
    s = dXerror.block(0, 0, 3, 1) + P * Xerror.block(0, 0, 3, 1);

    tau_msr = tau_msr + dtau_msr * t.toSec();

    if (ifPDplus)
      dtau_msr = -Gamma_inv * J1_pinv * (dXerror.block(0, 0, 3, 1) + r * (1 / (1 + Xerror.block(0, 0, 3, 1).norm())) * Xerror.block(0, 0, 3, 1));
    else
      dtau_msr = -Gamma_inv * J1_pinv * s;

    F_msr = J_pinv.transpose() * tau_msr;

    Lambdav = (Z.transpose() * M * Z);
    uv = (Z.transpose() * C_pin - Lambdav * dZ_inv) * Z;
    v = Z_inv * dq;

    if (ifPDplus)
      ddxc = ddX_d.block(0, 0, 3, 1) + Lambdax_inv * ((ux + PD_D) * dXerror.block(0, 0, 3, 1) + PD_K * Xerror.block(0, 0, 3, 1) + J1_pinv.transpose() * tau_msr);
    else
      ddxc = ddX_d.block(0, 0, 3, 1) + P * dXerror.block(0, 0, 3, 1) + Lambdax_inv * ((ux + K) * s + J1_pinv.transpose() * tau_msr);

    dvc = Lambdav.inverse() * ((uv + Bv) * (-v) + Z.transpose() * Kd * (task2_q_d - q));
    ddqc = J1_pinv * (ddxc - dJ1 * dq) + Z * (dvc - dZ_inv * dq);

    // this->tau_task = J1.transpose() * (Lambdax_inv.inverse() * (-P * dx - dJ1 * dq) + (ux + K) * s /* + J1_pinv.transpose() * tau_msr */);
    // this->tau_null = Z_inv.transpose() * (Lambdav * dZ_inv * dq + (uv + Bv) * v + Z.transpose() * Kd * (q - task2_q_d));

    this->tau_d << M * ddqc + c;
    // this->tau_d << tau_task /* + tau_null */ + c;

    // 记录数据
    this->time++;
    this->myfile << "dXerror: " << dXerror.transpose() << "\n";
    this->myfile << "Xerror: " << Xerror.transpose() << "\n";
    this->myfile << "Xerror.block(0, 0, 3, 1).norm(): " << Xerror.block(0, 0, 3, 1).norm() << "\n";

    // 画图
    for (int i = 0; i < 7; i++)
    {
      this->param_debug.tau_d[i] = this->tau_d[i];
      this->param_debug.tau_msr[i] = this->tau_msr[i];
      this->param_debug.dtau_msr[i] = this->dtau_msr[i];
      if (i == 6)
        break;
      this->param_debug.F_msr[i] = this->F_msr[i];
      this->param_debug.X[i] = this->X[i];
      this->param_debug.X_d[i] = X_d[i];
      this->param_debug.dX[i] = this->dX[i];
      this->param_debug.dX_d[i] = dX_d[i];
      this->param_debug.Xerror[i] = Xerror[i];
      this->param_debug.dXerror[i] = dXerror[i];
      if (i >= 3)
        continue;
      this->param_debug.S[i] = this->s[i];
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

  void NullSpaceImpedanceEBObserverController::upDateParam()
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

    // pinInteractive->forwardKinematics(this->q);
    // pinInteractive->updateFramePlacements();

    // pinInteractive->computeJointJacobians(this->J_pin, this->q);
    // pinInteractive->computeCoriolisMatrix(this->C_pin, this->q, this->dq);
  }

  void NullSpaceImpedanceEBObserverController::recordData()
  {
    // this->myfile << "time: " << this->time << "_\n";

    // this->myfile << "tau_d: " << this->tau_d.transpose() << "\n";
    // this->myfile << "q: " << this->q.transpose() << "\n";
    // this->myfile << "dq: " << this->dq.transpose() << "\n";
    // this->myfile << "X: " << this->X.transpose() << "\n";
    // this->myfile << "dX: " << this->dX.transpose() << "\n";

    // this->myfile << "R: \n";
    // this->myfile << this->T.rotation() << "\n";

    // this->myfile << "J_pin: \n";
    // this->myfile << this->J_pin << "\n";
    // this->myfile << "J: \n";
    // this->myfile << this->J << "\n";
    // this->myfile << "J1: \n";
    // this->myfile << this->J1 << "\n";
    // this->myfile << "J1_pinv: \n";
    // this->myfile << this->J1_pinv << "\n";

    // this->myfile << "dJ: \n";
    // this->myfile << this->dJ << "\n";
    // this->myfile << "dJ1: \n";
    // this->myfile << this->dJ1 << "\n";
    // this->myfile << "M: \n";
    // this->myfile << this->M << "\n";
    // this->myfile << "c: \n";
    // this->myfile << this->c.transpose() << "\n";
    // this->myfile << "C_pin*dq: \n";
    // this->myfile << (this->C_pin * this->dq).transpose() << "\n";
    // this->myfile << "G: \n";
    // this->myfile << this->G.transpose() << "\n";

    // this->myfile << "Z: \n";
    // this->myfile << this->Z << "\n";
    // this->myfile << "Z_inv: \n";
    // this->myfile << this->Z_inv << "\n";
    // this->myfile << "dZ_inv: \n";
    // this->myfile << this->dZ_inv << "\n";
    // this->myfile << "Lambdax_inv: \n";
    // this->myfile << this->Lambdax_inv << "\n";
    // this->myfile << "Lambdav: \n";
    // this->myfile << this->Lambdav << "\n";
    // this->myfile << "ux: \n";
    // this->myfile << this->ux << "\n";
    // this->myfile << "uv: \n";
    // this->myfile << this->uv << "\n";

    this->myfile << "tau_msr: " << this->tau_msr.transpose() << "\n";
    this->myfile << "dtau_msr: " << this->dtau_msr.transpose() << "\n";
    this->myfile << "ddxc: " << this->ddxc.transpose() << "\n";
    this->myfile << "dvc: " << this->dvc.transpose() << "\n";
    // this->myfile << "ddqc: " << this->ddqc.transpose() << "\n";

    // this->myfile << "task2_q_d: " << this->task2_q_d.transpose() << "\n";
    // this->myfile << "P: \n";
    // this->myfile << this->P << "\n";
    // this->myfile << "K: \n";
    // this->myfile << this->K << "\n";
    // this->myfile << "P_d: \n";
    // this->myfile << this->P_d << "\n";
    // this->myfile << "K_d: \n";
    // this->myfile << this->K_d << "\n";

    // this->myfile << "Bv: \n";
    // this->myfile << this->Bv << "\n";
    // this->myfile << "Kd: \n";
    // this->myfile << this->Kd << "\n";
    // this->myfile << "Bv_d: \n";
    // this->myfile << this->Bv_d << "\n";
    // this->myfile << "Kd_d: \n";
    // this->myfile << this->Kd_d << "\n";

    // this->myfile << "Gamma_inv: \n";
    // this->myfile << this->Gamma_inv << "\n";
    // this->myfile << "Gamma_inv_d: \n";
    // this->myfile << this->Gamma_inv_d << "\n";

    this->myfile << "------------------" << std::endl;
  }

  void NullSpaceImpedanceEBObserverController::complianceParamCallback(franka_example_controllers::nullspace_impedance_controller_paramConfig &config, uint32_t /*level*/)
  {
    if (time == 0)
    {
      P << config.P * Eigen::MatrixXd::Identity(3, 3);
      K << config.K * Eigen::MatrixXd::Identity(3, 3);

      PD_D << config.ebPD_D * Eigen::MatrixXd::Identity(3, 3);
      PD_K << config.ebPD_K * Eigen::MatrixXd::Identity(3, 3);

      Bv << config.Bv * Eigen::MatrixXd::Identity(4, 4);
      Kd << config.Kd * Eigen::MatrixXd::Identity(7, 7);

      Gamma_inv << config.Gamma_inv * Eigen::MatrixXd::Identity(7, 7);
    }
    PD_D_d << config.ebPD_D * Eigen::MatrixXd::Identity(3, 3);
    PD_K_d << config.ebPD_K * Eigen::MatrixXd::Identity(3, 3);

    P_d << config.P * Eigen::MatrixXd::Identity(3, 3);
    K_d << config.K * Eigen::MatrixXd::Identity(3, 3);

    Bv_d << config.Bv * Eigen::MatrixXd::Identity(4, 4);
    Kd_d << config.Kd * Eigen::MatrixXd::Identity(7, 7);

    Gamma_inv_d << config.Gamma_inv * Eigen::MatrixXd::Identity(7, 7);
  }
  void NullSpaceImpedanceEBObserverController::controllerParamRenew()
  {
    P = filter_params * P_d + (1.0 - filter_params) * P;
    K = filter_params * K_d + (1.0 - filter_params) * K;

    Bv = filter_params * Bv_d + (1.0 - filter_params) * Bv;
    Kd = filter_params * Kd_d + (1.0 - filter_params) * Kd;

    Gamma_inv = filter_params * Gamma_inv_d + (1.0 - filter_params) * Gamma_inv;
  }

  Eigen::Matrix<double, 7, 1> NullSpaceImpedanceEBObserverController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
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

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::NullSpaceImpedanceEBObserverController,
                       controller_interface::ControllerBase)
