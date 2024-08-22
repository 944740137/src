// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/pinocchino_interactive.h>

#include <franka_example_controllers/admittance_dyn_controller.h>

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
  AdmittanceDYNController::AdmittanceDYNController()
  {
    // std::cout << "[------------------] AdmittanceController 构造" << std::endl;
  }

  AdmittanceDYNController::~AdmittanceDYNController()
  {
    std::cout << "[------------------] 关闭传感器" << std::endl;
    hps->ftc_close();
  }
  bool AdmittanceDYNController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "[------------------] init1:AdmittanceDYNController" << std::endl;
    std::cout << "[------------------] init2:AdmittanceDYNController" << std::endl;

    // 皮诺曹计算
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

    // 参数服务器获取机器人类型
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("AdmittanceDYNController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("AdmittanceDYNController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("AdmittanceDYNController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("AdmittanceDYNController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("AdmittanceDYNController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("AdmittanceDYNController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("AdmittanceDYNController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("AdmittanceDYNController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // 动态参数
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::admittance_controller_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&AdmittanceDYNController::complianceParamCallback, this, _1, _2));

    // 发布数据
    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    double ft[6] = {0};
    this->hps = new hps_sensor();
    this->isConnect = this->hps->connect_sensor(8080, "172.16.0.3");

    return true;
  }

  void AdmittanceDYNController::starting(const ros::Time & /*time*/)
  {
    std::cout << "[------------------] start1:NocontactImpedance\n";
    std::cout << "[------------------] start2:NocontactImpedance\n";
    std::cout << "[------------------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------------------] 编译时刻:" << __TIME__ << "\n";
    std::cout << "[------------------] 非接触阻抗" << std::endl;

    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    this->T0 = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())); // 齐次变换矩阵
    this->q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
    this->X0 << Eigen::Vector3d(this->T0.translation()), Eigen::Quaterniond(this->T0.rotation()).toRotationMatrix().eulerAngles(2, 1, 0);
    this->pos0 = this->T0.translation();
    this->ori0 = Eigen::Quaterniond(this->T0.rotation());
    this->task2_q_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());

    if (!this->myfile.is_open())
    {
      this->myfile.open("/home/wd/log/franka/master/AdmittanceDYNController.txt");
      this->myfile << "AdmittanceDYNController" << std::endl;
      this->myfile << "编译日期:" << __DATE__ << "\n";
      this->myfile << "编译时刻:" << __TIME__ << "\n";
      this->myfile << "导纳动力学控制" << std::endl;
    }
    this->pos_a = this->pos0;
    this->dpos_a = Eigen::MatrixXd::Zero(3, 1);
    this->ddpos_a = Eigen::MatrixXd::Zero(3, 1);
    task2_K = 30 * task2_K;
  }

  void AdmittanceDYNController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    upDateParam();
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
    static Eigen::Matrix<double, 3, 1> pos_d, dpos_d, ddpos_d, pos_error, dpos_error;
    static Eigen::Matrix<double, 3, 1> ori_d, dori_d, ddori_d, ori_error, dori_error;
    static Eigen::Matrix<double, 3, 1> pos_error_old = Eigen::MatrixXd::Zero(3, 1);
    static Eigen::Matrix<double, 3, 1> ori_error_old = Eigen::MatrixXd::Zero(3, 1);
    static Eigen::Matrix<double, 3, 1> Ipos_error = Eigen::MatrixXd::Zero(3, 1);
    static Eigen::Matrix<double, 3, 1> Iori_error = Eigen::MatrixXd::Zero(3, 1);
    // pos
    cartesianPosTrajectory0(time / 1000, 0.6, 0.5, this->pos0, this->pos, this->dpos, pos_d, dpos_d, ddpos_d, pos_error, dpos_error);
    // cartesianPosTrajectoryX1(time / 1000, 0.2, 0.5, this->pos0, this->pos, this->dpos, pos_d, dpos_d, ddpos_d, pos_error, dpos_error);

    // ori
    ddori_d = Eigen::MatrixXd::Zero(3, 1);
    if (this->ori0.coeffs().dot(this->ori.coeffs()) < 0.0)
      this->ori.coeffs() << -this->ori.coeffs();
    Eigen::Quaterniond error_quaternion(this->ori.inverse() * this->ori0);
    ori_error << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    ori_error << this->T.rotation() * ori_error.tail(3);
    dori_error = -this->dX.tail(3);
    Iori_error = Iori_error + 0.5 * (ori_error + ori_error_old) * t.toSec();
    ori_error_old = ori_error;

    if (this->isConnect)
    {
      hps->get_ftData(this->ft);
      for (int i = 0; i < 6; i++)
      {
        // if (this->ft[i] > 0.4)
        this->ft_new[i] = this->ft[i]; // 换方向
        //   else
        //     this->ft_new[i] = 0;
      }
      // 变换力矢量的表示到基坐标系下
      this->ft_new[0] = this->ft[0] * std::sqrt(2) / 2 - this->ft[1] * std::sqrt(2) / 2;
      this->ft_new[1] = -(this->ft[0] * std::sqrt(2) / 2 + this->ft[1] * std::sqrt(2) / 2);
      this->ft_new[2] = -this->ft_new[2];
    }
    // 死区
    // for (int i = 0; i < 6; i++)
    // {
    //   if (std::fabs(this->ft_new[i]) < 0.4)
    //     this->ft_new[i] = 0;
    // }
    // BF-VIC

    // 非线性导纳

    this->ft_fil = ft_new;
    this->ddpos_a = -Md.inverse() * (-this->ft_fil.block(0, 0, 3, 1) - this->Kd * (pos_d - this->pos_a) - this->Dd * (dpos_d - this->dpos_a)) + ddpos_d;
    this->dpos_a = this->dpos_a + this->ddpos_a * 0.001;
    this->pos_a = this->pos_a + this->dpos_a * 0.001;
    static Eigen::Matrix<double, 3, 1> pos_ACerror, dpos_ACerror, Ipos_ACerror;
    static Eigen::Matrix<double, 3, 1> pos_ACerror_old = Eigen::MatrixXd::Zero(3, 1);

    // 导纳，得出xa，跟踪误差pos_error = xa-a
    pos_ACerror = this->pos_a - this->pos;
    dpos_ACerror = this->dpos_a - this->dpos;
    Ipos_ACerror = Ipos_ACerror + 0.5 * (pos_ACerror + pos_ACerror_old) * t.toSec();
    pos_ACerror_old = pos_ACerror;

    // 命令加速度与输入力
    this->xc1 = ddpos_d + (this->Kp_pos * pos_ACerror + this->Ki_pos * Ipos_ACerror + this->Kv_pos * dpos_ACerror);
    this->F_c.head(3) = this->Lambda.block(0, 0, 3, 3) * (this->xc1 - (this->dJ * this->dq).head(3));
    this->xc2 = ddori_d + (this->Kp_ori * ori_error + this->Ki_ori * Iori_error + this->Kv_ori * dori_error);
    this->F_c.tail(3) = this->Lambda.block(2, 2, 3, 3) * (this->xc2 - (this->dJ * this->dq).tail(3));

    this->tau_d = this->J.transpose() * this->F_c + this->N * (this->task2_K * (task2_q_d - this->q) + this->task2_D * -this->dq) + this->c;
    if (this->time == 0)
      std::cout << "导纳动力学控制" << std::endl;

    // 记录数据
    this->time++;
    recordData();
    // this->myfile << "pos_d: " << pos_d.transpose() << "_\n";

    // 画图
    for (int i = 0; i < 3; i++)
    {
      this->param_debug.pos_a[i] = this->pos_a[i];
      this->param_debug.dpos_a[i] = this->dpos_a[i];
      this->param_debug.ddpos_a[i] = this->ddpos_a[i];
      this->param_debug.pos[i] = this->pos[i];
      this->param_debug.pos_d[i] = pos_d[i];
      this->param_debug.ori[i] = this->ori.toRotationMatrix().eulerAngles(2, 1, 0)[i];
      this->param_debug.ori_d[i] = this->ori0.toRotationMatrix().eulerAngles(2, 1, 0)[i];
      this->param_debug.pos_error[i] = pos_error[i];
      this->param_debug.ori_error[i] = ori_error[i];
      this->param_debug.pos_ACerror[i] = pos_ACerror[i];
      this->param_debug.dpos_ACerror[i] = dpos_ACerror[i];
    }
    for (int i = 0; i < 6; i++)
    {
      this->param_debug.F_sensor[i] = this->ft_new[i];
      this->param_debug.F_sensor_fil[i] = this->ft_fil[i];
    }
    for (int i = 0; i < 7; i++)
    {
      this->param_debug.tau_d[i] = this->tau_d[i];
    }
    // 目标位置，控制参数更新
    controllerParamRenew();

    // 平滑命令
    tau_d << saturateTorqueRate(this->tau_d, this->tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(this->tau_d(i)); // 关节句柄设置力矩命令
    }

    this->paramForDebug.publish(this->param_debug);

    // 清零
    for (int i = 0; i < 6; i++)
      this->ft_new[i] = 0.0;
  }

  Eigen::Matrix<double, 7, 1> AdmittanceDYNController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }

  void AdmittanceDYNController::upDateParam()
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
    this->pos = this->T.translation();
    this->ori = Eigen::Quaterniond(this->T.rotation());
    this->dX = this->J * this->dq;
    this->dpos = this->dX.block(0, 0, 3, 1);
    this->dori = this->dX.block(0, 2, 3, 1);

    this->tau_ext = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_ext_hat_filtered.data());
    this->F_ext0 = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data());
    this->F_extK = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.K_F_ext_hat_K.data());

    this->comRatio = robot_state.control_command_success_rate;

    weightedPseudoInverse2<6, 7>(this->J, this->M, this->J_inv, this->Lambda);
    this->N = this->I - this->J.transpose() * this->J_inv.transpose();
    //
    pPandaDynLibManager->upDataModel(this->q);
    pPandaDynLibManager->computeKinData(this->J_pin, this->dJ_pin, this->q, this->dq);
    pPandaDynLibManager->computeDynData(this->M_pin, this->C_pin, this->G_pin, this->q, this->dq);
  }

  void AdmittanceDYNController::recordData()
  {
    // this->myfile << "time: " << this->time << "_\n";
    // this->myfile << "pos: " << this->pos.transpose() << "_\n";
    // this->myfile << "pos_a: " << this->pos_a.transpose()<< "_\n";
    // this->myfile << "I: \n";
    // this->myfile << this->J * this->J_inv << "\n";
    // this->myfile << "N: \n";
    // this->myfile << this->J * this->N << "\n";
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

  void AdmittanceDYNController::complianceParamCallback(franka_example_controllers::admittance_controller_paramConfig &config, uint32_t /*level*/)
  {
    if (this->time == 0)
    {
      this->Kp_pos = config.Kp_pos * Eigen::MatrixXd::Identity(3, 3);
      this->Kv_pos = config.Kv_pos * Eigen::MatrixXd::Identity(3, 3);
      this->Ki_pos = config.Ki_pos * Eigen::MatrixXd::Identity(3, 3);

      this->Kp_ori = config.Kp_ori * Eigen::MatrixXd::Identity(3, 3);
      this->Kv_ori = config.Kv_ori * Eigen::MatrixXd::Identity(3, 3);
      this->Ki_ori = config.Ki_ori * Eigen::MatrixXd::Identity(3, 3);

      this->Kd = config.AC_Kd * Eigen::MatrixXd::Identity(3, 3);
      this->Dd = config.AC_Dd * Eigen::MatrixXd::Identity(3, 3);
      this->Md = config.AC_Md * Eigen::MatrixXd::Identity(3, 3);
    }
    this->Kp_pos_d = config.Kp_pos * Eigen::MatrixXd::Identity(3, 3);
    this->Kv_pos_d = config.Kv_pos * Eigen::MatrixXd::Identity(3, 3);
    this->Ki_pos_d = config.Ki_pos * Eigen::MatrixXd::Identity(3, 3);

    this->Kp_ori_d = config.Kp_ori * Eigen::MatrixXd::Identity(3, 3);
    this->Kv_ori_d = config.Kv_ori * Eigen::MatrixXd::Identity(3, 3);
    this->Ki_ori_d = config.Ki_ori * Eigen::MatrixXd::Identity(3, 3);

    this->Kd_d = config.AC_Kd * Eigen::MatrixXd::Identity(3, 3);
    this->Dd_d = config.AC_Dd * Eigen::MatrixXd::Identity(3, 3);
    this->Md_d = config.AC_Md * Eigen::MatrixXd::Identity(3, 3);
  }

  void AdmittanceDYNController::controllerParamRenew()
  {
    this->Kp_pos = filter_params * this->Kp_pos_d + (1.0 - filter_params) * this->Kp_pos;
    this->Kv_pos = filter_params * this->Kv_pos_d + (1.0 - filter_params) * this->Kv_pos;
    this->Ki_pos = filter_params * this->Ki_pos_d + (1.0 - filter_params) * this->Ki_pos;

    this->Kp_ori = filter_params * this->Kp_ori_d + (1.0 - filter_params) * this->Kp_ori;
    this->Kv_ori = filter_params * this->Kv_ori_d + (1.0 - filter_params) * this->Kv_ori;
    this->Ki_ori = filter_params * this->Ki_pos_d + (1.0 - filter_params) * this->Ki_ori;

    this->Kd = filter_params * this->Kd_d + (1.0 - filter_params) * this->Kd;
    this->Dd = filter_params * this->Dd_d + (1.0 - filter_params) * this->Dd;
    this->Md = filter_params * this->Md_d + (1.0 - filter_params) * this->Md;
  }
} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::AdmittanceDYNController,
                       controller_interface::ControllerBase)
