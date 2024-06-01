// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/pinocchino_interactive.h>

#include <franka_example_controllers/nocontact_impedance_controller.h>

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
  bool NocontactImpedanceController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::cout << "[------------------] init1:NocontactImpedanceController" << std::endl;
    std::cout << "[------------------] init2:NocontactImpedanceController" << std::endl;

    // camera
    image_transport::ImageTransport image(node_handle);

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
      ROS_ERROR_STREAM("NocontactImpedanceController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("NocontactImpedanceController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
      return false;
    }

    // 运动学/动力学模型类：实例化
    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("NocontactImpedanceController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("NocontactImpedanceController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // 机器人完整状态类：实例化
    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("NocontactImpedanceController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("NocontactImpedanceController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    // 关节控制类（ROS自带）：实例化
    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("NocontactImpedanceController: Error getting effort joint interface from hardware");
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
        ROS_ERROR_STREAM("NocontactImpedanceController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // 动态参数
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_example_controllers::nocontact_impedance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&NocontactImpedanceController::complianceParamCallback, this, _1, _2));

    // 发布数据
    paramForDebug = node_handle.advertise<franka_example_controllers::paramForDebug>("paramForDebug", 20);

    // 深度图
    this->depthSub = image.subscribe("/camera/depth/image_rect_raw", 1, boost::bind(&NocontactImpedanceController::depthCameraCallback, this, _1));
    return true;
  }

  void NocontactImpedanceController::starting(const ros::Time & /*time*/)
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
      this->myfile.open("/home/wd/log/franka/master/NocontactImpedance.txt");
      this->myfile << "NocontactImpedance" << std::endl;
      this->myfile << "编译日期:" << __DATE__ << "\n";
      this->myfile << "编译时刻:" << __TIME__ << "\n";
      this->myfile << "非接触阻抗" << std::endl;
    }

    task2_K = 30 * task2_K;
  }

  void NocontactImpedanceController::update(const ros::Time & /*time*/, const ros::Duration &t)
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
    static Eigen::Matrix<double, 3, 1> Ipos_error = Eigen::MatrixXd::Zero(3, 1);
    static Eigen::Matrix<double, 3, 1> Iori_error = Eigen::MatrixXd::Zero(3, 1);
    cartesianPosTrajectoryX1(time / 1000, 0.6, 0.5, this->pos0, this->pos, this->dpos, pos_d, dpos_d, ddpos_d, pos_error, dpos_error);
    ddori_d = Eigen::MatrixXd::Zero(3, 1);
    if (this->ori0.coeffs().dot(this->ori.coeffs()) < 0.0)
      this->ori.coeffs() << -this->ori.coeffs();
    Eigen::Quaterniond error_quaternion(this->ori.inverse() * this->ori0);
    ori_error << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    ori_error << this->T.rotation() * ori_error.tail(3);
    dori_error = -this->dX.tail(3);

    // 命令加速度与输入力
    this->xc1 = ddpos_d + this->Md_pos.inverse() * (this->Kd_pos * pos_error + this->Dd_pos * dpos_error);
    this->F_c.head(3) = /* this->Lambda *  */ (this->xc1 /*  - this->dJ * this->dq */);

    this->xc2 = ddori_d + this->Md_ori.inverse() * (this->Kd_ori * ori_error + this->Dd_ori * dori_error);
    this->F_c.tail(3) = /* this->Lambda *  */ (this->xc2 /*  - this->dJ * this->dq */);

    this->tau_d = this->J.transpose() * this->F_c + this->N * (this->task2_K * (task2_q_d - this->q) + this->task2_D * -this->dq) + this->c;
    if (this->time == 0)
      std::cout << "非接触阻抗" << std::endl;

    // 记录数据
    this->time++;
    recordData();
    // this->myfile << "ori_error: " << ori_error.transpose() << "\n";
    // this->myfile << "F_c: " << this->F_c.transpose() << "\n";
    // this->myfile << "xc2: " << this->xc2.transpose() << "\n";

    // 画图
    for (int i = 0; i < 3; i++)
    {
      this->param_debug.pos[i] = this->pos[i];
      this->param_debug.pos_d[i] = pos_d[i];
      this->param_debug.ori[i] = this->ori.toRotationMatrix().eulerAngles(2, 1, 0)[i];
      this->param_debug.ori_d[i] = this->ori0.toRotationMatrix().eulerAngles(2, 1, 0)[i];
      this->param_debug.pos_error[i] = pos_error[i];
      this->param_debug.ori_error[i] = ori_error[i];
    }
    for (int i = 0; i < 7; i++)
    {
      this->param_debug.tau_d[i] = this->tau_d[i];
      if (i == 6)
        break;
      this->param_debug.F_ext0[i] = this->F_ext0[i];
      this->param_debug.F_extK[i] = this->F_extK[i];
    }
    this->param_debug.VirtualForce = this->VirtualForce;
    this->param_debug.depthDistance = this->depthDistance;
    // 目标位置，控制参数更新
    controllerParamRenew();

    // 平滑命令
    tau_d << saturateTorqueRate(this->tau_d, this->tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(this->tau_d(i)); // 关节句柄设置力矩命令
    }

    this->paramForDebug.publish(this->param_debug);
  }

  Eigen::Matrix<double, 7, 1> NocontactImpedanceController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max); // 6
    }
    return tau_d_saturated;
  }

  void NocontactImpedanceController::depthCameraCallback(const sensor_msgs::ImageConstPtr &depth_msg)
  {
    try
    {
      this->depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
    this->depth_ptr->image.copyTo(this->img);

    // 定义图像中心点
    cv::Point center(this->img.cols / 2, this->img.rows / 2);
    // 定义计算距离的范围
    cv::Rect RectRange(center.x - this->range.width / 2, center.y - this->range.height / 2, this->range.width, this->range.height);
    // std::cout << "this->img.cols" << this->img.cols << std::endl;
    // std::cout << "this->img.rows" << this->img.rows << std::endl;

    // 画出范围
    float distance_sum = 0;
    int effective_pixel = 0;
    for (int y = RectRange.y; y < RectRange.y + RectRange.height; y++)
    {
      for (int x = RectRange.x; x < RectRange.x + RectRange.width; x++)
      {
        // 不是0就有位置信息
        if (this->img.at<uint16_t>(y, x)) // 出现位置信息
        {
          distance_sum += 0.001 * this->img.at<uint16_t>(y, x);
          effective_pixel++;
        }
      }
    }
    float effective_distance = 0;
    if (effective_pixel == 0)
      effective_distance = 0.25; // m
    else
      effective_distance = distance_sum / effective_pixel;
    double filter_params = 0.5;
    static float effective_distance_new = 0;
    effective_distance_new = filter_params * effective_distance + (1.0 - filter_params) * effective_distance_new;
    this->depthDistance = effective_distance_new;
    // std::cout << "有效像素点：" << effective_pixel << std::endl; // 输出数据
    std::cout << "目标距离：" << effective_distance_new << "m" << std::endl;

    cv::rectangle(img, RectRange, cv::Scalar(255, 255, 255), 2, 8);
    cv::imshow("深度图", this->img);
    cv::waitKey(30);
  }

  void NocontactImpedanceController::upDateParam()
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

  void NocontactImpedanceController::recordData()
  {
    // this->myfile << "time: " << this->time << "_\n";
    // this->myfile << "comRatio: " << this->comRatio << "_\n";
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

  void NocontactImpedanceController::complianceParamCallback(franka_example_controllers::nocontact_impedance_paramConfig &config, uint32_t /*level*/)
  {
    if (this->time == 0)
    {
      this->Kd_pos = config.Cartesian_Kd_pos * Eigen::MatrixXd::Identity(3, 3);
      this->Dd_pos = 2.0 * sqrt(config.Cartesian_Kd_pos) * Eigen::MatrixXd::Identity(3, 3);
      this->Kd_ori = config.Cartesian_Kd_ori * Eigen::MatrixXd::Identity(3, 3);
      this->Dd_ori = 2.0 * sqrt(config.Cartesian_Kd_ori) * Eigen::MatrixXd::Identity(3, 3);
    }
    this->Kd_d_pos = config.Cartesian_Kd_pos * Eigen::MatrixXd::Identity(3, 3);
    this->Dd_d_pos = 2.0 * sqrt(config.Cartesian_Kd_pos) * Eigen::MatrixXd::Identity(3, 3);
    this->Kd_d_ori = config.Cartesian_Kd_ori * Eigen::MatrixXd::Identity(3, 3);
    this->Dd_d_ori = 2.0 * sqrt(config.Cartesian_Kd_ori) * Eigen::MatrixXd::Identity(3, 3);

    this->task2_K = config.Joint_Kd * Eigen::MatrixXd::Identity(7, 7);
    this->task2_D = 2.0 * sqrt(config.Joint_Kd) * Eigen::MatrixXd::Identity(7, 7);
  }

  void NocontactImpedanceController::controllerParamRenew()
  {
    this->Kd_pos = filter_params * this->Kd_d_pos + (1.0 - filter_params) * this->Kd_pos;
    this->Dd_pos = filter_params * this->Dd_d_pos + (1.0 - filter_params) * this->Dd_pos;
    this->Md_pos = filter_params * this->Md_d_pos + (1.0 - filter_params) * this->Md_pos;

    this->Kd_ori = filter_params * this->Kd_d_ori + (1.0 - filter_params) * this->Kd_ori;
    this->Dd_ori = filter_params * this->Dd_d_ori + (1.0 - filter_params) * this->Dd_ori;
    this->Md_ori = filter_params * this->Md_d_ori + (1.0 - filter_params) * this->Md_ori;
  }
} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::NocontactImpedanceController,
                       controller_interface::ControllerBase)
