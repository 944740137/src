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
    robotInit();
    std::cout << "--------------init1:MainController--------------" << std::endl;
    std::cout << "--------------init2:MainController--------------" << std::endl;

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
    std::cout << "--------------start1:MainController--------------" << std::endl;
    std::cout << "--------------start2:MainController--------------" << std::endl;
    std::cout << "------编译日期:" << __DATE__ << "------" << std::endl;
    std::cout << "------编译时刻:" << __TIME__ << "------" << std::endl;
    // 获取机器人初始状态
    franka::RobotState initial_state = state_handle_->getRobotState();
    // 获取当前关节位置
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initialMap(initial_state.q.data());

    // 控制参数赋初值
    int tmp = 1;
    Kp.setIdentity();
    Kp = tmp * Eigen::MatrixXd::Identity(7, 7);
    Kv.setIdentity();
    Kv = tmp * Eigen::MatrixXd::Identity(7, 7);
    K1.setIdentity();
    K1 = tmp * Eigen::MatrixXd::Identity(7, 7);
    K2.setIdentity();
    K2 = tmp * Eigen::MatrixXd::Identity(7, 7);

    q_d.setZero();
    dq_d.setZero();
    ddq_d.setZero();
    q_initial = q_initialMap;
    elapsed_time = ros::Duration(0.0);

    robotStart(q_initial, 1);
  }
  void MainController::update(const ros::Time & /*time*/, const ros::Duration &t)
  {
    // 记录数据
    if (firstUpdate)
    {
      std::cout << "--------------update1:MainController--------------" << std::endl;
      std::cout << "--------------update2:MainController--------------" << std::endl;
      // debug
      myfile.open("/home/wd/MainController.txt");
      myfile << "MainController\n"
             << std::endl;
      firstUpdate = false;
    }

    // 发布数据
    main_controller::paramForDebug param_debug;

    // 获取状态,C,M，q,dq的array类
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 7> g_array = model_handle_->getGravity();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // 将array类转成矩阵
    Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolisTermMap(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> inertiaMatrixMap(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> G(g_array.data());

    Eigen::Map<Eigen::Matrix<double, 7, 1>> qMap(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dqMap(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_dMap(robot_state.tau_J_d.data());
    Eigen::Matrix<double, 7, 1> coriolisTerm = coriolisTermMap;
    Eigen::Matrix<double, 7, 7> inertiaMatrix = inertiaMatrixMap;
    Eigen::Matrix<double, 7, 1> q = qMap;
    Eigen::Matrix<double, 7, 1> dq = dqMap;
    Eigen::Matrix<double, 7, 1> tau_J_d = tau_J_dMap;
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // 齐次变换矩阵
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.rotation());

    // 期望轨迹生成
    // elapsed_time += t;
    time++;
    double TPP = 0.4;  // Trajectory position parameters
    double TVP = 10.0; // Trajectory velocity parameters
    double delta_angle = M_PI / 4 * std::sin(M_PI / TVP * time / 1000.0) * TPP;
    double dot_delta_angle = M_PI / 4 * M_PI / TVP * (std::cos(M_PI / TVP * time / 1000.0)) * TPP;
    double ddot_delta_angle = -M_PI / 4 * M_PI / TVP * M_PI / TVP * (std::sin(M_PI / TVP * time / 1000.0)) * TPP;

    for (size_t i = 0; i < 7; ++i)
    {
      q_d[i] = q_initial[i] + delta_angle;
      dq_d[i] = dot_delta_angle;
      ddq_d[i] = ddot_delta_angle;
    }

    // 误差计算
    Eigen::Matrix<double, 7, 1> error;
    Eigen::Matrix<double, 7, 1> derror;
    Eigen::Matrix<double, 7, 1> r;
    Eigen::Matrix<double, 7, 1> dr;
    Eigen::Matrix<double, 7, 1> error2;
    error = q_d - q;
    derror = dq_d - dq;
    error2 = derror + K1 * error;
    r = dq_d + K1 * error;
    dr = ddq_d + K1 * derror;

    // 命令加速度与输入力矩
    Eigen::VectorXd qc(7), tau_d(7);
    qc = ddq_d + Kp * error + Kv * derror;
    tau_d << inertiaMatrix * (qc) + coriolisTerm; /* + G */

    Eigen::Matrix<double, 7, 1> tau_ddd;
    robotRun(q, dq, tau_J_d, position, orientation, transform, tau_ddd);

    // 平滑命令
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i)); // 关节句柄设置力矩命令
    }
    // debug

    myfile << "" << std::endl;
    myfile << "time: " << time << "_" << std::endl;
    myfile << "q0: " << q_initial.transpose() << std::endl;
    myfile << "q: " << q.transpose() << std::endl;
    myfile << "dq: " << dq.transpose() << std::endl;
    myfile << "q_d: " << q_d.transpose() << std::endl;
    myfile << "dq_d: " << dq_d.transpose() << std::endl;
    myfile << "ddq_d: " << ddq_d.transpose() << std::endl;
    myfile << "Position: " << position.transpose() << std::endl;
    myfile << "Orientation: " << std::endl;
    myfile << orientation.toRotationMatrix() << std::endl;

    myfile << "T: " << std::endl;
    myfile << Eigen::Matrix4d::Map(robot_state.O_T_EE.data()) << std::endl;
    myfile << "M: " << std::endl;
    myfile << inertiaMatrix << std::endl;
    myfile << "C: " << std::endl;
    myfile << coriolisTerm << std::endl;
    myfile << "G: " << std::endl;
    myfile << G << std::endl;

    myfile << "Kp: " << std::endl;
    myfile << Kv << std::endl;
    myfile << "Kv: " << std::endl;
    myfile << Kp << std::endl;

    myfile << "J: " << std::endl;
    myfile << J << std::endl;
    myfile << "qc: " << qc.transpose() << std::endl;

    myfile << "getTorque: " << tau_J_d.transpose() << std::endl;
    myfile << "tau_d: " << tau_d.transpose() << std::endl;
    myfile << "-------------------" << std::endl;

    pController->pubData(param_debug, pPanda);

    // plot
    for (int i = 0; i < 7; i++)
    {
      param_debug.jointError[i] = error[i];
      param_debug.q_d[i] = q_d[i];
      param_debug.q[i] = q[i];
      param_debug.tau_d[i] = tau_d[i];
    }
    paramForDebug.publish(param_debug);

    // 控制参数更新
    controllerParamRenew();
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
    K1_target = config.K1 * Eigen::MatrixXd::Identity(7, 7);
    K2_target = config.K2 * Eigen::MatrixXd::Identity(7, 7);
    Kp_target = config.Kp * Eigen::MatrixXd::Identity(7, 7);
    Kv_target = config.Kv * Eigen::MatrixXd::Identity(7, 7);

    pController->dynamicSetParameter(config);
  }
  void MainController::controllerParamRenew()
  {
    Kp = filter_params * Kp_target + (1.0 - filter_params) * Kp;
    Kv = filter_params * Kv_target + (1.0 - filter_params) * Kv;
    K1 = filter_params * K1_target + (1.0 - filter_params) * K1;
    K2 = filter_params * K2_target + (1.0 - filter_params) * K2;
  }

} // namespace main_controller

PLUGINLIB_EXPORT_CLASS(main_controller::MainController,
                       controller_interface::ControllerBase)
