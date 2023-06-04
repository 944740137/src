#include "pandaController.h"

void my_controller::ComputedTorqueMethod::dynamicSetParameter(panda_controller::panda_controller_paramConfig &config)
{
    std::array<double, 7> Kp_values = {config.Kp1, config.Kp2, config.Kp3, config.Kp4, config.Kp5, config.Kp6, config.Kp7};
    std::array<double, 7> Kv_values = {config.Kv1, config.Kv2, config.Kv3, config.Kv4, config.Kv5, config.Kv6, config.Kv7};

    if (this->time == 0)
    {
        for (int i = 0; i < 7; i++)
        {
            Kp(i, i) = Kp_values[i];
            Kv(i, i) = Kv_values[i];
        }
    }
    for (int i = 0; i < 7; i++)
    {
        Kp_d(i, i) = Kp_values[i];
        Kv_d(i, i) = Kv_values[i];
    }
}
void my_controller::ComputedTorqueMethod::controllerParamRenew()
{
    Kv = this->filterParams * Kv_d + (1.0 - this->filterParams) * Kv;
    Kp = this->filterParams * Kp_d + (1.0 - this->filterParams) * Kp;
}
bool my_controller::ComputedTorqueMethod::setControllerLaw(my_robot::Robot<DIM> *robot, Eigen::Matrix<double, DIM, 1> &tau_d_in)
{
    this->qc = this->ddq_d + Kp * this->jointError + Kv * this->djointError;
    this->tau_d << robot->getM() * (this->qc) + robot->getC() * robot->getdq() /* + G */;
    tau_d_in = this->tau_d;
    return true;
}
void my_controller::ComputedTorqueMethod::calDesire(my_robot::Robot<DIM> *robot)
{
    double TPP = 0.3;  // Trajectory position parameters
    double TVP = 10.0; // Trajectory velocity parameters
    double delta_angle = M_PI / 4 * std::sin(M_PI / TVP * this->time / 1000.0) * TPP;
    double dot_delta_angle = M_PI / 4 * M_PI / TVP * (std::cos(M_PI / TVP * this->time / 1000.0)) * TPP;
    double ddot_delta_angle = -M_PI / 4 * M_PI / TVP * M_PI / TVP * (std::sin(M_PI / TVP * this->time / 1000.0)) * TPP;

    for (size_t i = 0; i < 7; ++i)
    {
        this->q_d[i] = robot->getq0()[i] + delta_angle;
        this->dq_d[i] = dot_delta_angle;
        this->ddq_d[i] = ddot_delta_angle;
    }
}
void my_controller::ComputedTorqueMethod::recordData(my_robot::Robot<DIM> *robot)
{
    if (this->recordPeriod == 0)
        return;
    if (this->time == 1)
    {
        this->myfile.open("/home/wd/ComputedTorqueMethod.txt");
        this->myfile << "ComputedTorqueMethod" << std::endl;
        this->myfile << "--------程序编译日期:" << __DATE__ << "--------" << std::endl;
        this->myfile << "--------程序编译时刻:" << __TIME__ << "--------" << std::endl;
    }

    this->myfile << "time: " << this->time << "_" << std::endl;
    this->myfile << "q0: " << robot->getq0().transpose() << std::endl;
    this->myfile << "q: " << robot->getq().transpose() << std::endl;
    this->myfile << "dq: " << robot->getdq().transpose() << std::endl;
    this->myfile << "q_d: " << this->q_d.transpose() << std::endl;
    this->myfile << "dq_d: " << this->dq_d.transpose() << std::endl;
    this->myfile << "ddq_d: " << this->ddq_d.transpose() << std::endl;
    this->myfile << "Position: " << robot->getPosition().transpose() << std::endl;

    this->myfile << "Orientation: " << std::endl;
    this->myfile << robot->getOrientation().toRotationMatrix() << std::endl;

    this->myfile << "T: " << std::endl;
    this->myfile << robot->getT().matrix() << std::endl;
    this->myfile << "M: " << std::endl;
    this->myfile << robot->getM() << std::endl;
    this->myfile << "C: " << std::endl;
    this->myfile << robot->getC() * robot->getdq() << std::endl;
    this->myfile << "G: " << std::endl;
    this->myfile << robot->getG() << std::endl;

    this->myfile << "Kp: " << std::endl;
    this->myfile << Kv << std::endl;
    this->myfile << "Kv: " << std::endl;
    this->myfile << Kp << std::endl;

    this->myfile << "J: " << std::endl;
    this->myfile << robot->getJ() << std::endl;
    this->myfile << "qc: " << this->qc.transpose() << std::endl;

    this->myfile << "getTorque: " << robot->getTorque().transpose() << std::endl;
    this->myfile << "tau_d: " << this->tau_d.transpose() << std::endl;
    this->myfile << "-------------------" << std::endl;
}
void my_controller::ComputedTorqueMethod::pubData(panda_controller::paramForDebug &param_debug, my_robot::Robot<DIM> *robot)
{
    for (int i = 0; i < DIM; i++)
    {
        param_debug.jointError[i] = this->jointError[i];
        param_debug.q[i] = robot->getq()[i];
        param_debug.q_d[i] = this->q_d[i];
        param_debug.dq[i] = robot->getdq()[i];
        param_debug.dq_d[i] = this->dq_d[i];
        param_debug.tau_d[i] = this->tau_d[i];
    }
    for (int i = 0; i < 6; i++)
    {
        param_debug.cartesianError[i] = this->cartesianError[i];
    }
    for (int i = 0; i < 3; i++)
    {
        param_debug.position[i] = robot->getPosition()[i];
        param_debug.position_d[i] = this->position_d[i];
        param_debug.orientation[i] = robot->getOrientation().toRotationMatrix().eulerAngles(2, 1, 0)[i];
        param_debug.orientation_d[i] = this->orientation_d.toRotationMatrix().eulerAngles(2, 1, 0)[i];
    }
}

Robot7 *pPanda = nullptr;
Robot7Controller *pController = nullptr;


/***/
/***/
/***/
/***/
/***/
/***/
void pandaRun(Eigen::Matrix<double, DIM, 1> q, Eigen::Matrix<double, DIM, 1> dq, Eigen::Matrix<double, DIM, 1> tau, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Affine3d TO2E, Eigen::Matrix<double, DIM, 1> &tau_d, panda_controller::paramForDebug &param_debug)
{
    pController->updateTime();

    pPanda->updateJointData(q, dq, tau);

    pPanda->updateEndeffectorData(position, orientation, TO2E);

    pPanda->calculation(pController->ddq_d);

    pController->calDesire(pPanda);

    pController->calError(pPanda);
    pController->setControllerLaw(pPanda, tau_d);

    pController->recordData(pPanda);

    pController->controllerParamRenew();

    pController->pubData(param_debug, pPanda);
}

void pandaInit()
{
    // if (pinInteractive == nullptr)
    // {
    //     pinInteractive = new pinLibInteractive();
    // }
    if (pPandaDynLibManager == nullptr)
    {
        pPandaDynLibManager = new pandaDynLibManager();
    }
    if (pController == nullptr)
    {
        pController = new my_controller::ComputedTorqueMethod();
    }
    if (pPanda == nullptr)
    {
        pPanda = new my_robot::Robot<DIM>();
    }
}

void pandaStart(Eigen::Matrix<double, DIM, 1> q0, int recordPeriod)
{
    pController->setRecord(recordPeriod);
    pPanda->setq0(q0);
}

void pandaGetDyn(Eigen::Matrix<double, 7, 7> M,Eigen::Matrix<double, 7, 1> c,Eigen::Matrix<double, 7, 1> G,Eigen::Matrix<double, 6, 7> J)
{
    pPanda->setExternM(M);
    pPanda->setExternc(c);
    pPanda->setExternG(G);
    pPanda->setExternJ(J);
}