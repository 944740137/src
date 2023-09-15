#include "communication/communication.h"
#include "robot/robot.hpp"

struct ControllerParam
{
    char *paramName = nullptr;
    double value = 0.0;
};

template <int _Dofs>
struct ControllerParamBase
{
public:
    ControllerParam jointParam1[_Dofs];
    ControllerParam jointParam2[_Dofs];
    ControllerParam jointParam3[_Dofs];
    ControllerParam cartesianParam1[6];
    ControllerParam cartesianParam2[6];
    ControllerParam cartesianParam3[6];
};

//
// 控制律基类
template <int _Dofs>
class ControllerLaw
{
public:
    TaskSpace taskSpace;
    std::string controllerLawName;

    // 当前时刻误差
    Eigen::Matrix<double, _Dofs, 1> jointError;
    Eigen::Matrix<double, _Dofs, 1> djointError;
    Eigen::Matrix<double, 6, 1> cartesianError;
    Eigen::Matrix<double, 6, 1> dcartesianError;

    // 当前时刻期望 关节空间
    Eigen::Matrix<double, _Dofs, 1> q_d;
    Eigen::Matrix<double, _Dofs, 1> dq_d;
    Eigen::Matrix<double, _Dofs, 1> ddq_d;

    // 当前时刻期望 笛卡尔空间
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;
    Eigen::Vector3d dposition_d;
    Eigen::Quaterniond dorientation_d;
    Eigen::Matrix<double, 6, 1> ddX_d; // position+orientation

    // controllerLaw
    Eigen::Matrix<double, _Dofs, 1> tau_d;
    Eigen::Matrix<double, _Dofs, 1> qc;

public:
    ControllerLaw(const ControllerLaw &) = delete;
    void operator=(const ControllerLaw &) = delete;

    ControllerLaw() = delete;
    ControllerLaw(TaskSpace createTaskSpace, std::string createControllerLawName);
    virtual ~ControllerLaw();

    virtual void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d) = 0;
    virtual void dynamicSetParameter(const ControllerParamBase<_Dofs> &config, unsigned int time) = 0;
    virtual void controllerParamRenew(double filterParams) = 0;
};
template <int _Dofs>
ControllerLaw<_Dofs>::~ControllerLaw()
{
}
template <int _Dofs>
ControllerLaw<_Dofs>::ControllerLaw(TaskSpace createTaskSpace, std::string createControllerLawName) : jointError(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                                                                      djointError(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                                                                      cartesianError(Eigen::Matrix<double, 6, 1>::Zero()),
                                                                                                      dcartesianError(Eigen::Matrix<double, 6, 1>::Zero()),
                                                                                                      q_d(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                                                                      dq_d(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                                                                      ddq_d(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                                                                      position_d(Eigen::Matrix<double, 3, 1>::Zero()),
                                                                                                      orientation_d(Eigen::Quaterniond::Identity()),
                                                                                                      dposition_d(Eigen::Matrix<double, 3, 1>::Zero()),
                                                                                                      dorientation_d(Eigen::Quaterniond::Identity()),
                                                                                                      ddX_d(Eigen::Matrix<double, 6, 1>::Zero()),
                                                                                                      tau_d(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                                                                      qc(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                                                                      taskSpace(createTaskSpace),
                                                                                                      controllerLawName(createControllerLawName)

{
    // 全部初始化为0
}

//
//
// 计算力矩控制器
template <int _Dofs>
class ComputedTorqueMethod : public ControllerLaw<_Dofs>
{
public:
    // 关节空间
    Eigen::Matrix<double, _Dofs, _Dofs> jointKv;
    Eigen::Matrix<double, _Dofs, _Dofs> jointKp;

    Eigen::Matrix<double, _Dofs, _Dofs> jointKv_d;
    Eigen::Matrix<double, _Dofs, _Dofs> jointKp_d;

    // 笛卡尔空间
    Eigen::Matrix<double, 6, 6> cartesianKp;
    Eigen::Matrix<double, 6, 6> cartesianKv;

    Eigen::Matrix<double, 6, 6> cartesianKp_d;
    Eigen::Matrix<double, 6, 6> cartesianKv_d;

public:
    ComputedTorqueMethod(const ComputedTorqueMethod &) = delete;
    void operator=(const ComputedTorqueMethod &) = delete;
    ComputedTorqueMethod() = delete;

    ~ComputedTorqueMethod();
    explicit ComputedTorqueMethod(TaskSpace taskSpace);

    void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d);

    void dynamicSetParameter(const ControllerParamBase<_Dofs> &config, unsigned int time);
    void controllerParamRenew(double filterParams);
};
template <int _Dofs>
ComputedTorqueMethod<_Dofs>::~ComputedTorqueMethod()
{
}
template <int _Dofs>
ComputedTorqueMethod<_Dofs>::ComputedTorqueMethod(TaskSpace taskSpace) : ControllerLaw<_Dofs>(taskSpace, "ComputedTorqueMethod"),
                                                                         jointKv(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                                                         jointKp(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                                                         jointKv_d(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                                                         jointKp_d(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                                                         cartesianKp(Eigen::Matrix<double, 6, 6>::Zero()),
                                                                         cartesianKv(Eigen::Matrix<double, 6, 6>::Zero()),
                                                                         cartesianKp_d(Eigen::Matrix<double, 6, 6>::Zero()),
                                                                         cartesianKv_d(Eigen::Matrix<double, 6, 6>::Zero())
{
    std::cout << "[robotController] 设置控制律: " << this->controllerLawName << std::endl;
}
template <int _Dofs>
void ComputedTorqueMethod<_Dofs>::setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d_in)
{
    if (this->taskSpace == jointSpace)
    {
        this->qc = this->ddq_d + jointKp * this->jointError + jointKv * this->djointError;
        this->tau_d << robot->getM() * (this->qc) + robot->getC() * robot->getdq() /* + G */;
        tau_d_in = this->tau_d;
    }
    else
    {
        // Eigen::Matrix<double, 6, 1> ddX;
        // ddX = ddX_d - cartesianKp_d * cartesianError - cartesianKv_d * dcartesianError;
        // this->tau_d << robot->getM() * (robot->getJ_inv() * (ddX - robot->getExternJ() * robot->getdq())) + robot->getC() * robot->getdq() /* + G */;
        // tau_d_in = this->tau_d;
    }
}
template <int _Dofs>
void ComputedTorqueMethod<_Dofs>::dynamicSetParameter(const ControllerParamBase<_Dofs> &config, unsigned int time)
{
    if (this->taskSpace == jointSpace)
    {
        if (time == 0)
        {
            for (int i = 0; i < _Dofs; i++)
            {
                jointKp(i, i) = config.jointParam1[i].value;
                jointKv(i, i) = config.jointParam2[i].value;
            }
        }
        for (int i = 0; i < _Dofs; i++)
        {
            jointKp_d(i, i) = config.jointParam1[i].value;
            jointKv_d(i, i) = config.jointParam2[i].value;
        }
    }
    else
    {
    }
}
template <int _Dofs>
void ComputedTorqueMethod<_Dofs>::controllerParamRenew(double filterParams)
{
    this->jointKp = filterParams * this->jointKp_d + (1.0 - filterParams) * this->jointKp;
    this->jointKv = filterParams * this->jointKv_d + (1.0 - filterParams) * this->jointKv;

    this->cartesianKp = filterParams * this->cartesianKp_d + (1.0 - filterParams) * this->cartesianKp;
    this->cartesianKv = filterParams * this->cartesianKv_d + (1.0 - filterParams) * this->cartesianKv;
}

//
//
// 反步控制器
template <int _Dofs>
class Backstepping : public ControllerLaw<_Dofs>
{
    // 关节空间
    Eigen::Matrix<double, _Dofs, _Dofs> jointK1;
    Eigen::Matrix<double, _Dofs, _Dofs> jointK2;

    Eigen::Matrix<double, _Dofs, _Dofs> jointK1_d;
    Eigen::Matrix<double, _Dofs, _Dofs> jointK2_d;

    // 笛卡尔空间
    Eigen::Matrix<double, 6, 6> cartesianK1;
    Eigen::Matrix<double, 6, 6> cartesianK2;

    Eigen::Matrix<double, 6, 6> cartesianK1_d;
    Eigen::Matrix<double, 6, 6> cartesianK2_d;

    // 误差中间变量
    Eigen::Matrix<double, _Dofs, 1> e1;
    Eigen::Matrix<double, _Dofs, 1> e2;
    Eigen::Matrix<double, _Dofs, 1> r;
    Eigen::Matrix<double, _Dofs, 1> dr;

public:
    Backstepping(const Backstepping &) = delete;
    void operator=(const Backstepping &) = delete;
    Backstepping() = delete;

    ~Backstepping();
    explicit Backstepping(TaskSpace taskSpace);

    void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d);

    void dynamicSetParameter(const ControllerParamBase<_Dofs> &config, unsigned int time);
    void controllerParamRenew(double filterParams);
};
template <int _Dofs>
Backstepping<_Dofs>::~Backstepping()
{
}
template <int _Dofs>
Backstepping<_Dofs>::Backstepping(TaskSpace taskSpace) : ControllerLaw<_Dofs>(taskSpace, "Backstepping"),
                                                         jointK1(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                                         jointK2(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                                         jointK1_d(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                                         jointK2_d(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                                         cartesianK1(Eigen::Matrix<double, 6, 6>::Zero()),
                                                         cartesianK2(Eigen::Matrix<double, 6, 6>::Zero()),
                                                         cartesianK1_d(Eigen::Matrix<double, 6, 6>::Zero()),
                                                         cartesianK2_d(Eigen::Matrix<double, 6, 6>::Zero()),
                                                         e1(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                         e2(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                         r(Eigen::Matrix<double, _Dofs, 1>::Zero()),
                                                         dr(Eigen::Matrix<double, _Dofs, 1>::Zero())
{
    std::cout << "[robotController] 设置控制律: " << this->controllerLawName << std::endl;
}
template <int _Dofs>
void Backstepping<_Dofs>::setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d_in)
{
    // note: 父类是抽象模板类，子类使用其成员需显式指定：this->或base::
    if (this->taskSpace == jointSpace)
    {
        this->e1 = this->jointError;
        this->e2 = this->djointError + this->jointK1 * e1;
        this->r = this->dq_d + this->jointK1 * e1;
        this->dr = this->ddq_d + this->jointK1 * this->djointError;

        this->tau_d << robot->getM() * (this->dr) + robot->getC() * (this->dr) /* + G */ + this->jointK2 * this->e2 + this->e1;
        tau_d_in = this->tau_d;
    }
    else
    {
        // Eigen::Matrix<double, 6, 1> ddX;
        // ddX = ddX_d - cartesianKp_d * cartesianError - cartesianKv_d * dcartesianError;
        // this->tau_d << robot->getM() * (robot->getJ_inv() * (ddX - robot->getExternJ() * robot->getdq())) + robot->getC() * robot->getdq() /* + G */;
        // tau_d_in = this->tau_d;
    }
}
template <int _Dofs>
void Backstepping<_Dofs>::dynamicSetParameter(const ControllerParamBase<_Dofs> &config, unsigned int time)
{
    if (this->taskSpace == jointSpace)
    {
        if (time == 0)
        {
            for (int i = 0; i < _Dofs; i++)
            {
                jointK1(i, i) = config.jointParam1[i].value;
                jointK2(i, i) = config.jointParam2[i].value;
            }
        }
        for (int i = 0; i < _Dofs; i++)
        {
            jointK1_d(i, i) = config.jointParam1[i].value;
            jointK2_d(i, i) = config.jointParam2[i].value;
        }
    }
    else
    {
    }
}
template <int _Dofs>
void Backstepping<_Dofs>::controllerParamRenew(double filterParams)
{
    this->jointK1 = filterParams * this->jointK1_d + (1.0 - filterParams) * this->jointK1;
    this->jointK2 = filterParams * this->jointK2_d + (1.0 - filterParams) * this->jointK2;

    this->cartesianK1 = filterParams * this->cartesianK1_d + (1.0 - filterParams) * this->cartesianK1;
    this->cartesianK2 = filterParams * this->cartesianK2_d + (1.0 - filterParams) * this->cartesianK2;
}

//
//
// PD+重力补偿
template <int _Dofs>
class PD : public ControllerLaw<_Dofs>
{
public:
    // 关节空间
    Eigen::Matrix<double, _Dofs, _Dofs> jointKv;
    Eigen::Matrix<double, _Dofs, _Dofs> jointKp;

    Eigen::Matrix<double, _Dofs, _Dofs> jointKv_d;
    Eigen::Matrix<double, _Dofs, _Dofs> jointKp_d;

    // 笛卡尔空间
    Eigen::Matrix<double, 6, 6> cartesianKp;
    Eigen::Matrix<double, 6, 6> cartesianKv;

    Eigen::Matrix<double, 6, 6> cartesianKp_d;
    Eigen::Matrix<double, 6, 6> cartesianKv_d;

public:
    PD(const PD &) = delete;
    void operator=(const PD &) = delete;
    PD() = delete;

    ~PD();
    explicit PD(TaskSpace taskSpace);

    void setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d);

    void dynamicSetParameter(const ControllerParamBase<_Dofs> &config, unsigned int time);
    void controllerParamRenew(double filterParams);
};
template <int _Dofs>
PD<_Dofs>::~PD()
{
}
template <int _Dofs>
PD<_Dofs>::PD(TaskSpace taskSpace) : ControllerLaw<_Dofs>(taskSpace, "PD"),
                                     jointKv(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                     jointKp(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                     jointKv_d(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                     jointKp_d(Eigen::Matrix<double, _Dofs, _Dofs>::Zero()),
                                     cartesianKp(Eigen::Matrix<double, 6, 6>::Zero()),
                                     cartesianKv(Eigen::Matrix<double, 6, 6>::Zero()),
                                     cartesianKp_d(Eigen::Matrix<double, 6, 6>::Zero()),
                                     cartesianKv_d(Eigen::Matrix<double, 6, 6>::Zero())
{
    std::cout << "[robotController] 设置控制律: " << this->controllerLawName << std::endl;
}
template <int _Dofs>
void PD<_Dofs>::setControllerLaw(my_robot::Robot<_Dofs> *robot, Eigen::Matrix<double, _Dofs, 1> &tau_d_in)
{
    if (this->taskSpace == jointSpace)
    {
        this->tau_d << this->ddq_d + jointKp * this->jointError + jointKv * this->djointError /* + G */;
        tau_d_in = this->tau_d;
    }
    else
    {
        // Eigen::Matrix<double, 6, 1> ddX;
        // ddX = ddX_d - cartesianKp_d * cartesianError - cartesianKv_d * dcartesianError;
        // this->tau_d << robot->getM() * (robot->getJ_inv() * (ddX - robot->getExternJ() * robot->getdq())) + robot->getC() * robot->getdq() /* + G */;
        // tau_d_in = this->tau_d;
    }
}
template <int _Dofs>
void PD<_Dofs>::dynamicSetParameter(const ControllerParamBase<_Dofs> &config, unsigned int time)
{
    if (this->taskSpace == jointSpace)
    {
        if (time == 0)
        {
            for (int i = 0; i < _Dofs; i++)
            {
                jointKp(i, i) = config.jointParam1[i].value;
                jointKv(i, i) = config.jointParam2[i].value;
            }
        }
        for (int i = 0; i < _Dofs; i++)
        {
            jointKp_d(i, i) = config.jointParam1[i].value;
            jointKv_d(i, i) = config.jointParam2[i].value;
        }
    }
    else
    {
    }
}
template <int _Dofs>
void PD<_Dofs>::controllerParamRenew(double filterParams)
{
    this->jointKp = filterParams * this->jointKp_d + (1.0 - filterParams) * this->jointKp;
    this->jointKv = filterParams * this->jointKv_d + (1.0 - filterParams) * this->jointKv;

    this->cartesianKp = filterParams * this->cartesianKp_d + (1.0 - filterParams) * this->cartesianKp;
    this->cartesianKv = filterParams * this->cartesianKv_d + (1.0 - filterParams) * this->cartesianKv;
}

//

template <int _Dofs>
bool newControllerLaw(std::unique_ptr<ControllerLaw<_Dofs>> &controllerLaw, ControllerLawType controllerLawType, TaskSpace taskSpace)
{
    if (controllerLaw != nullptr)
        controllerLaw.reset(nullptr);
    switch (controllerLawType)
    {
    case ControllerLawType::ComputedTorqueMethod_:
        controllerLaw = std::make_unique<ComputedTorqueMethod<_Dofs>>(taskSpace);
        break;
    case ControllerLawType::Backstepping_:
        controllerLaw = std::make_unique<Backstepping<_Dofs>>(taskSpace);
        break;
    case ControllerLawType::PD_:
        controllerLaw = std::make_unique<PD<_Dofs>>(taskSpace);
        break;
    default:
        controllerLaw = std::make_unique<ComputedTorqueMethod<_Dofs>>(TaskSpace::jointSpace);
        return false;
        break;
    }
    return true;
}