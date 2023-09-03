#include <cmath>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>

template <int _Dofs>
void calQuinticPlanTime(bool isCoordinated, double *maxVel, double *maxAcc, double *T,
                        const Eigen::Matrix<double, _Dofs, 1> &q0, const Eigen::Matrix<double, _Dofs, 1> &qf)
{
    double Tf = 0.0;
    for (int i = 0; i < _Dofs; i++)
    {
        double error = std::fabs(qf[i] - q0[i]);
        double t1 = (15.0 / 8.0 * error) / (maxVel[i]); // note 15.0/8.0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        double t2 = sqrt(5.7735 * error / maxAcc[i]);   // 10/3^0.5
        T[i] = std::max(t1, t2);
        Tf = std::max(Tf, T[i]);
    }
    if (isCoordinated)
    {
        for (int i = 0; i < _Dofs; i++)
        {
            T[i] = Tf;
        }
    }
}

// isCoordinated为false未测试
template <int _Dofs>
void calQuinticPlan(bool isCoordinated, double deltaT, double *maxVel, double *maxAcc,
                    const Eigen::Matrix<double, _Dofs, 1> &q0, const Eigen::Matrix<double, _Dofs, 1> &qf,
                    std::vector<std::queue<double>> &q_d, std::vector<std::queue<double>> &dq_d, std::vector<std::queue<double>> &ddq_d)
{
    double T[_Dofs] = {0.0};
    double dq0[_Dofs] = {0.0};
    double ddq0[_Dofs] = {0.0};
    double dqf[_Dofs] = {0.0};
    double ddqf[_Dofs] = {0.0};
    int pointNum[_Dofs] = {0};

    calQuinticPlanTime<_Dofs>(isCoordinated, maxVel, maxAcc, T, q0, qf);

    double t = 0;
    for (int i = 0; i < _Dofs; i++)
    {
        pointNum[i] = static_cast<int>(T[i] / deltaT) + 1;
        double a0 = q0[i];
        double a1 = 0;
        double a2 = 0;
        double a3 = (20 * qf[i] - 20 * q0[i] - (8 * dqf[i] + 12 * dq0[i]) * T[i] - (3 * ddq0[i] - ddqf[i]) * pow(T[i], 2)) / (2 * pow(T[i], 3));
        double a4 = (30 * q0[i] - 30 * qf[i] + (14 * dqf[i] + 16 * dq0[i]) * T[i] + (3 * ddq0[i] - 2 * ddqf[i]) * pow(T[i], 2)) / (2 * pow(T[i], 4));
        double a5 = (12 * qf[i] - 12 * q0[i] - (6 * dqf[i] + 6 * dq0[i]) * T[i] - (ddq0[i] - ddqf[i]) * pow(T[i], 2)) / (2 * pow(T[i], 5));

        for (int j = 1; j <= pointNum[i]; j++)
        {
            t = j * deltaT;
            if (j == pointNum[i])
                q_d[i].push(qf[i]);
            else
                q_d[i].push(a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5));
            dq_d[i].push(a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4));
            ddq_d[i].push(2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3));
        }
    }
}
template <int _Dofs>
double calStopPlanTime(const Eigen::Matrix<double, _Dofs, 1> &ddq, double dddq)
{
    double Tmax = 0.0;
    for (int i = 0; i < _Dofs; i++)
    {
        double error = std::fabs(ddq[i]);
        double T = error / dddq;
        if (i == 0 || T > Tmax)
            Tmax = T;
    }
    return Tmax;
}
template <int _Dofs>
void calStopPlan(double deltaT, double dddq, const Eigen::Matrix<double, _Dofs, 1> &ddq,
                 std::vector<std::queue<double>> &q_d, std::vector<std::queue<double>> &dq_d, std::vector<std::queue<double>> &ddq_d)
{
    double stopT = calStopPlanTime(ddq, dddq);
}
// template <int _Dofs, typename pubDataType, typename dynParamType>
// void Controller<_Dofs, pubDataType, dynParamType>::calStopQueue(my_robot::Robot<_Dofs> *robot)
// {
//     std::queue<double> empty;

//     // 清空运行队列
//     for (int i = 0; i < _Dofs; i++)
//     {
//         swap(empty, q_dQueue[i]);
//         swap(empty, dq_dQueue[i]);
//         swap(empty, ddq_dQueue[i]);
//     }
// }