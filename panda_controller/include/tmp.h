struct RobotData
{
    double q[7];
    double dq[7];
    double q_d[7];
    double dq_d[7];
    double ddq_d[7];

    double tau[7];
    double tau_d[7];

    double position[3];
    double orientation[3];
    double position_d[3];
    double orientation_d[3];
};