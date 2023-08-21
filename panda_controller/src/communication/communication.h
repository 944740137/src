/**进程通信头文件*/
#include <sys/msg.h>
#include <sys/shm.h>

#define SM_ID 0x1122
#define MS_ID 0x1122

enum TaskSpace
{
    jointSpace,
    cartesianSpace
};
enum CommandType
{
    append,
    changeStatus
};
enum ControllerStatus
{
    run,
    stop,
    // wait,

    run2stop,
    stop2run
};

struct Task
{
    double Tf;
    double q_final[7];
    TaskSpace plannerTaskSpace;

    // double orientation_final[3];
    // double position_final[3];
};

// msg
struct RobotData
{
    double q[7];
    double dq[7];
    double q_d[7];

    double tau[7];

    double position[3];
    double orientation[3];

    // char *plannerLawName;
    // char *controllerLawName;
};

struct ControllerCommand
{
    unsigned int taskNum;
    CommandType commandType;

    ControllerStatus ControllerStatus_d;

    Task task;
};

// 消息队列报文
struct Message
{
    long mytype = 1; // 消息类型为1
    unsigned int time;
    RobotData robotData;
};

// 共享内存报文
struct SharedMemory
{
    unsigned int slaveHeartbeat;      // 从站写
    unsigned int masterHeartbeat;     // 主站写
    ControllerCommand controllerData; // 主站写
};