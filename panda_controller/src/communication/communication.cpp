#include "communication.h"
#include <stdio.h>
Communication::~Communication()
{
}
Communication::Communication()
{
}
bool Communication::checkConnect()
{
    this->timeoutCount++;
    if (this->timeoutCount >= this->maxTimeoutCount)
    {
        if (this->HeartBeatRecord == this->sharedMemoryBuff->masterHeartbeat)
            this->isConnect = false;
        else
            this->isConnect = true;
        this->HeartBeatRecord = this->sharedMemoryBuff->masterHeartbeat;
        this->timeoutCount = 0;
    }
    return this->isConnect;
}
bool Communication::createConnect(key_t messageKey, key_t sharedMemorykey, RobotData *&robotData,
                                  ControllerCommand *&controllerCommand, ControllerState *&controllerState)
{
    void *shared_memory = nullptr;
    robotData = &(this->messageBuff.robotData);

    this->shm_id = shmget((key_t)SM_ID, sizeof(struct SharedMemory), 0666 | IPC_CREAT);
    if (this->shm_id < 0)
    {
        printf("第一次共享内存创建失败\n");
        return false;
    }
    else
        printf("共享内存创建成功\n");

    shared_memory = shmat(this->shm_id, NULL, 0);
    if (shared_memory == nullptr)
    {
        printf("共享内存映射失败\n");
        return false;
    }
    else
        printf("共享内存映射成功\n");

    this->sharedMemoryBuff = (struct SharedMemory *)shared_memory;
    this->sharedMemoryBuff->slaveHeartbeat = 0;
    this->HeartBeatRecord = this->sharedMemoryBuff->masterHeartbeat;

    controllerCommand = &(this->sharedMemoryBuff->controllerCommand);
    controllerState = &(this->sharedMemoryBuff->controllerState);

    this->msgid = msgget((key_t)messageKey, 0666 | IPC_CREAT);
    if (this->msgid == -1)
    {
        printf("消息队列创建失败\n");
        return false;
    }
    else
        printf("消息队列创建成功\n");

    return true;
}
bool Communication::comSendMessage()
{
    if (this->checkConnect())
    {
        this->messageBuff.time++;

        /*读取*/
        if (!this->connectStatus)
        {
            printf("主站连接\n");
            this->connectStatus = true;
        }
        // printf("主站在线\n");
        if (msgsnd(this->msgid, (void *)&(this->messageBuff), sizeof(struct Message) - sizeof(long), IPC_NOWAIT) != 0)
        {
            // printf("发送失败\n");
        }
    }
    else
    {
        if (this->connectStatus)
        {
            printf("主站断开\n");
            this->connectStatus = false;
        }
        // printf("主站离线\n");
    }
    this->sharedMemoryBuff->slaveHeartbeat++;

    return this->connectStatus;
}
bool Communication::comRecvMessage()
{

    return this->connectStatus;
}
bool Communication::closeConnect()
{
    return true;
}
