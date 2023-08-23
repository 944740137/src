#include "communication.h"
#include <stdio.h>
Communication::~Communication()
{
}
Communication::Communication()
{
}
bool Communication::checkConnect(SharedMemory *sharedMemoryData)
{
    this->timeoutCount++;
    if (this->timeoutCount >= this->maxTimeoutCount)
    {
        if (HeartBeatRecord == sharedMemoryData->masterHeartbeat)
            this->isConnect = false;
        else
            this->isConnect = true;
        HeartBeatRecord = sharedMemoryData->masterHeartbeat;
        this->timeoutCount = 0;
    }
    return this->isConnect;
}
bool Communication::createConnect(key_t messageKey, key_t sharedMemorykey, Message *messageData, SharedMemory *sharedMemoryData)
{
    void *shared_memory = nullptr;
    messageData = new Message();

    this->shm_id = shmget((key_t)SM_ID, sizeof(struct SharedMemory), 0666 | IPC_CREAT);
    if (this->shm_id < 0)
    {
        printf("第一次共享内存创建失败\n");
        return false;
    }
    else
        printf("共享内存创建成功\n");

    shared_memory = shmat(sharedMemorykey, NULL, 0);
    if (shared_memory == nullptr)
    {
        printf("共享内存映射失败\n");
        return false;
    }
    else
        printf("共享内存映射成功\n");

    sharedMemoryData = (struct SharedMemory *)shared_memory;
    sharedMemoryData->slaveHeartbeat = 0;

    this->msgid = msgget(messageKey, 0666 | IPC_CREAT);
    if (this->msgid == -1)
    {
        printf("消息队列创建失败\n");
        return false;
    }
    else
        printf("消息队列创建成功\n");

    return true;
}
bool Communication::comSendMessage(Message *messageData, SharedMemory *sharedMemoryData)
{
    if (checkConnect(sharedMemoryData))
    {
        messageData->time++;

        /*读取*/
        if (!this->connectStatus)
        {
            printf("主站连接\n");
            this->connectStatus = true;
        }
        // printf("主站在线\n");
        if (msgsnd(this->msgid, (void *)messageData, sizeof(struct Message) - sizeof(long), IPC_NOWAIT) != 0)
        {
            // printf("发送失败\n");
        }
        // printf("messageData: %d\n", messageData->time);
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
    return this->connectStatus;
}
bool Communication::comRecvMessage(Message *messageData, SharedMemory *sharedMemoryData)
{
    
    return this->connectStatus;
}
bool Communication::closeConnect(Message *messageData, SharedMemory *sharedMemoryData)
{
    return true;
}