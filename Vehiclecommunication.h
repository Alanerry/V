#ifndef VEHICLECOMMUNICATION_H_
#define VEHICLECOMMUNICATION_H_

#include "omnetpp.h"
#include "GpsrRouting.h"  // 包含 GPSR 协议定义的头文件

using namespace omnetpp;

class VehicleModule : public cSimpleModule
{
private:
    static std::map<double, int> communicationCounts;  // 记录每个时间戳的通讯次数
    static double lastOutputTime;  // 记录最后一次输出的时间
    cMessage *updatePositionMsg = nullptr;  // 用于位置更新的消息
    GpsrRouting gpsr;  // GPSR 协议对象
    cMessage *timerMsg;  // 用于定时发送消息的计时器

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    void updatePosition();
    void scheduleNextUpdate();
    void sendMessage();

public:
    VehicleModule() : timerMsg(nullptr) {}
    ~VehicleModule();
};

#endif /* VEHICLECOMMUNICATION_H_ */
