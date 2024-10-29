#ifndef VEHICLECOMMUNICATION_H_
#define VEHICLECOMMUNICATION_H_

#include <omnetpp.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <queue>
#include <unordered_map>
#include "GpsrRouting.h"

using namespace omnetpp;

// 定义路由表结构
struct RouteEntry {
    cMessage* msg;
    simtime_t receiveTime;

    RouteEntry(cMessage* m, simtime_t t) : msg(m), receiveTime(t) {}
};

class VehicleModule : public cSimpleModule
{
private:
    static std::map<double, int> communicationCounts;  // 记录每个时间戳的通讯次数
    static double lastOutputTime;  // 记录最后一次输出的时间

    cMessage *updatePositionMsg = nullptr;  // 用于位置更新的消息
    cMessage *timerMsg = nullptr;  // 用于定时发送消息的计时器
    cMessage *sendMsgTimer = nullptr;  // 用于发送路由表中的消息的定时器

    GpsrRouting gpsr;  // GPSR 路由协议实例
    std::priority_queue<RouteEntry, std::vector<RouteEntry>, std::greater<RouteEntry>> routeTable;  // 路由表

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    void updatePosition();
    void sendMessage();
    void scheduleNextUpdate();
    void updateNeighbors();

public:
    std::unordered_map<int, std::pair<double, double>> vehiclePositions; // 新增的变量声明
    VehicleModule() : cSimpleModule(), updatePositionMsg(nullptr), timerMsg(nullptr), sendMsgTimer(nullptr) {}
        ~VehicleModule() {
            cancelAndDelete(updatePositionMsg);
            cancelAndDelete(timerMsg);
            cancelAndDelete(sendMsgTimer);
        }

    // 计算两点之间的距离
    double calculateDistance(double x1, double y1, double x2, double y2);
};

#endif /* VEHICLECOMMUNICATION_H_ */
