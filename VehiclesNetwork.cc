#include "VehicleCommunication.h"
#include <omnetpp.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

using namespace omnetpp;

VehicleModule::~VehicleModule() {
    cancelAndDelete(updatePositionMsg);
    cancelAndDelete(timerMsg);
}

void VehicleModule::initialize()
{
    updatePositionMsg = new cMessage("updatePosition");
    scheduleAt(0, updatePositionMsg);  // 从0秒开始调度位置更新

    timerMsg = new cMessage("sendMessage");
    scheduleAt(simTime() + 1, timerMsg);  // 1秒后开始发送第一条消息

    // 初始化 GPSR 协议相关参数
    my_id = getIndex();
    my_x = par("my_x").intValue();
    my_y = par("my_y").intValue();

    // 初始化 GPSR 协议对象
    gpsr = GpsrRouting(my_id, my_x, my_y);

    // 假设这里初始化邻居表
    // gpsr.updateNeighborTable(/* 初始邻居ID */, /* 初始邻居X */, /* 初始邻居Y */);  // 如果需要初始化邻居表
}

void VehicleModule::handleMessage(cMessage *msg)
{
    if (msg == updatePositionMsg)
    {
        double currentTime = simTime().dbl();
        updatePosition();
        communicationCounts[currentTime]++;
        scheduleNextUpdate();
    }
    else if (msg == timerMsg)
    {
        sendMessage();
        scheduleAt(simTime() + 1, timerMsg);  // 每隔一秒发送一次消息
    }
    else
    {
        // 接收其他模块的消息
        int sender = msg->par("sender").intValue();
        double timestamp = msg->par("timestamp").doubleValue();
        EV_INFO << "Received message from node " << sender << " at time " << timestamp << "\n";
        // 更新邻居表
        // gpsr.updateNeighborTable(sender, /* 邻居X */, /* 邻居Y */);  // 如果需要更新邻居表
        delete msg;  // 删除接收到的消息
    }
}

void VehicleModule::updatePosition()
{
    const char *filename = par("xmlFilename").stringValue();
    cXMLElement *xml = getEnvir()->getXMLDocument(filename, nullptr);
    if (xml)
    {
        double currentTime = simTime().dbl();
        cXMLElement *vehicleElement = xml->getFirstChildWithTag("VehiclePosition");
        while (vehicleElement)
        {
            double timestamp = atof(vehicleElement->getAttribute("time"));
            if (timestamp == currentTime)
            {
                int vehicleId = atoi(vehicleElement->getFirstChildWithTag("VehicleID")->getNodeValue());
                if (vehicleId == getIndex() + 1)  // 确保当前模块对应正确的车辆ID
                {
                    const char* x = vehicleElement->getFirstChildWithTag("X")->getNodeValue();
                    const char* y = vehicleElement->getFirstChildWithTag("Y")->getNodeValue();
                    double displayX = atof(x) + 300;  // 中心点调整
                    double displayY = atof(y) + 350;
                    getDisplayString().setTagArg("p", 0, std::to_string(displayX).c_str());
                    getDisplayString().setTagArg("p", 1, std::to_string(displayY).c_str());
                }
            }
            vehicleElement = vehicleElement->getNextSiblingWithTag("VehiclePosition");
        }
    }
}

void VehicleModule::scheduleNextUpdate()
{
    double currentTime = simTime().dbl();
    auto it = communicationCounts.upper_bound(currentTime);
    if (it != communicationCounts.end())
    {
        scheduleAt(it->first, updatePositionMsg);
    }
    else
    {
        scheduleAt(currentTime + 1, updatePositionMsg);  // 默认情况下每秒更新一次位置
    }
}

void VehicleModule::sendMessage()
{
    // 创建一条消息，并设置一些参数
    cMessage *message = new cMessage("info");
    message->addPar("sender") = my_id;  // 设置发送者ID
    message->addPar("timestamp") = simTime().dbl();  // 设置发送时间戳

    // 获取目的地坐标
    int destX = par("destX").intValue();
    int destY = par("destY").intValue();

    // 使用 GPSR 协议选择下一跳
    int nextHopId = gpsr.greedy_forwarding(destX, destY, true /* useGG */);

    // 如果有下一跳，发送消息
    if (nextHopId != -1)
    {
        for (auto &neighbor : getParentModule()->getSubmodule("nodes")->getSubmodules())
        {
            if (neighbor->getIndex() == nextHopId)
            {
                sendDirect(message, neighbor->gate("in"));
                break;
            }
        }
    }
}

// 初始化静态变量
std::map<double, int> VehicleModule::communicationCounts;
double VehicleModule::lastOutputTime = -1;
