#include <omnetpp.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <queue>
#include "VehiclesNetwork.h"

using namespace omnetpp;

extern int my_id;

// 初始化静态变量
std::map<double, int> VehicleModule::communicationCounts;
double VehicleModule::lastOutputTime = -1;
Define_Module(VehicleModule);


    void VehicleModule::initialize(){
        // 初始化消息对象
        updatePositionMsg = new cMessage("updatePosition");
        timerMsg = new cMessage("timerMsg");
        sendMsgTimer = new cMessage("sendMsgTimer");

        // 开始调度消息发送
        scheduleAt(simTime() + 1, sendMsgTimer);  // 从1秒后开始发送路由表中的消息
    }

    void VehicleModule::handleMessage(cMessage *msg){
        if (msg == updatePositionMsg) {
            updatePosition();
            double currentTime = simTime().dbl();
            communicationCounts[currentTime]++;
            scheduleNextUpdate();
        } else if (msg == timerMsg) {
            sendMessage();
            scheduleAt(simTime() + 1, timerMsg);  // 每秒发送一次消息
        } else if (msg == sendMsgTimer) {
            if (!routeTable.empty()) {
                cMessage* msgToSend = routeTable.top().msg;
                routeTable.pop();
                send(msgToSend, "out");
            }
            scheduleAt(simTime() + 1, sendMsgTimer);  // 每秒发送一次路由表中的消息
        } else {
            // 处理从其他模块接收的消息
            EV << "Received message: " << msg->getName() << "\n";
            routeTable.emplace(msg, simTime());  // 将接收到的消息添加到路由表中
        }
    }

    void VehicleModule::updatePosition() {
        const char *filename = par("xmlFilename").stringValue();
        cXMLElement *xml = getEnvir()->getXMLDocument(filename, nullptr);
        if (xml) {
            double currentTime = simTime().dbl();
            cXMLElement *vehicleElement = xml->getFirstChildWithTag("VehiclePosition");
            while (vehicleElement) {
                double timestamp = atof(vehicleElement->getAttribute("time"));
                if (timestamp == currentTime) {
                    int vehicleId = atoi(vehicleElement->getFirstChildWithTag("VehicleID")->getNodeValue());
                    const char* x = vehicleElement->getFirstChildWithTag("X")->getNodeValue();
                    const char* y = vehicleElement->getFirstChildWithTag("Y")->getNodeValue();
                    double displayX = atof(x) + 300;  // 中心点调整
                    double displayY = atof(y) + 350;
                    vehiclePositions[vehicleId] = std::make_pair(displayX, displayY); // 更新vehiclePositions

                    // 如果是当前车辆，更新显示
                    if (vehicleId == getIndex() + 1) {
                        getDisplayString().setTagArg("p", 0, std::to_string(displayX).c_str());
                        getDisplayString().setTagArg("p", 1, std::to_string(displayY).c_str());
                    }
                }
                vehicleElement = vehicleElement->getNextSiblingWithTag("VehiclePosition");
            }
        }
    }

    void VehicleModule::updateNeighbors() {
        int myId = getIndex() + 1;  // 当前车辆的ID
        double myX = vehiclePositions[myId].first;
        double myY = vehiclePositions[myId].second;

        gpsr.neighborTable.clear();  // 清空邻居表，重新添加

        // 遍历所有车辆，计算距离，并判断是否在50米范围内
        for (const auto& [vehicleId, position] : vehiclePositions) {
            if (vehicleId != myId) {
                double distance = calculateDistance(myX, myY, position.first, position.second);
                if (distance <= 50.0) {
                    // 添加到邻居表
                    GPSR_neighborRecord neighbor;
                    neighbor.id = vehicleId;
                    neighbor.x = position.first;
                    neighbor.y = position.second;
                    neighbor.ts = simTime().dbl();  // 时间戳
                    gpsr.neighborTable[vehicleId] = neighbor;
                }
            }
        }
    }

    double VehicleModule::calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    void VehicleModule::scheduleNextUpdate() {
        double currentTime = simTime().dbl();
        auto it = communicationCounts.upper_bound(currentTime);
        if (it != communicationCounts.end()) {
            scheduleAt(it->first, updatePositionMsg);
        } else {
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
