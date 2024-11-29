#include <omnetpp.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include "GpsrRouting.h"

using namespace omnetpp;

// 消息记录结构体
struct MessageRecord {
    std::string content; // 消息内容
    double arrivalTime;  // 到达时间
    int destination;     // 目的地索引
};

// VehicleModule 类定义
class VehicleModule : public cSimpleModule
{
private:
    // 静态成员变量：用于记录每个时间戳的通讯次数
    static std::map<double, int> communicationCounts;
    // 静态成员变量：记录最后一次输出的时间
    static double lastOutputTime;
    // 用于位置更新的消息
    cMessage *updatePositionMsg = nullptr;

    // 成员变量
    std::queue<MessageRecord> receivedMessages;
    cMessage *sendBufferedMsgsMsg = nullptr;

    // GPSR 路由器实例
    GpsrRouting* gpsrInstance;
    //std::map<int, GPSR_neighborRecord> neighborTable; // 邻居表

protected:
    // 初始化函数，OMNeT++ 在模块开始时调用
    virtual void initialize() override {
        // 创建位置更新消息
        updatePositionMsg = new cMessage("updatePosition");
        // 从0秒开始调度位置更新
        scheduleAt(0, updatePositionMsg);

        // 定期发送缓冲消息的定时器，每 0.5 秒触发一次
        sendBufferedMsgsMsg = new cMessage("sendBufferedMessages");
        scheduleAt(0.5, sendBufferedMsgsMsg);
    }

    virtual void processBufferedMessages() {
        while (!receivedMessages.empty()) {
            MessageRecord record = receivedMessages.front();
            receivedMessages.pop();
            EV << "车辆 " << (getIndex() + 1) << " 在时刻 " << simTime()
               << " 转发消息: " << record.content << " (记录时间: " << record.arrivalTime
               << ", 目的地: " << record.destination << ")" << endl;
        }
    }

    // 处理接收到的消息
    virtual void handleMessage(cMessage *msg) override {
        if (msg == updatePositionMsg) {
            double currentTime = simTime().dbl();
            updatePosition();
            communicationCounts[currentTime]++;
            scheduleNextUpdate();
        }
    }



    // 更新车辆的位置
    virtual void updatePosition() {
        const char *filename = par("xmlFilename").stringValue();  // 获取参数中指定的 XML 文件名
        cXMLElement *xml = getEnvir()->getXMLDocument(filename, nullptr);  // 加载 XML 文档
        if (xml) {
            double currentTime = simTime().dbl();  // 获取当前时间
            // 获取第一个车辆位置元素
            cXMLElement *vehicleElement = xml->getFirstChildWithTag("VehiclePosition");
            // 遍历所有车辆位置元素
            while (vehicleElement) {
                double timestamp = atof(vehicleElement->getAttribute("time"));  // 获取时间戳
                // 如果时间戳与当前时间匹配
                if (timestamp == currentTime) {
                    int vehicleId = atoi(vehicleElement->getFirstChildWithTag("VehicleID")->getNodeValue());  // 获取车辆ID
                    // 确保当前模块对应正确的车辆ID
                    if (vehicleId == getIndex() + 1) {
                        // 获取车辆的X和Y坐标
                        const char* x = vehicleElement->getFirstChildWithTag("X")->getNodeValue();
                        const char* y = vehicleElement->getFirstChildWithTag("Y")->getNodeValue();
                        // 调整显示坐标
                        double displayX = atof(x) + 300;
                        double displayY = atof(y) + 350;
                        // 设置模块的显示位置
                        getDisplayString().setTagArg("p", 0, std::to_string(displayX).c_str());
                        getDisplayString().setTagArg("p", 1, std::to_string(displayY).c_str());
                    }
                }
                // 获取下一个车辆位置元素
                vehicleElement = vehicleElement->getNextSiblingWithTag("VehiclePosition");
            }
        }
    }

    // 调度下一个位置更新
    virtual void scheduleNextUpdate() {
        double currentTime = simTime().dbl();  // 获取当前时间
        auto it = communicationCounts.upper_bound(currentTime);  // 查找下一个时间戳
        // 如果找到了下一个时间戳
        if (it != communicationCounts.end()) {
            scheduleAt(it->first, updatePositionMsg);  // 在下一个时间戳时调度位置更新
        } else {
            scheduleAt(currentTime + 1, updatePositionMsg);  // 默认情况下每秒更新一次位置
        }
    }
};

// 初始化静态变量
std::map<double, int> VehicleModule::communicationCounts;
double VehicleModule::lastOutputTime = -1;

// 定义模块
Define_Module(VehicleModule);

/**
~VehicleModule() {
    cancelAndDelete(updatePositionMsg);
    if (sendMsg) {
        cancelAndDelete(sendMsg);
    }
}
**/
