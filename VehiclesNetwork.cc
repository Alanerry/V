#include <omnetpp.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include "GpsrRouting.h"

using namespace omnetpp;

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

    cMessage *sendMsg = nullptr;  // 用于发送消息的事件
    bool hasReceivedMsg = false;  // 标记是否收到消息

protected:
    // 初始化函数，OMNeT++ 在模块开始时调用
    virtual void initialize() override {
        // 创建位置更新消息
        updatePositionMsg = new cMessage("updatePosition");
        // 从0秒开始调度位置更新
        scheduleAt(0, updatePositionMsg);

        // 为车辆2添加发送消息的调度
        if (getIndex() == 2) {
            sendMsg = new cMessage("Message from Car 2");
            scheduleAt(18.0, sendMsg);
        }
        // 为车辆1添加发送消息的调度
        if (getIndex() == 1) {
            sendMsg = new cMessage("Message from Car 1");
            scheduleAt(18.0, sendMsg);
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
        else if (msg == sendMsg) {
            // 获取父模块（网络模块）
            cModule *network = getParentModule();
            if (!network) {
                EV << "错误：无法获取网络模块" << endl;
                return;
            }

            if (getIndex() + 1 == 2) {  // 车辆2发送消息给车辆1
                // 通过父模块获取车辆1
                cModule *car1 = network->getSubmodule("vehicle", 0);  // 获取索引为0的vehicle模块
                if (car1) {
                    cMessage *msgToSend = new cMessage("来自车辆2的消息");
                    sendDirect(msgToSend, car1, "in", 0);  // 使用第一个输入门
                    EV << "车辆2在时刻 " << simTime() << " 向车辆1发送了消息" << endl;
                } else {
                    EV << "错误：找不到车辆1模块" << endl;
                }
            }
            else if (getIndex() + 1 == 1) {  // 车辆1发送消息给车辆2
                // 通过父模块获取车辆2
                cModule *car2 = network->getSubmodule("vehicle", 1);  // 获取索引为1的vehicle模块
                if (car2) {
                    cMessage *msgToSend = new cMessage("来自车辆1的消息");
                    sendDirect(msgToSend, car2, "in", 0);  // 使用第一个输入门
                    EV << "车辆1在时刻 " << simTime() << " 向车辆2发送了消息" << endl;
                } else {
                    EV << "错误：找不到车辆2模块" << endl;
                }
            }
        }
        else {
            EV << "车辆 " << (getIndex() + 1) << " 在时刻 " << simTime()
               << " 收到消息: " << msg->getName() << endl;
            delete msg;
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
