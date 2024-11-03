#ifndef GPSRROUTING_H_
#define GPSRROUTING_H_

#include <omnetpp.h>
#include <map>
#include <vector>

using namespace omnetpp;

// 邻居节点记录结构
struct GPSR_neighborRecord {
    int id;
    double x;
    double y;
    simtime_t ts;
};

class GpsrRouting : public cSimpleModule {
private:
    // 私有成员变量
    static constexpr double PI = 3.14159265358979323846;
    std::map<int, GPSR_neighborRecord> neighborTable;
    int my_id;     // 当前节点ID
    double my_x;   // 当前节点的x坐标
    double my_y;   // 当前节点的y坐标

    // 私有辅助函数
    double angle(double x1, double y1, double x2, double y2);
    bool intersect(int theother, double sx, double sy, double dx, double dy);
    std::vector<GPSR_neighborRecord> rng_planarize();
    std::vector<GPSR_neighborRecord> gg_planarize();
    int peri_nexthop(bool useGG, int last, double sx, double sy, double dx, double dy);

public:
    friend class VehicleModule;
    // 构造和析构函数
    GpsrRouting();
    virtual ~GpsrRouting();

    // OMNeT++必需的虚函数
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    // 公共接口
    virtual int greedy_forwarding(int destX, int destY, bool useGG ,int nowid , int nowx , int nowy);
    virtual double calculateDistance(int x1, int y1, int x2, int y2);

    // Getter/Setter方法
    const std::map<int, GPSR_neighborRecord>& getNeighborTable() const {
        return neighborTable;
    }
};

#endif /* GPSRROUTING_H_ */
