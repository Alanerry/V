#ifndef _GPSRROUTING_H_
#define _GPSRROUTING_H_

// 定义默认的GPSR超时时间
#define DEFAULT_GPSR_TIMEOUT   20.0
/*
   默认超时时间为20.0秒
   如果在此期间未收到hello消息
   邻居列表中的条目会被删除
*/

// 如果出现问题，将在GPSR_RETRY_HELLO_DELAY秒后重试发送HELLO消息
#define GPSR_RETRY_HELLO_DELAY 1

using namespace std;

// 定义邻居节点记录结构
struct GPSR_neighborRecord {
    int id;      // 节点ID
    int x;       // 节点坐标：地理信息
    int y;       // 节点坐标：地理信息
    double ts;   // 从该节点接收到的最后一条hello消息的时间戳

    // 构造函数，初始化所有成员
    GPSR_neighborRecord() {
       id = 0;
       x = 0.0;
       y = 0.0;
       ts = 0.0;
    }
};

// 定义GPSR路由定时器枚举
enum GpsrRoutingTimers {
    GPSR_HELLO_MSG_REFRESH_TIMER = 0, // 刷新HELLO消息的定时器
    GPSR_HELLO_MSG_EXPIRE_TIMER  = 1, // HELLO消息过期的定时器
};

// GPSR路由类定义
class GpsrRouting {
 private:
    map<int, GPSR_neighborRecord> neighborTable; // 邻居表

 protected:
    // GPSR路由协议核心函数将在此处定义

 public:
    // 添加公共函数接口供外部调用
    int greedy_forwarding(int destX, int destY, bool useGG);
    int peri_nexthop(bool useGG, int last, double sx, double sy, double dx, double dy);
    std::vector<GPSR_neighborRecord> rng_planarize();
    std::vector<GPSR_neighborRecord> gg_planarize();
    double calculateDistance(int x1, int y1, int x2, int y2);
    double angle(double x1, double y1, double x2, double y2);
    bool intersect(int theother, double sx, double sy, double dx, double dy);
};

#endif  // _GPSRROUTING_H_
