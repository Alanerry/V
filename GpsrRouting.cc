/*********************************************************/
/*  Copyright (c) 2011. University of Pau, France        */
/*  LIUPPA Laboratory, T2I Team                          */
/*                                                       */
/*  Permission to use, copy, modify and distribute this  */
/*  code, without fee, and without written agreement is  */
/*  hereby granted, provided that the above copyright    */
/*  notice and the authors appear in all copies          */
/*                                                       */
/*  GPSR Routing Protocol                                */
/*  Version:  1.0                                        */
/*  Authors: Diop Mamour <serignemamour.diop@gmail.com>  */
/*           Congduc Pham <congduc.pham@univ-pau.fr>     */
/*********************************************************/

#include "GpsrRouting.h"
#include "math.h"
#include <map>
#include <limits>
#include <cmath>
#include <algorithm>

using namespace std;

// 定义类成员变量
int my_id;
int my_x;
int my_y;
const double PI = 3.14159265358979323846;

// 贪婪算法实现部分
// 计算离目标最短节点，如果没有则使用周边转发
int GpsrRouting::greedy_forwarding(int destX, int destY, bool useGG) {
    // 先找到距离目的地最近的邻居节点

    int closestNeighborId = -1;
    double minDistance = calculateDistance(my_x, my_y, destX, destY);

    for (const auto& [id, neighbor] : neighborTable) {
        double distance = calculateDistance(neighbor.x, neighbor.y, destX, destY);
        if (distance < minDistance) {
            minDistance = distance;
            closestNeighborId = id;
        }
    }

    // 如果找不到比自己更接近目的地的邻居节点，进入路由空洞
    if (closestNeighborId == -1 || minDistance >= calculateDistance(my_x, my_y, destX, destY)) {
        // 进入路由空洞，切换到周边转发模式
        int last = -1;  // 假设上一跳节点不存在，传递-1
        double dx = static_cast<double>(destX);
        double dy = static_cast<double>(destY);
        return peri_nexthop(useGG, last, my_x, my_y, dx, dy);
    }

    // 否则，返回贪婪转发最近的邻居节点ID
    return closestNeighborId;
}


// 辅助函数：计算两点之间的距离
double GpsrRouting::calculateDistance(int x1, int y1, int x2, int y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

// 实现RNG平面化
std::vector<GPSR_neighborRecord> GpsrRouting::rng_planarize() {
    std::vector<GPSR_neighborRecord> result;
    for (const auto& [id, neighbor] : neighborTable) {
        double mdis = calculateDistance(my_x, my_y, neighbor.x, neighbor.y);
        bool isValid = true;
        for (const auto& [temp_id, temp] : neighborTable) {
            if (temp_id != id) {
                double tempdis1 = calculateDistance(my_x, my_y, temp.x, temp.y);
                double tempdis2 = calculateDistance(neighbor.x, neighbor.y, temp.x, temp.y);
                if (tempdis1 < mdis && tempdis2 < mdis) {
                    isValid = false;
                    break;
                }
            }
        }
        if (isValid) {
            result.push_back(neighbor);
        }
    }
    return result;
}

// 实现GG平面化
std::vector<GPSR_neighborRecord> GpsrRouting::gg_planarize() {
    std::vector<GPSR_neighborRecord> result;
    for (const auto& [id, neighbor] : neighborTable) {
        double midpx = my_x + (neighbor.x - my_x) / 2.0;
        double midpy = my_y + (neighbor.y - my_y) / 2.0;
        double mdis = calculateDistance(my_x, my_y, midpx, midpy);
        bool isValid = true;
        for (const auto& [temp_id, temp] : neighborTable) {
            if (temp_id != id) {
                double tempdis = calculateDistance(midpx, midpy, temp.x, temp.y);
                if (tempdis < mdis) {
                    isValid = false;
                    break;
                }
            }
        }
        if (isValid) {
            result.push_back(neighbor);
        }
    }
    return result;
}

// 计算两个点的角度
double GpsrRouting::angle(double x1, double y1, double x2, double y2) {
    double line_len = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    if (line_len == 0.0) {
        return -1.0;
    }
    double cos_theta = (x2 - x1) / line_len;
    double theta = std::acos(cos_theta);
    if (y2 < y1) {
        theta = 2 * PI - theta;
    }
    return theta;
}

// 判断两条线是否相交
bool GpsrRouting::intersect(int theother, double sx, double sy, double dx, double dy) {
    const auto& other = neighborTable[theother];
    double x1 = my_x, y1 = my_y;
    double x2 = other.x, y2 = other.y;
    double x3 = sx, y3 = sy;
    double x4 = dx, y4 = dy;

    double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (denom == 0) return false;

    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

    return (ua > 0 && ua < 1 && ub > 0 && ub < 1);
}

// 周边转发算法
int GpsrRouting::peri_nexthop(bool useGG, int last, double sx, double sy, double dx, double dy) {
    std::vector<GPSR_neighborRecord> planar_neighbors = useGG ? gg_planarize() : rng_planarize();

    double alpha;
    if (last != -1) {
        const auto& lastnb = neighborTable[last];
        alpha = angle(my_x, my_y, lastnb.x, lastnb.y);
    } else {
        alpha = angle(my_x, my_y, dx, dy);
    }

    double minangle = std::numeric_limits<double>::max();
    int nexthop = -1;

    for (const auto& neighbor : planar_neighbors) {
        if (neighbor.id != last) {
            double delta = angle(my_x, my_y, neighbor.x, neighbor.y) - alpha;
            if (delta < 0.0) {
                delta += 2 * PI;
            }
            if (delta < minangle) {
                minangle = delta;
                nexthop = neighbor.id;
            }
        }
    }

    if (planar_neighbors.size() > 1 && intersect(nexthop, sx, sy, dx, dy)) {
        return peri_nexthop(useGG, nexthop, sx, sy, dx, dy);
    }

    return nexthop;
}
