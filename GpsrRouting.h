#ifndef GPSRROUTING_H_
#define GPSRROUTING_H_

#include <omnetpp.h>
#include <map>

using namespace omnetpp;

struct GPSR_neighborRecord {
    int id;
    double x;
    double y;
    simtime_t ts;
};

class GpsrRouting : public cSimpleModule {
public:
    GpsrRouting();
    virtual ~GpsrRouting();

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

    std::map<int, GPSR_neighborRecord> neighborTable;

    virtual int greedy_forwarding(int destX, int destY, bool useGG);
    virtual int peri_nexthop(bool useGG, int last, double sx, double sy, double dx, double dy);
    virtual double calculateDistance(int x1, int y1, int x2, int y2);
    virtual std::vector<GPSR_neighborRecord> rng_planarize();
    virtual std::vector<GPSR_neighborRecord> gg_planarize();
    virtual double angle(double x1, double y1, double x2, double y2);
    virtual bool intersect(int theother, double sx, double sy, double dx, double dy);

    virtual void finish();
};

#endif /* GPSRROUTING_H_ */
