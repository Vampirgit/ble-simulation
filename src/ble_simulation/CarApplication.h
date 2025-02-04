#pragma once

#include "ble_simulation/ble_simulation.h"

#include <fstream>
#include <deque>
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

namespace ble_simulation {

class BLE_SIMULATION_API CarApplication : public veins::DemoBaseApplLayer {
public:
    void initialize(int stage) override;

protected:
    std::ofstream outFile;

    simtime_t scanInterval;
    simtime_t scanWindow;

    std::deque<simtime_t> lastPackages;

    double minRssi;
    double rssiDeviation;
    double calibratedRssi;

    int distanceMode;
    double rssiTunableError;
    double rssiTunableKalmanReduction;

    simtime_t lastScan;

    cMessage* scanEvt;

    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    int bleDecider(veins::BaseFrame1609_4* frame, double& rssi);
    void collisionPredictor(veins::BaseFrame1609_4* frame);

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;

    double distanceOracle(int nodeId);
    double distanceEstimation(double& rssi, double oracleDistance);
    double distanceToIntersection(const veins::Coord& vruPosition, const veins::Coord& vruSpeed, const double& distanceToVru);
};

} // namespace ble_simulation
