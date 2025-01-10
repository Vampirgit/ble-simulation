#pragma once

#include "ble_simulation/ble_simulation.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

namespace ble_simulation {

class BLE_SIMULATION_API CarApplication : public veins::DemoBaseApplLayer {
public:
    void initialize(int stage) override;

protected:
    simtime_t scanInterval;
    simtime_t scanWindow;

    double minRssi;
    double rssiVariance;
    double calibratedRssi;

    simtime_t lastScan;

    cMessage* scanEvt;

    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    bool bleDecider(veins::BaseFrame1609_4* frame, double& rssi);

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;


    double distanceOracle(int nodeId);
    double distanceEstimation(double& rssi);
};

} // namespace ble_simulation
