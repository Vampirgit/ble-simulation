#pragma once

#include "ble_simulation/ble_simulation.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

namespace ble_simulation {

class BLE_SIMULATION_API CarApplication : public veins::DemoBaseApplLayer {
public:
    void initialize(int stage) override;

protected:
    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;

    double distanceOracle(int nodeId);
};

} // namespace ble_simulation
