#pragma once

#include "ble_simulation/ble_simulation.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

namespace ble_simulation {

class BLE_SIMULATION_API VruApplication : public veins::DemoBaseApplLayer {
public:
    void initialize(int stage) override;

protected:
    simtime_t advInterval;
    simtime_t minAdvDelay;
    simtime_t maxAdvDelay;

    cMessage* sendAdvEvt;

protected:
    void advSend();

    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
};

} // namespace ble_simulation
