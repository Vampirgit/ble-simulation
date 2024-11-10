#include "CarApplication.h"

#include "ble_simulation/ApplicationLayerTestMessage_m.h"

using namespace veins;
using namespace ble_simulation;

Define_Module(ble_simulation::CarApplication);

void CarApplication::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
}

void CarApplication::onWSA(DemoServiceAdvertisment* wsa)
{

}

void CarApplication::onWSM(BaseFrame1609_4* frame)
{

}

void CarApplication::handleSelfMsg(cMessage* msg)
{

}

void CarApplication::handlePositionUpdate(cObject* obj)
{

}
