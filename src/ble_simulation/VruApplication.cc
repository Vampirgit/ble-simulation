#include "VruApplication.h"

#include "ble_simulation/ApplicationLayerTestMessage_m.h"

using namespace veins;
using namespace ble_simulation;

Define_Module(ble_simulation::VruApplication);

void VruApplication::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
}

void VruApplication::onWSA(DemoServiceAdvertisment* wsa)
{

}

void VruApplication::onWSM(BaseFrame1609_4* frame)
{

}

void VruApplication::handleSelfMsg(cMessage* msg)
{

}

void VruApplication::handlePositionUpdate(cObject* obj)
{

}
