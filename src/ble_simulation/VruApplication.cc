#include "VruApplication.h"

#include "ble_simulation/BleMessage_m.h"

using namespace veins;
using namespace ble_simulation;

Define_Module(ble_simulation::VruApplication);

void VruApplication::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    isActive = par("isActive");
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
    if (!isActive)
        return;

    BleMessage* wsm = new BleMessage();
    populateWSM(wsm);
    wsm->setNodeId(getParentModule()->getId());
    sendDown(wsm);
}
