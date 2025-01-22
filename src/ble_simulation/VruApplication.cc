#include "VruApplication.h"

#include "ble_simulation/BleMessage_m.h"

using namespace veins;
using namespace ble_simulation;

Define_Module(ble_simulation::VruApplication);

void VruApplication::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    advInterval = par("advInterval").doubleValue();
    minAdvDelay = par("minAdvDelay").doubleValue();
    maxAdvDelay = par("maxAdvDelay").doubleValue();

    if (stage == 0)
    {
        sendAdvEvt = new cMessage("sendAdvEvt");
        scheduleAt(simTime() + SimTime(2, SIMTIME_S), sendAdvEvt);
    }
}

void VruApplication::advSend()
{
    // Calculate speed vector (in practice, calculated over multiple frames using different types of sensors e.g. GPS etc.)
    cModule* thisMobility = getParentModule()->getSubmodule("veinsmobility");
    BaseMobility* thisMobilityModule = check_and_cast<BaseMobility*>(thisMobility);
    Coord speedVector = thisMobilityModule->getCurrentSpeed();

    EV_TRACE << "VRU current speed vector: " << speedVector << std::endl;

    BleMessage* ble = new BleMessage();
    populateWSM(ble);
    ble->setByteLength(31);
    ble->setSpeedVector(speedVector);
    ble->setNodeId(getParentModule()->getId());
    sendDown(ble);
}

void VruApplication::onWSA(DemoServiceAdvertisment* wsa)
{

}

void VruApplication::onWSM(BaseFrame1609_4* frame)
{

}

void VruApplication::handleSelfMsg(cMessage* msg)
{
    if (!strcmp(msg->getName(), "sendAdvEvt"))
    {
        advSend();
        scheduleAt(simTime() + advInterval + uniform(minAdvDelay.dbl(), maxAdvDelay.dbl()), sendAdvEvt);
    }
}

void VruApplication::handlePositionUpdate(cObject* obj)
{

}
