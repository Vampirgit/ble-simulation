#include "CarApplication.h"

#include "ble_simulation/BleMessage_m.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/phy/DeciderResult80211.h"

using namespace veins;
using namespace ble_simulation;

Define_Module(ble_simulation::CarApplication);

void CarApplication::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    scanInterval = par("scanInterval").doubleValue();
    scanWindow = par("scanWindow").doubleValue();

    if (stage == 0)
    {
        scanEvt = new cMessage("scanEvt");
        scheduleAt(simTime() + SimTime(1, SIMTIME_S), scanEvt);
    }
}

double CarApplication::distanceOracle(int nodeId)
{
    cModule* targetNode = getSimulation()->getModule(nodeId);
    cModule* targetMobility = targetNode->getSubmodule("veinsmobility");
    BaseMobility* targetMobilityModule = check_and_cast<BaseMobility*>(targetMobility);
    Coord targetPosition = targetMobilityModule->getPositionAt(simTime());

    cModule* thisMobility = getParentModule()->getSubmodule("veinsmobility");
    BaseMobility* thisMobilityModule = check_and_cast<BaseMobility*>(thisMobility);
    Coord thisPosition = thisMobilityModule->getPositionAt(simTime());

    return thisPosition.distance(targetPosition);

}

void CarApplication::onWSA(DemoServiceAdvertisment* wsa)
{

}

void CarApplication::onWSM(BaseFrame1609_4* frame)
{
    if (!bleDecider(frame))
    {
        EV_TRACE << "Message was rejected!" << std::endl;
        return;
    }

    BleMessage* ble = check_and_cast<BleMessage*>(frame);

    // Sender ID
    EV_TRACE << "Message received from: " << ble->getNodeId() << std::endl;

    // Packet Length
    EV_TRACE << "Packet length: " << ble->getByteLength() << " byte" << std::endl;

    // Distance
    EV_TRACE << "Oracle distance: " << distanceOracle(ble->getNodeId()) << " m" << std::endl;

    // RSSI
    PhyToMacControlInfo* phyToMacControlInfo = check_and_cast<PhyToMacControlInfo*>(frame->getControlInfo());
    double recvPower = check_and_cast<DeciderResult80211*>(phyToMacControlInfo->getDeciderResult())->getRecvPower_dBm();
    EV_TRACE << "Received power: " << recvPower << " dBm" << std::endl;

}

bool CarApplication::bleDecider(BaseFrame1609_4* frame)
{
    // Reason: Out of scanning window
    if (simTime() > lastScan + scanWindow)
    {
        EV_TRACE << "Message was not received during the scan window!" << std::endl;
        return false;
    }

    return true;
}

void CarApplication::handleSelfMsg(cMessage* msg)
{
    if (!strcmp(msg->getName(), "scanEvt"))
    {
        lastScan = simTime();
        scheduleAt(simTime() + scanInterval, scanEvt);
    }
}

void CarApplication::handlePositionUpdate(cObject* obj)
{

}
