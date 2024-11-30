#include "CarApplication.h"

#include "ble_simulation/BleMessage_m.h"

using namespace veins;
using namespace ble_simulation;

Define_Module(ble_simulation::CarApplication);

void CarApplication::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
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
    BleMessage* wsm = check_and_cast<BleMessage*>(frame);

    EV_TRACE << "Message received from: " << wsm->getNodeId() << std::endl;
    EV_TRACE << "Oracle distance: " << distanceOracle(wsm->getNodeId()) << std::endl;
}

void CarApplication::handleSelfMsg(cMessage* msg)
{

}

void CarApplication::handlePositionUpdate(cObject* obj)
{

}
