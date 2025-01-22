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

    minRssi = par("minRssi").doubleValue();
    rssiVariance = par("rssiVariance").doubleValue();
    calibratedRssi = par("calibratedRssi").doubleValue();

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

double CarApplication::distanceEstimation(double& rssi)
{
    // Measurement accuracy modeling
    rssi = rssi + uniform(-rssiVariance, rssiVariance);

    // According to Log-distance path loss model: Distance = 10 ^ ((Power at one meter - RSSI)/(10 * N));
    // Environment factor N = 2, because open space.
    // A more accurate function would need the reference RSSI to be calculated at different distances
    // Technically it is better to use the median RSSI with a Kalman filter, but for simplicity:
    double estimatedDistance = pow(10, (calibratedRssi - rssi) / (10 * 2));

    return estimatedDistance;
}

void CarApplication::onWSA(DemoServiceAdvertisment* wsa)
{

}

void CarApplication::onWSM(BaseFrame1609_4* frame)
{
    PhyToMacControlInfo* phyToMacControlInfo = check_and_cast<PhyToMacControlInfo*>(frame->getControlInfo());
    double rssi = check_and_cast<DeciderResult80211*>(phyToMacControlInfo->getDeciderResult())->getRecvPower_dBm();

    if (!bleDecider(frame, rssi))
    {
        EV_TRACE << "Message was rejected!" << std::endl;
        return;
    }

    BleMessage* ble = check_and_cast<BleMessage*>(frame);

    // Sender ID
    EV_TRACE << "Message received from: " << ble->getNodeId() << std::endl;

    // Packet Length
    EV_TRACE << "Packet length: " << ble->getByteLength() << " byte" << std::endl;

    // Estimated Distance
    double estimatedDistance = distanceEstimation(rssi);
    EV_TRACE << "Estimated distance: " << estimatedDistance << " m" << std::endl;

    // Oracle Distance
    double oracleDistance = distanceOracle(ble->getNodeId());
    EV_TRACE << "Oracle distance: " << oracleDistance << " m" << std::endl;

    // Distance delta
    double distanceDelta = fabs(oracleDistance - estimatedDistance);
    EV_TRACE << "Distance delta: " << distanceDelta << " m" << std::endl;

    // RSSI Info
    EV_TRACE << "Measured power: " << rssi << " dBm" << std::endl;

    // Collision warning
    /* It is not impossible to determine (or estimate) the VRU's position using only the speed vector of VRU, the car and
     * the estimated distance to the VRU. In practice, this would require extrapolating the direction
     * of the VRU out of these values over multiple time points. For simplicity, we will just assume that we already have this
     * angle, although the estimated distance will still have an effect on the accuracy of the prediction.     */
    cModule* thisMobility = getParentModule()->getSubmodule("veinsmobility");
    TraCIMobility* thisMobilityModule = check_and_cast<TraCIMobility*>(thisMobility);
    cModule* targetNode = getSimulation()->getModule(ble->getNodeId());
    cModule* targetMobility = targetNode->getSubmodule("veinsmobility");
    BaseMobility* targetMobilityModule = check_and_cast<BaseMobility*>(targetMobility);

    Coord vruPosition = targetMobilityModule->getPositionAt(simTime());
    Coord thisPosition = thisMobilityModule->getPositionAt(simTime());
    Coord directionToVru = vruPosition - thisPosition;
    Coord directionToVruNorm = directionToVru * (1.0 / directionToVru.length());
    Coord estimatedVruPosition = thisPosition + directionToVruNorm * estimatedDistance;

    EV_TRACE << "Estimated Position of the VRU: " << estimatedVruPosition << std::endl;

    Coord vruSpeed = ble->getSpeedVector();
    Coord carSpeed = thisMobilityModule->getHeading().toCoord() * thisMobilityModule->getSpeed();

    Coord carNormal = Coord(-carSpeed.y, carSpeed.x, 0);
    Coord relativePosition = thisPosition - estimatedVruPosition;
    double numerator = carNormal * relativePosition;
    double denominator = carNormal * vruSpeed;
    double intersectionTime = numerator / denominator;

    Coord carPositionAtIntersection = thisPosition + carSpeed * intersectionTime;
    Coord intersectionPoint = estimatedVruPosition + vruSpeed * intersectionTime;
    double distanceToIntersection = (intersectionPoint - thisPosition).length();

    EV_TRACE << "Intersection Point: " << intersectionPoint << std::endl;
    EV_TRACE << "Distance to Intersection: " << distanceToIntersection << "m" << std::endl;

    double mpsSpeed = thisMobilityModule->getSpeed();
    double kmhSpeed = mpsSpeed * 3.6;

    double stoppingDistance = (kmhSpeed/10) * 3 + (kmhSpeed/10) * (kmhSpeed/10);
    EV_TRACE << "Stopping Distance: " << stoppingDistance << "m" << std::endl;

    double brakingThreshold = stoppingDistance + 10;
    EV_TRACE << "Braking Threshold: " << brakingThreshold << "m" << std::endl;

    if (distanceToIntersection < stoppingDistance)
    {
        EV_WARN << "COLLISION INBOUND!" << std::endl;
    }
    else if (distanceToIntersection < brakingThreshold)
    {
        EV_WARN << "AUTOMATIC BRAKING SYSTEM ENGAGED!" << std::endl;
    }
    else
    {
        EV_TRACE << "No potential for collision detected. " << std::endl;
    }
}

bool CarApplication::bleDecider(BaseFrame1609_4* frame, double& rssi)
{
    // Reason: Out of scanning window
    if (simTime() > lastScan + scanWindow)
    {
        EV_TRACE << "Message was not received during the scan window!" << std::endl;
        return false;
    }

    // Reason: RSSI
    if (rssi <= minRssi)
    {
        EV_TRACE << "Message could not be decoded correctly, as the received power was too low!" << std::endl;
        EV_TRACE << "Received power: " << rssi << " dBm" << " is under minRssi: "<< minRssi << " dBm" << std::endl;
        return false;
    }
    EV_TRACE << "Received power: " << rssi << " dBm" << std::endl;

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
