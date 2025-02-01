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
    rssiDeviation = par("rssiDeviation").doubleValue();
    calibratedRssi = par("calibratedRssi").doubleValue();

    distanceMode = par("distanceMode").intValue();
    rssiTunableError = par("rssiTunableError").doubleValue();
    rssiTunableKalmanReduction = par("rssiTunableKalmanReduction").doubleValue();

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

double CarApplication::distanceEstimation(double& rssi, double oracleDistance)
{
    // Environmental constant
    double a = 2.0;

    // Option 0: Simulated log-distance path loss model
    // CalibratedRssi is heavily dependent on multiple simulation parameters, including broadcast power, antenna gain and
    // the SimplePathlossModel in config.xml.
    double estimatedDistanceCustom = pow(10, (calibratedRssi - rssi) / (10 * a));
    EV_TRACE << "Estimated distance from simulation data: " << estimatedDistanceCustom << " m" << std::endl;

    // Option 1: Non-tunable model
    double refRssi = calibratedRssi - (10*a) * log10(oracleDistance);
    EV_TRACE << "Calculated reference RSSI: " << refRssi << " dBm" << std::endl;
    double rssiDelta =  refRssi - rssi;
    //Sub-linear growth of RSSI deviation
    rssiDeviation = 2 + 2 * log10(oracleDistance);
    rssiDeviation = rssiDeviation * 0.27;
    EV_TRACE << "RSSI DEVIATION: " << rssiDeviation << " dBm" << std::endl;
    double correctedRssi = refRssi - uniform(0, rssiDeviation) - fabs(rssiDelta);
    double estimatedDistanceNonTunable = pow(10, (calibratedRssi - correctedRssi) / (10 * a));
    EV_TRACE << "Estimated distance from non-tunable model: " << estimatedDistanceNonTunable << " m" << std::endl;

    // Option 2: Tunable model
    // Latency for computation of Kalman Filter (to 27% reduction in rssi deviation after 5 measurements),
    // accuracy does not scale with further measurements.
    rssiDeviation = rssiTunableError;
    rssiDeviation = rssiDeviation * rssiTunableKalmanReduction;
    correctedRssi = refRssi - uniform(0, rssiDeviation) - fabs(rssiDelta);
    double estimatedDistanceTunable = pow(10, (calibratedRssi - correctedRssi) / (10 * a));
    EV_TRACE << "Estimated distance from tunable model: " << estimatedDistanceTunable << " m" << std::endl;

    double estimatedDistance = 0.0;
    switch(distanceMode)
    {
    case 0:
        estimatedDistance = estimatedDistanceCustom;
        break;
    case 1:
        estimatedDistance = estimatedDistanceNonTunable;
        break;
    case 2:
        estimatedDistance = estimatedDistanceTunable;
        break;
    }

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

    // Oracle Distance
    double oracleDistance = distanceOracle(ble->getNodeId());
    EV_TRACE << "Oracle distance: " << oracleDistance << " m" << std::endl;

    // Estimated Distance
    double estimatedDistance = distanceEstimation(rssi, oracleDistance);

    // Distance delta
    double distanceDelta = fabs(oracleDistance - estimatedDistance);
    EV_TRACE << "Distance delta: " << distanceDelta << " m" << std::endl;

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
    EV_TRACE << "Received RSSI: " << rssi << " dBm" << std::endl;

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
