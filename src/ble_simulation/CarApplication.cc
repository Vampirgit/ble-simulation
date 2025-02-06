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
    // Calculate average packages per second
    double pps = 0.0;
    if (lastPackages.size() >= 2)
    {
        simtime_t packageTimeSpan = lastPackages.back() - lastPackages.front();
        if (packageTimeSpan > 0)
        {
            pps = (lastPackages.size() - 1) / packageTimeSpan.dbl();
        }
    }

    EV_TRACE << "Average packages per second: " << pps << std::endl;

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
    // Logarithmic growth of RSSI deviation, constant deviation due to absorption and reflection (e.g. sensor positioning,
    // object shadowing etc.). Shadowing due to buildings added later.
    rssiDeviation = 2 + 6 * log10(oracleDistance);
    // Apply Kalman Filter with reduction to a max 27% depending on the package rate with exponential decay.
    double kalmanReduction = 1.0;
    if (pps > 1.0)
    {
        kalmanReduction = 0.9 * exp((log(0.27 / 0.9) / pow(4, 0.5)) * pow(pps - 1, 0.5));
    }
    EV_TRACE << "Filter reduction from non-tunable model: " << kalmanReduction << std::endl;
    rssiDeviation = rssiDeviation * kalmanReduction;
    EV_TRACE << "RSSI deviation from non-tunable model: " << rssiDeviation << " dBm" << std::endl;
    double correctedRssi = refRssi - uniform(-rssiDeviation, rssiDeviation) - fabs(rssiDelta);
    double estimatedDistanceNonTunable = pow(10, (calibratedRssi - correctedRssi) / (10 * a));
    EV_TRACE << "Estimated distance from non-tunable model: " << estimatedDistanceNonTunable << " m" << std::endl;

    // Option 2: Tunable model
    // rssiTunableError = 2 + 6 * log10(oracleDistance);
    if (pps > 1.0)
    {
        kalmanReduction = 0.9 * exp((log(rssiTunableKalmanReduction / 0.9) / pow(4, 0.5)) * pow(pps - 1, 0.5));
    }
    rssiDeviation = rssiTunableError * kalmanReduction;
    correctedRssi = refRssi - uniform(-rssiDeviation, rssiDeviation) - fabs(rssiDelta);
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

double CarApplication::distanceToIntersection(const veins::Coord& vruPosition, const Coord& vruSpeed, const double& distanceToVru)
{
    cModule* thisMobility = getParentModule()->getSubmodule("veinsmobility");
    TraCIMobility* thisMobilityModule = check_and_cast<TraCIMobility*>(thisMobility);

    Coord thisPosition = thisMobilityModule->getPositionAt(simTime());
    Coord directionToVru = vruPosition - thisPosition;
    Coord directionToVruNorm = directionToVru * (1.0 / directionToVru.length());
    Coord estimatedVruPosition = thisPosition + directionToVruNorm * distanceToVru;
    EV_TRACE << "Estimated Position of the VRU: " << estimatedVruPosition << std::endl;

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

    return distanceToIntersection;
}

void CarApplication::onWSA(DemoServiceAdvertisment* wsa)
{

}

void CarApplication::onWSM(BaseFrame1609_4* frame)
{
    outFile.open("results/simOutput.txt", std::ios::app);

    PhyToMacControlInfo* phyToMacControlInfo = check_and_cast<PhyToMacControlInfo*>(frame->getControlInfo());
    double rssi = check_and_cast<DeciderResult80211*>(phyToMacControlInfo->getDeciderResult())->getRecvPower_dBm();

    BleMessage* ble = check_and_cast<BleMessage*>(frame);

    // Oracle Distance
    double oracleDistance = distanceOracle(ble->getNodeId());
    EV_TRACE << "Oracle distance: " << oracleDistance << " m" << std::endl;

    int deciderCode = bleDecider(frame, rssi);
    if (deciderCode != 0)
    {
        EV_TRACE << "Message was rejected!" << std::endl;
        if (deciderCode == 1)
        {
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << -1 << "\n";
        }
        else if (deciderCode == 2)
        {
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << -2 << "\n";
        }
        outFile.close();
        return;
    }

    simtime_t currentTime = simTime();
    lastPackages.push_back(currentTime);
    if (lastPackages.size() > 5)
    {
        lastPackages.pop_front();
    }

    // Sender ID
    EV_TRACE << "Message received from: " << ble->getNodeId() << std::endl;

    // Packet Length
    EV_TRACE << "Packet length: " << ble->getByteLength() << " byte" << std::endl;

    // Estimated Distance
    double estimatedDistance = distanceEstimation(rssi, oracleDistance);

    // Distance delta
    double distanceDelta = fabs(oracleDistance - estimatedDistance);
    EV_TRACE << "Distance delta to oracle: " << distanceDelta << " m" << std::endl;

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
    Coord vruSpeed = ble->getSpeedVector();

    double realDistanceToIntersection= distanceToIntersection(vruPosition, vruSpeed, oracleDistance);
    double estimatedDistanceToIntersection= distanceToIntersection(vruPosition, vruSpeed, estimatedDistance);

    double mpsSpeed = thisMobilityModule->getSpeed();
    double kmhSpeed = mpsSpeed * 3.6;

    double stoppingDistance = (kmhSpeed/10) * 3 + (kmhSpeed/10) * (kmhSpeed/10);
    EV_TRACE << "Stopping Distance: " << stoppingDistance << "m" << std::endl;
    double brakingThreshold = stoppingDistance + 10;     // Safety buffer of 10 meters
    EV_TRACE << "Braking Threshold: " << brakingThreshold << "m" << std::endl;

    if (realDistanceToIntersection < stoppingDistance)
    {
        if (estimatedDistanceToIntersection < stoppingDistance)
        {
            EV_WARN << "COLLISION UNAVOIDABLE, SYSTEM CORRECTLY KNOWS!" << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 1 << "\n";
        }
        else if (estimatedDistanceToIntersection < brakingThreshold)
        {
            EV_WARN << "COLLISION UNAVOIDABLE, BUT SYSTEM THINKS IT CAN STILL BRAKE!" << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 2 << "\n";
        }
        else
        {
            EV_WARN << "COLLISION UNAVOIDABLE AND SYSTEM DOES NOT EVEN BRAKE!" << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 3 << "\n";
        }
    }
    else if (realDistanceToIntersection < brakingThreshold)
    {
        if (estimatedDistanceToIntersection < stoppingDistance)
        {
            EV_WARN << "COLLISION AVOIDABLE! BUT SYSTEM THINKS IT IS NOT!" << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 4 << "\n";
        }
        else if (estimatedDistanceToIntersection < brakingThreshold)
        {
            EV_WARN << "COLLISION AVOIDABLE! SYSTEM CORRECTLY BRAKING!" << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 5 << "\n";
        }
        else
        {
            EV_WARN << "COLLISION AVOIDABLE, BUT SYSTEM DOES NOT EVEN BRAKE!" << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 6 << "\n";
        }
    }
    else
    {
        if (estimatedDistanceToIntersection < stoppingDistance)
        {
            EV_WARN << "NO COLLISION! BUT SYSTEM THINKS IT IS UNAVOIDABLE!" << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 7 << "\n";
        }
        else if (estimatedDistanceToIntersection < brakingThreshold)
        {
            EV_WARN << "NO COLLISION! BUT SYSTEM STILL BRAKES!" << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 8 << "\n";
        }
        else
        {
            EV_WARN << "No potential for collision detected." << std::endl;
            outFile << simTime().format(-2) << ", " << oracleDistance << " m" << ", " << 0 << "\n";
        }
    }

    outFile.close();
}

int CarApplication::bleDecider(BaseFrame1609_4* frame, double& rssi)
{
    // Reason: Out of scanning window
    if (simTime() > lastScan + scanWindow)
    {
        EV_TRACE << "Message was not received during the scan window!" << std::endl;
        return 1;
    }

    // Reason: RSSI
    if (rssi <= minRssi)
    {
        EV_TRACE << "Message could not be decoded correctly, as the received power was too low!" << std::endl;
        EV_TRACE << "Received power: " << rssi << " dBm" << " is under minRssi: "<< minRssi << " dBm" << std::endl;
        return 2;
    }
    EV_TRACE << "Received RSSI: " << rssi << " dBm" << std::endl;

    return 0;
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
