/*******************************************************************************
 * @author  Joseph Kamel, Maxime Georges
 * @email   josephekamel@gmail.com, maxime.georges059@gmail.com
 * @date    10/06/2021
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __VEINS_LegacyChecks_H_
#define __VEINS_LegacyChecks_H_

#include <tuple>

#include <omnetpp.h>
#include "../mdBase/NodeTable.h"
#include "../mdReport/MDReport.h"
#include "../mdStats/MDStatistics.h"
#include "../mdSupport/networkLinksLib/LinkControl.h"
#include "../mdSupport/MDMLib.h"
#include "veins/modules/obstacle/ObstacleControl.h"
#include "veins/modules/obstacle/Obstacle.h"
#include <unordered_map>

using namespace veins;
using namespace omnetpp;

class LegacyChecks {

private:
    int version = 0;
    unsigned long myPseudonym;
    veins::Coord myPosition;
    veins::Coord mySpeed;
    veins::Coord mySize;
    veins::Coord myHeading;

    MDMLib mdmLib;

    double MAX_PLAUSIBLE_SPEED = 0;
    double MAX_PLAUSIBLE_ACCEL = 0;
    double MAX_PLAUSIBLE_DECEL = 0;

    LinkControl* LinkC;
    
    inet::MACAddress myId;

    F2MD_CAMParameters * params;

    std::unordered_map<int,veins::Coord> *realDynamicMap;

    double ProximityPlausibilityCheck(veins::Coord*testPosition, veins::Coord*myPosition, veins::Coord* myHeading);

    double RangePlausibilityCheck(veins::Coord*, veins::Coord*);
    double PositionConsistancyCheck(veins::Coord*, veins::Coord*, double);
    double SpeedConsistancyCheck(double, double, double);
    double PositionSpeedConsistancyCheck(veins::Coord*, veins::Coord *, double, double,
            double);
    double PositionSpeedMaxConsistancyCheck(veins::Coord*, veins::Coord *, double, double,
            double);
    double SpeedPlausibilityCheck(double);
    double IntersectionCheck(veins::Coord nodePosition1, veins::Coord nodeSize1, veins::Coord head1,
            veins::Coord nodePosition2, veins::Coord nodeSize2, veins::Coord head2,
            double deltaTime);
    double SuddenAppearenceCheck(veins::Coord*, veins::Coord*);
    double BeaconFrequencyCheck(double, double);
    double PositionPlausibilityCheck(veins::Coord*, double);
    double PositionHeadingConsistancyCheck(veins::Coord* curHeading,
            veins::Coord *curPosition, veins::Coord *oldPosition, double deltaTime,
            double curSpeed);

    void KalmanPositionSpeedConsistancyCheck(veins::Coord * curPosition,
            veins::Coord * curPositionConfidence, veins::Coord * curSpeed, veins::Coord * oldSpeed,
            veins::Coord * curSpeedConfidence, double time, Kalman_SVI * kalmanSVI,
            double retVal[]);

    void KalmanPositionSpeedScalarConsistancyCheck(veins::Coord * curPosition,veins::Coord * oldPosition,
            veins::Coord * curPositionConfidence, veins::Coord * curSpeed, veins::Coord * oldSpeed,
            veins::Coord * curSpeedConfidence, double time, Kalman_SC * kalmanSC,
            double retVal[]);

    double KalmanPositionConsistancyCheck(veins::Coord * curPosition, veins::Coord * oldPosition, veins::Coord * curPosConfidence,
             double time, Kalman_SI * kalmanSI);

    double KalmanPositionAccConsistancyCheck(veins::Coord * curPosition, veins::Coord * curSpeed, veins::Coord * curPosConfidence,
             double time, Kalman_SI * kalmanSI);

    double KalmanSpeedConsistancyCheck(veins::Coord * curSpeed, veins::Coord * oldSpeed, veins::Coord * curSpeedConfidence,
            double time, Kalman_SI * kalmanSI);

    InterTest MultipleIntersectionCheck(NodeTable * detectedNodes,
            BSM * bsm);

    void PrintBsmCheck(unsigned long senderPseudonym, BsmCheck bsmCheck);

public:

    LegacyChecks(int version, unsigned long myPseudonym, veins::Coord myPosition, veins::Coord mySpeed,
            veins::Coord myHeading, veins::Coord mySize, veins::Coord myLimits, LinkControl* LinkC, std::unordered_map<int,veins::Coord> *realDynamicMap, inet::MACAddress myId, F2MD_CAMParameters * params);
    BsmCheck CheckBSM(BSM * bsm, NodeTable * detectedNodes);

};

#endif
