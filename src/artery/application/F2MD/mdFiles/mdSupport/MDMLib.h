/*******************************************************************************
 * @author  Joseph Kamel
 * @email   josephekamel@gmail.com
 * @date    28/11/2018
 * @version 2.0
 *
 * SCA (Secure Cooperative Autonomous systems)
 * Copyright (c) 2013, 2018 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __VEINS_MDMLib_H_
#define __VEINS_MDMLib_H_

#include <omnetpp.h>
#include "F2MD_msgs/BSM_m.h"
#include "ellipseIntLib/EllipseIntLib.h"
#include "rectIntLib/RectIntLib.h"
#include "veins/base/utils/Coord.h"
#include "traci/sumo/utils/traci/TraCIAPI.h"

using namespace veins;

class MDMLib {

private:
    void countCircles(double rc, double rl, double rs);
    double calculateCircles(double dl, double ds);

    double importanceFactor(double r1, double r2, double d);

public:
    double gaussianSum(double x, double sig);
    double boundedGaussianSum(double x1, double x2, double sig);

    double calculateDistancePtr(veins::Coord *, veins::Coord *);
    double calculateSpeedPtr(veins::Coord * Speed);
    double calculateHeadingAnglePtr(veins::Coord * heading);

//     double calculateDistance(veins::Coord pos1, veins::Coord pos2);

    template<typename T>    
    double calculateDistance(T& pos1, T& pos2){return sqrt(pow(pos1.x - pos2.x, 2.0) + pow(pos1.y - pos2.y, 2.0));}


    double calculateSpeed(veins::Coord Speed);
    double calculateHeadingAngle(veins::Coord heading);
    double calculateDistanceXY(double xDistance, double yDistance);
    double calculateSpeedXY(double xSpeed, double ySpeed);

    double calculatePolynom(long double coof[], const int coofNum, double x);

    void calculateMaxMinDist(double curSpeed, double oldspeed, double time,
            double MAX_PLAUSIBLE_ACCEL, double MAX_PLAUSIBLE_DECEL,
            double MAX_PLAUSIBLE_SPEED, double * returnDistance);

    double calculateDeltaTime(BSM * bsm1,
            BSM * bsm2);
    double calculateCircleSegment(double radius, double intDistance);
    double calculateCircleCircleIntersection(double r1, double r2, double d);
    double SegmentSegmentFactor(double d, double r1, double r2, double range);
    double CircleSegmentFactor(double d, double r1, double r2, double range);
    double CircleCircleFactor(double d, double r1, double r2, double range);
    double OneSidedCircleSegmentFactor(double d, double r1, double r2,
            double range);
    double CircleIntersectionFactor(double conf1, double conf2, double d,
            double initRadius);

    double RectRectFactor(veins::Coord c1, veins::Coord c2, double heading1, double heading2,
            veins::Coord size1, veins::Coord size2);

    double EllipseEllipseIntersectionFactor(veins::Coord pos1, veins::Coord posConf1,
            veins::Coord pos2, veins::Coord posConf2, double heading1, double heading2,
            veins::Coord size1, veins::Coord size2);

    double SafeAcos(double x);

};

#endif
