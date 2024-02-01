/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    18/07/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __TrustManagementFunc__
#define __TrustManagementFunc__

#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <stdint.h>
#include "artery/utility/Geometry.h"
#include "artery/envmod/GlobalEnvironmentModel.h"


using namespace Eigen;
using namespace std;
using namespace artery;


class TrustManagementFunc
{
private:
    /* data */
public:
    TrustManagementFunc(){};

    double AssessmentFunction(double n, double a, double b, double y);

    // Lost target penalty function
    double PenaltyFunction1(double n, double a, double b, double min , double max); 

    // Valid source changes to invalid 
    double PenaltyFunction2(double n, double a, double b, double y, double valAF);


    // Referencce changed, XY distance convert
    Eigen::Vector2d XYDistanceConvert(double Delta_xPos, double Delta_yPos, double Heading);

    // Referencce changed, XY speed convert
    // Eigen::Vector2d XYSpeedConvert(double Xspeed, double Yspeed, double Heading);

    // Inverse, convert the relatived XY distance to XY Position
    Eigen::Vector2d InvXYConvert(double xDistance, double yDistance,double refX, double refY, double Heading);

    // The Four corner' position of a car, length = 3m, width = 2m
    std::vector<Position> PosOfFourCorners(double car_xPos, double car_yPos, double Heading);


    std::vector<std::shared_ptr<artery::EnvironmentModelObstacle>> ObstacleSelectedByArea(GlobalEnvironmentModel& mGlobalEnvironmentModel, 
    double SelectedRange, double x, double y);

    bool CheckTargetHideByObject(std::vector<Position> ListTargetCorner, 
            std::vector<Position> ObjectOutline, Position egoPos);

    ~TrustManagementFunc(){};
};


#endif