/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    18/07/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "TrustManagementFunc.h"

double TrustManagementFunc::AssessmentFunction(double n, double a, double b, double y){
    double val = a / (1 + exp(b*n + y));
    return val;
}

double TrustManagementFunc::PenaltyFunction1(double n, double a, double b, double min , double max){
    double val = a*n + b;

    if(val <= min){
        return min;
    }else if(val >= max){
        return max;
    }else{
        return val;
    }
}

double TrustManagementFunc::PenaltyFunction2(double n, double a, double b, double y, double valAF){
    double val = (-a) / (1 + exp(b*n + y));
    if(val <= (-valAF)){
        return (-valAF);
    }else{
        return val;
    }
}


/**
 * @brief XY distance convert
 * 
 * @param Delta_xPos 
 * @param Delta_yPos 
 * @param Heading radian
 * @return Vector2d 
 */
Eigen::Vector2d TrustManagementFunc::XYDistanceConvert(double Delta_xPos, double Delta_yPos, double Heading){
    Eigen::MatrixXd m1(2,2);
    Eigen::MatrixXd m2(2,2);
    Eigen::Vector2d obj_XY(Delta_xPos,Delta_yPos);
    m1 <<  cos(Heading),  -sin(Heading),
          sin(Heading),   cos(Heading);

    m2 << 0, 1,
        -1, 0;
    
    auto res = m1*(m2*obj_XY);
    return res;
 }


/**
 * @brief XY speed convert
 * 
 * @param Xspeed 
 * @param Yspeed 
 * @param Heading radian
 * @return Eigen::Vector2d 
 */
// Eigen::Vector2d TrustManagementFunc::XYSpeedConvert(double Xspeed, double Yspeed, double Heading){
//     Eigen::MatrixXd m1(2,2);
//     Eigen::MatrixXd m2(2,2);
//     Eigen::Vector2d obj_XY(Xspeed,Yspeed);
//     m1 <<  cos(Heading),  -sin(Heading),
//           sin(Heading),   cos(Heading);

//     m2 << 0, 1,
//         -1, 0;
    
//     auto res = m1*(m2*obj_XY);
//     return res;
//  }



/**
 * @brief inv relative Perceived object XY convert, cartesian plan
 * 
 * @param xDistance 
 * @param yDistance 
 * @param refX 
 * @param refY 
 * @param Heading radian
 * @return Eigen::Vector2d 
 */
Eigen::Vector2d TrustManagementFunc::InvXYConvert(double xDistance, double yDistance,double refX, double refY, double Heading){
    Eigen::MatrixXd m1(2,2);
    Eigen::MatrixXd m2(2,2);
    Eigen::Vector2d res;
    Eigen::Vector2d obj_XY(xDistance,yDistance);
    m1 <<  cos(Heading),   sin(Heading),
         -sin(Heading),   cos(Heading);
    m2 << 0, -1,
        1, 0;

    
    auto temp = m1*m2*obj_XY;
    
    res << temp(0) + refX,temp(1) + refY;

    return res;
 }




/**
 * @brief Calculate car's 4 corner position
 * 
 * @param car_xPos Car centric x Position
 * @param car_yPos Car centric y Position
 * @param Heading Heading of this car
 * @return a list of 4 corner Position
 */
 std::vector<Position> TrustManagementFunc::PosOfFourCorners(double car_xPos, double car_yPos, double Heading){
    
    double Delta_xPos, Delta_yPos;
    Eigen::MatrixXd RotationMatrix(3,3);
    Delta_xPos = 1;
    Delta_yPos = 1.5;
    Eigen::VectorXd Corn_XY(3);
    std::vector<Position> ListCornerPos;

    RotationMatrix <<  cos(Heading), sin(Heading), car_xPos,
                      -sin(Heading), cos(Heading), car_yPos,
                                  0,            0,        1;

    for(int i=0; i<4; i++){
        if(i == 0){
            Corn_XY << Delta_xPos, Delta_yPos, 1;
            auto res = RotationMatrix*Corn_XY;
            double r1 = res(0);
            double r2 = res(1);
            Position CornerPos(r1, r2);
            ListCornerPos.push_back(CornerPos);
       }; 

        if(i == 1){
            Corn_XY << Delta_xPos, -Delta_yPos, 1;
            auto res = RotationMatrix*Corn_XY;
            double r1 = res(0);
            double r2 = res(1);
            Position CornerPos(r1, r2);
            ListCornerPos.push_back(CornerPos);
       };

        if(i == 2){
            Corn_XY << -Delta_xPos, -Delta_yPos, 1;
            auto res = RotationMatrix*Corn_XY;
            double r1 = res(0);
            double r2 = res(1);
            Position CornerPos(r1, r2);
            ListCornerPos.push_back(CornerPos);
       };

        if(i == 3){
            Corn_XY << -Delta_xPos, Delta_yPos, 1;
            auto res = RotationMatrix*Corn_XY;
            double r1 = res(0);
            double r2 = res(1);
            Position CornerPos(r1, r2);
            ListCornerPos.push_back(CornerPos);
       };
    }

    return ListCornerPos;


 }




std::vector<std::shared_ptr<artery::EnvironmentModelObstacle>> TrustManagementFunc::ObstacleSelectedByArea
(GlobalEnvironmentModel& mGlobalEnvironmentModel, double SelectedRange, double x, double y){
        double x1 = x + SelectedRange;
        double x2 = x - SelectedRange;
        double y1 = y + SelectedRange;
        double y2 = y - SelectedRange;
        Position AreaPos1(x1,y1);
        Position AreaPos2(x1,y2);
        Position AreaPos3(x2,y2);
        Position AreaPos4(x2,y1);
        std::vector<artery::Position> SelectedArea = {AreaPos1,AreaPos2,AreaPos3,AreaPos4,AreaPos1};
        auto PreselectObstacle = mGlobalEnvironmentModel.preselectObstacles(SelectedArea);
        return PreselectObstacle;
}





 bool TrustManagementFunc::CheckTargetHideByObject(std::vector<Position> ListTargetCorner, 
 std::vector<Position> ObjectOutline, Position egoPos){
    for(const auto& CornPos:ListTargetCorner){
        boost::geometry::model::segment cornLine{egoPos, CornPos};
        if(!boost::geometry::intersects(cornLine,ObjectOutline)){
            return false;
        }
    }
    return true;
 }

 

