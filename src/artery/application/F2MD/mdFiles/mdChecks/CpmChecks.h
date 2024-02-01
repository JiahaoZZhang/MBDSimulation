/*******************************************************************************
 * @author  Jiahao ZHANG
 * @email   jiahao.zhang96@gmail.com
 * @date    09/11/2022
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

/*
# This file contain all the methods to perform checks and detect attacks on the CPMs
*/

#ifndef __CPMCHECKS_H_
#define __CPMCHECKS_H_

#include <omnetpp.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <stdint.h>
#include <boost/geometry.hpp>
#include <vanetza/asn1/cpm.hpp>
#include "../../F2MD_CPParameters.h"
#include "../mdBase/NodeTableCpm.h"
#include "../mdBase/NodeHistoryCpm.h"
#include "../mdBase/CpmCheckList.h"
#include "../mdSupport/MDMLib.h"
#include "../mdSupport/kalmanLib/Kalman_SVI.h"
#include "../mdSupport/kalmanLib/Kalman_SI.h"
#include "artery/envmod/Geometry.h"
#include "artery/traci/VehicleController.h"

namespace bg = boost::geometry;
using namespace artery;
using boost::geometry::get;
using namespace traci;
using namespace Eigen;


struct KalmanFilterList
{
    uint32_t senderID;
    uint32_t PerceivedObjectID;
    Kalman_SI mKalmanPC, mKalmanSC;
    Kalman_SVI mKalmanPSC;
    bool operator==(const uint32_t a[2]){
        return ((this->senderID == a[0]) && (this->PerceivedObjectID == a[1]));
    }

    KalmanFilterList operator=(const KalmanFilterList& e){
        this->senderID = e.senderID;
        this->PerceivedObjectID = e.PerceivedObjectID;
        this->mKalmanPC = e.mKalmanPC;
        this->mKalmanSC = e.mKalmanSC;
        this->mKalmanPSC = e.mKalmanPSC;
        return *this;
    }
};




class CpmChecks {

private:
    MDMLib mdmLib;
    F2MD_CPParameters* params;

    
    double distancePlausibility(double xDistance, double yDistance, double heading, 
                                    double length, double width, SensorInformationContainer_t& sensors);
    
    double speedPlausibility(double xSpeed, double ySpeed);
    double speedPlausibility(double Speed);

    double speedConsistency(double curXSpeed, double curYSpeed, double oldXSpeed, double oldYSpeed, double time);   

    double positionSpeedConsistency(double curXDistance, double curYDistance, double curSpeed,
                                double oldXDistance, double oldYDistance, double oldSpeed, double time);

    double KalmanPositionConsistency(double curXPosition, double curYPosition, double oldXPosition, double oldYPosition,
                                    double curXPostionConfidence, double curYPostionConfidence, double time, Kalman_SI* kalmanSI);

    double KalmanSpeedConsistancy(double curSpeed, double curAccel, double curSpeedDirection, double curSpeedConfidence,
                                                double oldSpeedDirection, double time, Kalman_SI* kalmanSI);

    void KalmanPositionSpeedConsistancy(double curXPosition, double curYPosition, double curXPositionConfidence, double curYPositionConfidence, 
                                                double curSpeed, double curAccel, double curSpeedConfidence, double curSpeedDirection, double oldSpeedDirection,
                                                double time, Kalman_SVI* kalmanSVI, double retVal[]);


    PerceivedObject_t* findPoContainerAddr(int objectID, vanetza::asn1::Cpm&);
    tuple<bool, KalmanFilterList*> FindKalmanFilter(uint32_t senderID, uint32_t objectID);

    NodeTableCpm* detectedNodes;  

    vector<KalmanFilterList> KalmanListTable;
    KalmanFilterList newKalmanFilter;

public:
    CpmChecks(){};
    CpmChecks(F2MD_CPParameters* params);

    double CpmChecksum(const vanetza::asn1::Cpm&, NodeTableCpm *detectedNodes, CpmCheckList& mCpmCheckList, std::shared_ptr<const traci::API>* TraciLiteAPI);

    bool IsPointInPolygon(artery::geometry::Point pt, artery::geometry::Polygon polygon);

};

#endif