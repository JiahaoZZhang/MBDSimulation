/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    01/06/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __NodeTrustTable__
#define __NodeTrustTable__

#include <vector>
#include <algorithm>
#include <iterator>
#include "NodeTypeClassification.h"
#include "../mdSupport/SubjectiveLogicLib/ObjectOpinion.h"

using namespace std;

typedef enum StationTypeTrustInit{
    StationTypeTrustInit_PassengerCar,
    StationTypeTrustInit_RoadSideUnit
}TrustStationType;

struct NodeTrust
{
    uint32_t stationID;
    double TrustInit;
    double TrustEnvState=0;
    double TrustPenality=0;
    double TrustConflictEvent=0;
    double TrustL1=0;
    double TrustL2=0;
    double TrustL3=0;
    ObjectOpinion TrustOpinion = ObjectOpinion();
    ObjectOpinion RevisedOpinion = ObjectOpinion();
    int InConflict = 0; // 0-No ; 1+ -Yes 
    bool NodeType = 0; // 0 invalid; 1 valid
    int LocalBonus = 0; // 01 local perceived ;  10 LifeTime created

    int RCtime = 0; // valid reception times
    int LostTime = 0; // Lost times
    int InvalidTime = 0; // Change to invalid times
    TrustStationType StationType = StationTypeTrustInit_PassengerCar;

    bool operator == (const NodeTrust& e) const{
        return this->stationID == e.stationID;
    }
    
    bool operator == (const uint32_t e) const{
        return this->stationID == e;
    }
};


class NodeTrustTable
{
    private:
        vector<NodeTrust> mNodeTrustTable;
        
        

    public:
        NodeTrustTable(){};

        void updateNodeList(NodeTableCpm& mNodeTableCpm);

        void addNode(uint32_t newNodeID, StationTypeTrustInit stationType);
        void delNode(uint32_t NodeID);

        void setNodeTrust(uint32_t NodeID, int Type, double val);
        void setNodeTrust(uint32_t NodeID, int Type, ObjectOpinion& op);

        vector<NodeTrust>& getTrustTable();
        NodeTrust* getNodeTrust(uint32_t stationID);

        std::tuple<bool,int> include(uint32_t stationID);
        
        ~NodeTrustTable(){};
};


#endif