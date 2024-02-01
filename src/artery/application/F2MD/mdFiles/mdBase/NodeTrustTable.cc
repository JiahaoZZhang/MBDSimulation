/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    01/06/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "NodeTrustTable.h"

void NodeTrustTable::addNode(uint32_t newNodeID, StationTypeTrustInit stationType){
    NodeTrust newNodeTrust;

    newNodeTrust.stationID = newNodeID;
    if(stationType == StationTypeTrustInit_PassengerCar){
        newNodeTrust.TrustInit = 0.5;
        newNodeTrust.StationType = StationTypeTrustInit_PassengerCar;
    }else if(stationType == StationTypeTrustInit_RoadSideUnit){
        newNodeTrust.TrustInit = 0.8;
        newNodeTrust.StationType = StationTypeTrustInit_RoadSideUnit;
    }
    
    newNodeTrust.TrustOpinion = ObjectOpinion({newNodeTrust.TrustInit,0},1-newNodeTrust.TrustInit,{0.9,0.1});

    this->mNodeTrustTable.push_back(newNodeTrust);
}

void NodeTrustTable::delNode(uint32_t NodeID){
    for(const auto& i: this->mNodeTrustTable){
        if(i.stationID == NodeID){
            auto indx = find(mNodeTrustTable.begin(),mNodeTrustTable.end(),i);
            mNodeTrustTable.erase(indx);
            break;
        }
    }
}

void NodeTrustTable::setNodeTrust(uint32_t NodeID, int Type, double val){ 
    auto indx = find(mNodeTrustTable.begin(),mNodeTrustTable.end(),NodeID);
    if(indx != mNodeTrustTable.end()){
        auto position = distance(mNodeTrustTable.begin(),indx);
        switch (Type)
        {
        case 1:
            this->mNodeTrustTable[position].TrustInit = val;
            break;
        
        case 2:
            this->mNodeTrustTable[position].TrustEnvState = val;
            break;

        case 3:
            this->mNodeTrustTable[position].TrustPenality = val;
            break;

        case 4:
            this->mNodeTrustTable[position].TrustConflictEvent = val;
            break;

        case 5:
            this->mNodeTrustTable[position].TrustL1 = val;
            break;

        case 6:
            this->mNodeTrustTable[position].TrustL2 = val;
            break;

        case 7:
            this->mNodeTrustTable[position].TrustL3 = val;
            break;

        case 10:
            this->mNodeTrustTable[position].NodeType = (bool)val;
            break;

        // case 11:
        //     this->mNodeTrustTable[position].RCtime = val;
        //     break;

        default:
            break;
        }
    }
}

void NodeTrustTable::setNodeTrust(uint32_t NodeID, int Type, ObjectOpinion& op){ 
    auto indx = find(mNodeTrustTable.begin(),mNodeTrustTable.end(),NodeID);
    if(indx != mNodeTrustTable.end()){
        auto position = distance(mNodeTrustTable.begin(),indx);
        switch (Type)
        {
        case 8:
            this->mNodeTrustTable[position].TrustOpinion = op;
            break;
        
        case 9:
            this->mNodeTrustTable[position].RevisedOpinion = op;
            break;
        
        default:
            break;

        }
    }
}


void NodeTrustTable::updateNodeList(NodeTableCpm& mNodeTableCpm){
    auto sourceList = mNodeTableCpm.getNodesNum();
    std::cout << "sourceList size::" << sourceList << std::endl;
    if(this->mNodeTrustTable.size() != sourceList){
        for(int i =0 ; i < sourceList; i++){
            auto nodeID = mNodeTableCpm.getNodeID(i);
            auto inList = get<0>(include(mNodeTableCpm.getNodeID(i)));
            // std::cout << "node ID ::"<< mNodeTableCpm.getNodeID(i) << "; inList::" << inList << std::endl;
            if(inList){
                continue;
            }else{
                auto lcpm = mNodeTableCpm.getNodeHistoryAddr(nodeID)->getLatestCPMAddr();
                auto mStationType = (*lcpm)->cpm.cpmParameters.managementContainer.stationType;
                // std::cout << "mStationType 111 ::" << mStationType << std::endl;
                if(mStationType == StationType_passengerCar){
                    this->addNode(mNodeTableCpm.getNodeID(i),StationTypeTrustInit_PassengerCar);
                    // std::cout << "sucess 111 ::" << std::endl;
                }else if(mStationType == StationType_roadSideUnit){
                    this->addNode(mNodeTableCpm.getNodeID(i),StationTypeTrustInit_RoadSideUnit);
                    // std::cout << "sucess 222 ::" << std::endl;
                }
            }
        }
    }
}

std::tuple<bool,int> NodeTrustTable::include(uint32_t stationID){
    auto iter = find(this->mNodeTrustTable.begin(),this->mNodeTrustTable.end(),stationID);
    if(iter == this->mNodeTrustTable.end()){
        return make_tuple(false,-1);
    }
    auto dis = iter - this->mNodeTrustTable.begin();
    return make_tuple(true,dis);
}

vector<NodeTrust>& NodeTrustTable::getTrustTable(){
    return this->mNodeTrustTable;
}


NodeTrust* NodeTrustTable::getNodeTrust(uint32_t stationID){
    try{
        auto iter = find(mNodeTrustTable.begin(),mNodeTrustTable.end(),stationID);
        if(iter != mNodeTrustTable.end()){
            auto indx = distance(mNodeTrustTable.begin(),iter);
            return &mNodeTrustTable[indx];
        }
        string tempStr = "The NodeTrustTable has not the input ID :" + std::to_string(stationID) + "!!!";
        throw tempStr;
    }catch(string tempStr){
        std::cout << tempStr << std::endl; 
    }
}