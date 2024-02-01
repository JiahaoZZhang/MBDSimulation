/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    02/06/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "ConflictEventTrustRevision.h"

void ConflictEventTrustRevision::update(NodeTrustTable* NTT,OpinionConstruction& OpinionConstruction, ConflictEventTable& CET){
    // this->mOpinionConstruction = &OpinionConstruction;
    // this->mNodeTrustTable = &NTT;

    // for(const auto& i : OpinionConstruction.getConflictEventOpinionsTable()){
    //     auto iter = find(mConflictEventNodeTrustRevision.begin(),mConflictEventNodeTrustRevision.end(),i.ConflictEventID);
    //     if(iter != mConflictEventNodeTrustRevision.end()){
    //         auto indx = distance(mConflictEventNodeTrustRevision.begin(),iter);
    //         calTR(i.ConflictEventID);
    //     }
    // }

    updateConflictEvent(NTT,OpinionConstruction,CET);
}

void ConflictEventTrustRevision::updateStation(uint32_t EventID,NodeTrustTable* NTT,OpinionConstruction& OpinionConstruction){
    auto index = EventIndx(EventID);
    auto ConflictEventStationOpinions = OpinionConstruction.getConflictEventStationOpinions(EventID);
    std::cout << "----EventIndx :" << index << std::endl;

    for(int g=0; g< ConflictEventStationOpinions.EventStationOpinionsTable.size() ; g++){
        uint32_t gStationID = ConflictEventStationOpinions.EventStationOpinionsTable[g].StationID;
        std::cout << "----findStation :" << findStation(EventID,gStationID) << std::endl;
        if(findStation(EventID,gStationID)){
            ObjectOpinion* ptr = &getVehicle(EventID, gStationID).EgoTrust;
            *ptr = NTT->getNodeTrust(gStationID)->TrustOpinion;
            ObjectOpinion* ptr1 = &getVehicle(EventID, gStationID).ConflictOpinion;
            *ptr1 = OpinionConstruction.getStationOpinion(EventID, gStationID).StationOpinionValue;
        }else{
            Vehicle newVehicle;
            newVehicle.stationID = gStationID;     
            auto pint = NTT->getNodeTrust(gStationID);
            ObjectOpinion newTempOpinion = OpinionConstruction.getStationOpinion(EventID, gStationID).StationOpinionValue;
            if((newTempOpinion.GetAdvice()[0] == 1) || (newTempOpinion.GetAdvice()[1] == 1)){
                newVehicle.ConflictOpinion = newTempOpinion;
                ObjectOpinion newTempOpinion2 = pint->TrustOpinion;
                newVehicle.EgoTrust = newTempOpinion2;
                mConflictEventNodeTrustRevision[index].VehicleList.push_back(newVehicle);
            }else{
                return;
            }
        }
    }

    vector<ObjectOpinion> e;
    vector<ObjectOpinion> s;
    for(int i=0; i < mConflictEventNodeTrustRevision[index].VehicleList.size(); i++){
        e.push_back(mConflictEventNodeTrustRevision[index].VehicleList[i].EgoTrust);
        std::cout << "----eStationID :" << mConflictEventNodeTrustRevision[index].VehicleList[i].stationID << std::endl;
        s.push_back(mConflictEventNodeTrustRevision[index].VehicleList[i].ConflictOpinion);
        std::cout << "----sStationID :" << mConflictEventNodeTrustRevision[index].VehicleList[i].stationID << std::endl;
    }
    SubjectiveLogicFusion mySLF(e,s);
    ObjectOpinion temp;
    for(int i=0; i < mConflictEventNodeTrustRevision[index].VehicleList.size(); i++){
        mConflictEventNodeTrustRevision[index].VehicleList[i].RevisedTrust = mySLF.TrustRevisedOpinion()[i];
        uint32_t VehicleID = mConflictEventNodeTrustRevision[index].VehicleList[i].stationID;
        if(NTT->getNodeTrust(VehicleID)->RevisedOpinion.GetAdvice()[0] != 0){
            temp = NTT->getNodeTrust(VehicleID)->RevisedOpinion;
            NTT->setNodeTrust(VehicleID,8,temp);
        }
        temp = mySLF.TrustRevisedOpinion()[i];
        NTT->setNodeTrust(VehicleID,9,temp);
        std::cout << "----oStationID :" << mConflictEventNodeTrustRevision[index].VehicleList[i].stationID << std::endl;
    }
    e.clear();
    e.shrink_to_fit();
    s.clear();
    s.shrink_to_fit();
}

void ConflictEventTrustRevision::updateConflictEvent(NodeTrustTable* NTT,OpinionConstruction& OpinionConstruction,ConflictEventTable& CET){
    for(const auto& i : OpinionConstruction.getConflictEventOpinionsTable()){
        auto iter = find(mConflictEventNodeTrustRevision.begin(),mConflictEventNodeTrustRevision.end(),i.ConflictEventID);
        if(iter == mConflictEventNodeTrustRevision.end()){
            ConflictEventNodeTrustRevision newCENTR;
            newCENTR.ConflictEventID = i.ConflictEventID;
            mConflictEventNodeTrustRevision.push_back(newCENTR);
            // auto index = mConflictEventNodeTrustRevision.size()-1;
            // auto ConflictEventStationOpinions = OpinionConstruction.getConflictEventStationOpinions(i.ConflictEventID);
            // for(const auto& g : ConflictEventStationOpinions.EventStationOpinionsTable){
            //     Vehicle newVehicle;
            //     newVehicle.stationID = g.StationID;     
            //     ObjectOpinion newTempOpinion = OpinionConstruction.getStationOpinion(i.ConflictEventID, g.StationID).StationOpinionValue;
                // newVehicle.ConflictOpinion = newTempOpinion;
                // ObjectOpinion newTempOpinion2 = NTT.getTrustTable()[0].TrustOpinion;
                // ObjectOpinion newTempOpinion2 = NTT.getNodeTrust(g.StationID).TrustOpinion;
                // newVehicle.EgoTrust = newTempOpinion2;
                // newVehicle.RevisedTrust = ObjectOpinion();
                // std::cout << "----newVehicle :" << NTT.getNodeTrust(g.StationID).TrustOpinion.GetU() << std::endl;
            //     mConflictEventNodeTrustRevision[index].VehicleList.push_back(newVehicle);
            // }
        }

        cout << "mConflictEventStatus:::" << CET.getConflictEventInfoTable(i.ConflictEventID).mConflictEventStatus << endl;

        if(CET.getConflictEventInfoTable(i.ConflictEventID).mConflictEventStatus == Stop){
            return ;
        }else{ 
            updateStation(i.ConflictEventID, NTT, OpinionConstruction);
        }
    }
}


vector<ConflictEventNodeTrustRevision> ConflictEventTrustRevision::getConflictEventNodeTrustRevision(){
    return this->mConflictEventNodeTrustRevision;
}

ConflictEventNodeTrustRevision ConflictEventTrustRevision::getConflictEventNodeTrustRevision(uint32_t ConflictEventID){
    auto iter = find(mConflictEventNodeTrustRevision.begin(),mConflictEventNodeTrustRevision.end(),ConflictEventID);
    if(iter != mConflictEventNodeTrustRevision.end()){
        auto indx = distance(mConflictEventNodeTrustRevision.begin(),iter);
        return this->mConflictEventNodeTrustRevision[indx];
    }
}


uint32_t ConflictEventTrustRevision::EventIndx(uint32_t EventID){
    auto iter = find(mConflictEventNodeTrustRevision.begin(),mConflictEventNodeTrustRevision.end(),EventID);
    auto indx = distance(mConflictEventNodeTrustRevision.begin(),iter);
    if(iter != mConflictEventNodeTrustRevision.end()){
        return indx;
    }
    return -1;
}

bool ConflictEventTrustRevision::findStation(uint32_t EventID,uint32_t stationID){
    auto iter = find(mConflictEventNodeTrustRevision[EventIndx(EventID)].VehicleList.begin(),mConflictEventNodeTrustRevision[EventIndx(EventID)].VehicleList.end(),stationID);
    if(iter != mConflictEventNodeTrustRevision[EventIndx(EventID)].VehicleList.end()){
        return true;
    }
    return false;
}

Vehicle& ConflictEventTrustRevision::getVehicle(uint32_t EventID,uint32_t stationID){
    auto iter = find(mConflictEventNodeTrustRevision[EventIndx(EventID)].VehicleList.begin(),mConflictEventNodeTrustRevision[EventIndx(EventID)].VehicleList.end(),stationID);
    auto indx = distance(mConflictEventNodeTrustRevision[EventIndx(EventID)].VehicleList.begin(),iter);
    return mConflictEventNodeTrustRevision[EventIndx(EventID)].VehicleList[indx];   
}