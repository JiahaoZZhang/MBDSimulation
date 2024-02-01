/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    12/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "OpinionConstruction.h"

/**
 * @brief get the table of stations' opinions in a conflict event
 */
vector<ConflictEventStationOpinions> OpinionConstruction::getConflictEventOpinionsTable(){
    return this->ConflictEventOpinionsTable;
}


/**
 * @brief update the table of stations' opinions
 * 
 * @param CET Conflict Event Table 
 * @param TableCpm CPM table 
 * @param vdp Ego vehicle's data
 */
void OpinionConstruction::UpdateConflictEventOpinionTable(ConflictEventTable& CET, NodeTableCpm& TableCpm,const VehicleDataProvider* vdp){
    double CpmNumThreshold = 5;
    double w = 0;
    double BaseRate = 0.1;

    ConflictEventStationOpinions newConflictEventStationOpinions;
    StationOpinion newStationOpinion;
    this->ConflictEventOpinionsTable.clear();
    auto mConflictEventTable = CET.getConflictEventInfoTable();
    for(auto& eachEvent : mConflictEventTable){
         newConflictEventStationOpinions.ConflictEventID = eachEvent.ConflictEventID;
         uint32_t mConflictObjectID = eachEvent.EventInfo.getConflictObjectID();
         for(auto& eachStation: eachEvent.EventInfo.getConflictSourceAssociated()){
            double evidentNum =0;
            double PerceivedObjCount;
            double OpinionBelief;
            double OpinionDisbelief;
            ObjectOpinion newObjectOpinion;

            newStationOpinion.StationID = eachStation;
            int NodeID = eachStation;
            uint32_t egoID = vdp->getStationId()%100;
            if(NodeID == egoID){
                continue;
            }
            NodeHistoryCpm* StationHist = TableCpm.getNodeHistoryAddr(NodeID);
            uint32_t HistCpmNum = StationHist->getCPMNum();
            if(HistCpmNum > CpmNumThreshold){
                for(int i=0; i < CpmNumThreshold; i++){
                    vanetza::asn1::Cpm& CpmInfo = *StationHist->getCPMAddr(i);
                    PerceivedObjCount = CpmInfo->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
                    for(int g=0; g < PerceivedObjCount; g++){
                        PerceivedObject_t* list = CpmInfo->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[g];
                        if( list->objectID == mConflictObjectID){
                            evidentNum++;
                            break;
                        }
                    }
                }
                OpinionBelief = evidentNum / (CpmNumThreshold + w);
                OpinionDisbelief = (CpmNumThreshold + w - evidentNum) / (CpmNumThreshold + w);
                newObjectOpinion.SetAdvice({OpinionBelief,OpinionDisbelief});
                if(OpinionBelief > 0.4){
                    newObjectOpinion.SetAdvice({1,0});
                }
                newObjectOpinion.SetU(0);
                newObjectOpinion.SetBaseRate({BaseRate,1-BaseRate});
            }else{
                for(int i=0; i < HistCpmNum; i++){
                    vanetza::asn1::Cpm& CpmInfo = *StationHist->getCPMAddr(i);
                    PerceivedObjCount = CpmInfo->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
                    for(int g=0; g < PerceivedObjCount; g++){
                        PerceivedObject_t* list = CpmInfo->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[g];
                        if(mConflictObjectID == list->objectID){
                            evidentNum++;
                            break;
                        }
                    }
                }
                OpinionBelief = evidentNum / (HistCpmNum + w);
                OpinionDisbelief = (HistCpmNum + w - evidentNum) / (HistCpmNum + w);
                if((HistCpmNum + w) == 0){
                    OpinionBelief = 0;
                    OpinionDisbelief = 1;
                }
                newObjectOpinion.SetAdvice({OpinionBelief,OpinionDisbelief});
                if(OpinionBelief > 0.4){
                    newObjectOpinion.SetAdvice({1,0});
                }
                newObjectOpinion.SetU(0);
                newObjectOpinion.SetBaseRate({BaseRate,1-BaseRate});
            }

            newStationOpinion.StationOpinionValue = newObjectOpinion;
            newConflictEventStationOpinions.EventStationOpinionsTable.push_back(newStationOpinion);
        }

        this->ConflictEventOpinionsTable.push_back(newConflictEventStationOpinions);
    }

    this->ConflictEventOpinionsTable.shrink_to_fit();
}

void OpinionConstruction::UpdateConflictEventOpinionTable(ConflictEventTable& CET, NodeTableCpm& TableCpm,const Identity* mIdentity){
    double CpmNumThreshold = 5;
    double w = 0;
    double BaseRate = 0.1;

    ConflictEventStationOpinions newConflictEventStationOpinions;
    StationOpinion newStationOpinion;
    this->ConflictEventOpinionsTable.clear();
    auto mConflictEventTable = CET.getConflictEventInfoTable();
    for(auto& eachEvent : mConflictEventTable){
         newConflictEventStationOpinions.ConflictEventID = eachEvent.ConflictEventID;
         uint32_t mConflictObjectID = eachEvent.EventInfo.getConflictObjectID();
         for(auto& eachStation: eachEvent.EventInfo.getConflictSourceAssociated()){
            double evidentNum =0;
            double PerceivedObjCount;
            double OpinionBelief;
            double OpinionDisbelief;
            ObjectOpinion newObjectOpinion;

            newStationOpinion.StationID = eachStation;
            int NodeID = eachStation;
            uint32_t egoID = mIdentity->application%100;
            if(NodeID == egoID){
                continue;
            }
            NodeHistoryCpm* StationHist = TableCpm.getNodeHistoryAddr(NodeID);
            uint32_t HistCpmNum = StationHist->getCPMNum();
            if(HistCpmNum > CpmNumThreshold){
                for(int i=0; i < CpmNumThreshold; i++){
                    vanetza::asn1::Cpm& CpmInfo = *StationHist->getCPMAddr(i);
                    PerceivedObjCount = CpmInfo->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
                    for(int g=0; g < PerceivedObjCount; g++){
                        PerceivedObject_t* list = CpmInfo->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[g];
                        if( list->objectID == mConflictObjectID){
                            evidentNum++;
                            break;
                        }
                    }
                }
                OpinionBelief = evidentNum / (CpmNumThreshold + w);
                OpinionDisbelief = (CpmNumThreshold + w - evidentNum) / (CpmNumThreshold + w);
                newObjectOpinion.SetAdvice({OpinionBelief,OpinionDisbelief});
                if(OpinionBelief > 0.4){
                    newObjectOpinion.SetAdvice({1,0});
                }
                newObjectOpinion.SetU(0);
                newObjectOpinion.SetBaseRate({BaseRate,1-BaseRate});
            }else{
                for(int i=0; i < HistCpmNum; i++){
                    vanetza::asn1::Cpm& CpmInfo = *StationHist->getCPMAddr(i);
                    PerceivedObjCount = CpmInfo->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
                    for(int g=0; g < PerceivedObjCount; g++){
                        PerceivedObject_t* list = CpmInfo->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[g];
                        if(mConflictObjectID == list->objectID){
                            evidentNum++;
                            break;
                        }
                    }
                }
                OpinionBelief = evidentNum / (HistCpmNum + w);
                OpinionDisbelief = (HistCpmNum + w - evidentNum) / (HistCpmNum + w);
                if((HistCpmNum + w) == 0){
                    OpinionBelief = 0;
                    OpinionDisbelief = 1;
                }
                newObjectOpinion.SetAdvice({OpinionBelief,OpinionDisbelief});
                if(OpinionBelief > 0.4){
                    newObjectOpinion.SetAdvice({1,0});
                }
                newObjectOpinion.SetU(0);
                newObjectOpinion.SetBaseRate({BaseRate,1-BaseRate});
            }

            newStationOpinion.StationOpinionValue = newObjectOpinion;
            newConflictEventStationOpinions.EventStationOpinionsTable.push_back(newStationOpinion);
        }

        this->ConflictEventOpinionsTable.push_back(newConflictEventStationOpinions);
    }

    this->ConflictEventOpinionsTable.shrink_to_fit();
}


ConflictEventStationOpinions OpinionConstruction::getConflictEventStationOpinions(uint32_t ConflictEventID){
    auto iter = find(ConflictEventOpinionsTable.begin(),ConflictEventOpinionsTable.end(),ConflictEventID);
    auto indx = distance(ConflictEventOpinionsTable.begin(),iter);
    return this->ConflictEventOpinionsTable[indx];   
}


StationOpinion OpinionConstruction::getStationOpinion(uint32_t ConflictEventID, uint32_t StationID){
    auto iter = find(ConflictEventOpinionsTable.begin(),ConflictEventOpinionsTable.end(),ConflictEventID);
    auto indx = distance(ConflictEventOpinionsTable.begin(),iter);
    auto StaitionList = this->ConflictEventOpinionsTable[indx].EventStationOpinionsTable;
    auto iter1 = find(StaitionList.begin(),StaitionList.end(),StationID);
    auto indx2 = distance(StaitionList.begin(),iter1);
    return StaitionList[indx2];
}