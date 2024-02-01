/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    12/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "ConflictEventTable.h"


/**
 * @brief add a Event element to the conflict table
 */
ConflictEventInfoStruct ConflictEventTable::addElementToConflictEventInfoTable(ConflictEventInfo newConflictEventInfo){
    ConflictEventInfoStruct newConflictEvent;
    newConflictEvent.ConflictEventID = this->ConflictEventIDGenerator;
    newConflictEvent.EventInfo = newConflictEventInfo;
    double temptime = simTime().dbl();
    newConflictEvent.mConflictEventStatus = InProgress;
    newConflictEvent.Timestamp = temptime;
    cTable.push_back(newConflictEvent);
    this->ConflictEventIDGenerator++;
    return newConflictEvent;
}

/**
 * @brief delete a Event element of the conflict table
 */
void ConflictEventTable::delElementToConflictEventInfoTable(ConflictEventInfo ConflictEvent){
    for(const auto& event: this->cTable){
        if(ConflictEvent == event.EventInfo){
            auto iterIndex = find(cTable.begin(),cTable.end(),event);
            cTable.erase(iterIndex);
        }
    }
}

void ConflictEventTable::delElementToConflictEventInfoTable(uint32_t ConflictEventID){
    for(const auto& event: this->cTable){
        if(ConflictEventID == event.ConflictEventID){
            auto iterIndex = find(cTable.begin(),cTable.end(),event);
            cTable.erase(iterIndex);
        }
    }
}

/**
 * @brief auto delete the NoConflict Type
 */
void ConflictEventTable::delElementToConflictEventInfoTable(){
    for(auto& event: this->cTable){
        if(event.EventInfo.getConflictType() == ConflictEventTypes::ConflictType::NoConflict){
            auto iterIndex = find(cTable.begin(),cTable.end(),event);
            cTable.erase(iterIndex);
        }
    }
}

/**
 * @brief get the conflict event information table
 */
vector<ConflictEventInfoStruct>  ConflictEventTable::getConflictEventInfoTable(){
    return this->cTable;
}

/**
 * @brief get the conflict event information by event id
 */
ConflictEventInfoStruct ConflictEventTable::getConflictEventInfoTable(uint32_t EventID){
    auto iter = find(cTable.begin(),cTable.end(),EventID);
    if(iter != cTable.end()){
        auto ListIndex = distance(cTable.begin(),iter);
        return this->cTable[ListIndex];
    }
}

/**
 * @brief get the lastest conflict event information
 */
ConflictEventInfoStruct  ConflictEventTable::getLastConflictEventInfo(){
    return this->cTable[this->ConflictEventIDGenerator-1];
}



/**
 * @brief Set the Conflict Event Status
 */
 void ConflictEventTable::setConflictEventStatus(uint32_t EventID,ConflictEventTypes::ConflictEventStatus newStatus){
    for(int i=0; i < cTable.size(); i++){
        if(cTable[i].ConflictEventID == EventID){
            cTable[i].mConflictEventStatus = newStatus;
            break;
        }
    }
 }


 /**
  * @brief 
  * 
  */
 tuple<bool,int> ConflictEventTable::IncludeEvent(int ConflictObjectID){
    for(auto& i:this->cTable){
        if(i.EventInfo.getConflictObjectID() == ConflictObjectID){
            return make_tuple(true,i.ConflictEventID);
        }
    }
    return make_tuple(false,NULL);
}