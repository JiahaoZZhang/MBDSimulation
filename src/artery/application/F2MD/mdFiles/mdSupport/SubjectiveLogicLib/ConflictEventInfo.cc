/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    12/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "ConflictEventInfo.h"


ConflictEventInfo::ConflictEventInfo(uint32_t ConflictObjectID, ConflictEventTypes::ConflictType ConflictType, vector<uint32_t> ConflictSourceAssociated){
    this->ConflictObjectID = ConflictObjectID;
    this->ConflictType = ConflictType;
    this->ConflictSourceAssociated = ConflictSourceAssociated;
}

/**
 * @brief add new conflict sources to the list
 * @param ConflictStationID 
 */
// template <typename T,typename...Args>
// void ConflictEventInfo::addConflictSouceAssociated(const T& t,const Args& ...ConflictStationID){
//         if constexpr (sizeof...(ConflictStationID)>0){
//             this->ConflictSouceAssociated.push_back(t);
//             sort(ConflictSouceAssociated.begin(),ConflictSouceAssociated.end());
//             ConflictSouceAssociated.erase(unique(ConflictSouceAssociated.begin(),ConflictSouceAssociated.end()), ConflictSouceAssociated.end());
//             addConflictSouceAssociated(ConflictStationID...);
//         }else{
//             this->ConflictSouceAssociated.push_back(t);
//             this->ConflictSouceAssociated.shrink_to_fit();
//         }
// }


void ConflictEventInfo::addConflictSourceAssociated(uint32_t ConflictStationID){
    // if(std::find(this->ConflictSouceAssociated.begin(),this->ConflictSouceAssociated.end(),ConflictStationID) == this->ConflictSouceAssociated.end()){
    this->ConflictSourceAssociated.push_back(ConflictStationID);
    // }
    sort(ConflictSourceAssociated.begin(),ConflictSourceAssociated.end());
    ConflictSourceAssociated.erase(unique(ConflictSourceAssociated.begin(),ConflictSourceAssociated.end()), ConflictSourceAssociated.end());
}


/**
 * @brief add new conflict sources to the list
 * 
 * @param ConflictStationIDList 
 */
// void ConflictEventInfo::addConflictSouceAssociated(vector<uint32_t> ConflictStationIDList){
//     for(auto const& i: ConflictStationIDList){
//         auto iter = std::find(this->ConflictSouceAssociated.begin(),this->ConflictSouceAssociated.end(),i);
//         if( iter != this->ConflictSouceAssociated.end()){
//             continue;
//         }else{
//             this->ConflictSouceAssociated.push_back(i);
//             sort(ConflictSouceAssociated.begin(),ConflictSouceAssociated.end());
//             ConflictSouceAssociated.erase(unique(ConflictSouceAssociated.begin(),ConflictSouceAssociated.end()), ConflictSouceAssociated.end());
//         }
//     }
// }


/**
 * @brief delete conflict sources in the list
 * 
 * @param ConflictStationID 
 */
void ConflictEventInfo::delConflictSourceAssociated(uint32_t ConflictStationID){
    auto iter = std::find(this->ConflictSourceAssociated.begin(),this->ConflictSourceAssociated.end(),ConflictStationID);
    if( iter != this->ConflictSourceAssociated.end()){
        ConflictSourceAssociated.erase(iter);
    }
}


uint32_t ConflictEventInfo::getConflictObjectID(){
    return this->ConflictObjectID;
}

ConflictEventTypes::ConflictType ConflictEventInfo::getConflictType(){
    return this->ConflictType;
}

vector<uint32_t> ConflictEventInfo::getConflictSourceAssociated(){
    return this->ConflictSourceAssociated;
}


void ConflictEventInfo::setConflictSourceAssociated(vector<uint32_t> newConflictSourceAssociated){
    this->ConflictSourceAssociated = newConflictSourceAssociated;
}

bool ConflictEventInfo::include(uint32_t stationID){
    for(const auto& i: this->ConflictSourceAssociated){
        if(i == stationID){
            return true;
        }
    }
    return false;
}