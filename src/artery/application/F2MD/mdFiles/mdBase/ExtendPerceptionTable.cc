/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    25/10/2023
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2023 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "ExtendPerceptionTable.h"

/**
 * @brief update the extended perception table
 * when receiving or transmitting a cpm 
 * 
 * @param cpm 
 * @param currentTime millisecond
 */
void ExtendPerceptionTable::updateTable(const vanetza::asn1::Cpm* cpm, int64_t currentTime){

    auto observerID = (*cpm)->header.stationId;
    
    const auto LTS = (*cpm)->payload.managementContainer.referenceTime;
    long LastUpdateTime;
    auto res = asn_INTEGER2long(&LTS, &LastUpdateTime);

    auto refPosition = (*cpm)->payload.managementContainer.referencePosition;

    auto containerSize = (*cpm)->payload.cpmContainers.list.count;
    for(int i=0; i< containerSize; i++){
        if ((*cpm)->payload.cpmContainers.list.array[i]->containerData.present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer){
            auto POC = (*cpm)->payload.cpmContainers.list.array[i]->containerData.choice.PerceivedObjectContainer;
            auto perceivedObjSize = POC.numberOfPerceivedObjects;
            for(int j=0; j < perceivedObjSize; j++){
                auto perceivedObject = POC.perceivedObjects.list.array[j];
                auto res = this->find(*perceivedObject->objectId);

                ExtendPerceivedObject newEPO;
                PerceivedObject_t* newperceivedObject = (PerceivedObject_t*)vanetza::asn1::copy(asn_DEF_PerceivedObject,perceivedObject);
                newEPO.LastUpdateTime = LastUpdateTime;
                newEPO.ObjectInfo = *newperceivedObject;
                newEPO.LastRecordTime = currentTime;
                newEPO.ObserverID = observerID;
                newEPO.PerceivedObjectID = *perceivedObject->objectId;
                newEPO.ReferencePosition = refPosition;

                if(res.second == false){
                    this->ExtendPerceptionList.insert(newEPO);
                }else{
                    this->ExtendPerceptionList.erase(res.first);
                    this->ExtendPerceptionList.insert(newEPO);
                }
            }
        }  
    }

    // delete all old objects
    this->updateTable(currentTime);
}


/**
 * @brief delete all old perceived objects (no updating over 5s)
 * 
 * @param currentTime 
 */
void ExtendPerceptionTable::updateTable(int64_t currentTime){
    for(auto it=this->ExtendPerceptionList.begin(); it!= this->ExtendPerceptionList.end();){
        if((currentTime - (*it).LastRecordTime) > MAX_MEMORY_EPO){
            it = this->ExtendPerceptionList.erase(it);
        }else{
            ++it;
        }
    }
    this->LastTimeStamp = currentTime;
}


std::pair<ExtendPerceivedObject, bool> ExtendPerceptionTable::getObjInfo(int objID){
    auto res = this->find(objID);
    if(res.second == false){
        ExtendPerceivedObject* p =nullptr;
        return std::make_pair(*p,false);
    }else{
        auto EPO = *res.first;
        return std::make_pair(EPO,true);
    }
}

std::pair<std::set<ExtendPerceivedObject>::iterator,bool> ExtendPerceptionTable::find(int objID){
   std::set<ExtendPerceivedObject>::iterator it;
   it = std::find(this->ExtendPerceptionList.begin(),this->ExtendPerceptionList.end(), objID);
   if(it == this->ExtendPerceptionList.end()){
        return std::make_pair(it,false);
   }else{
        return std::make_pair(it,true);
   }
}

