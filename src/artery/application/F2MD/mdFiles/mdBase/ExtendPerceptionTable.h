/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    25/10/2023
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2023 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __ExtendPerceptionTable__
#define __ExtendPerceptionTable__

#define MAX_MEMORY_EPO 5000

#include <vector>
#include <algorithm>
#include <iterator>
#include <set>
#include <vanetza/asn1/cpm.hpp>
#include <omnetpp/simtime.h>

using namespace std;
using namespace omnetpp;



struct ExtendPerceivedObject
{
    /* basic information */
    long LastUpdateTime;
    int PerceivedObjectID; // This extended perception ID should be allocated by ego
    int LastRecordTime;

    /* Location of observer */
    uint32_t ObserverID;
    ReferencePosition_t ReferencePosition;

    
    /* kinematic data of perceived object */
    PerceivedObject_t ObjectInfo;
    CartesianCoordinateLarge_t xCoordinateValue;
    CartesianCoordinateLarge_t yCoordinateValue;
    SpeedValue_t velocityMagnitudeValue;


    bool operator == (const ExtendPerceivedObject &e) const{
        return ((this->LastUpdateTime == e.LastUpdateTime) && (this->PerceivedObjectID == e.PerceivedObjectID));
    }

    bool operator == (const int &e) const{
        return (this->PerceivedObjectID == e);
    }

    bool operator<(const int &e) const{
        return (this->PerceivedObjectID < e);
    }

    bool operator<(const ExtendPerceivedObject &e) const{
        return (this->PerceivedObjectID < e.PerceivedObjectID);
    }

    bool operator>(const ExtendPerceivedObject &e) const{
        return (this->PerceivedObjectID > e.PerceivedObjectID);
    }

    bool operator()(const ExtendPerceivedObject &left, const int &right){
        return (left.PerceivedObjectID < right);
    }

    ExtendPerceivedObject& operator =(const ExtendPerceivedObject &e){
        this->LastUpdateTime = e.LastUpdateTime;
        this->ObjectInfo = e.ObjectInfo;
        this->ObserverID = e.ObserverID;
        this->PerceivedObjectID = e.PerceivedObjectID;
        this->ReferencePosition = e.ReferencePosition;
        
        return *this;
    }
};



class ExtendPerceptionTable
{
private:
    /* data */
    std::set<ExtendPerceivedObject> ExtendPerceptionList;
    std::pair<std::set<ExtendPerceivedObject>::iterator,bool> find(int objID);
    int64_t LastTimeStamp;

public:
    ExtendPerceptionTable(){
        this->ExtendPerceptionList.clear();
        this->LastTimeStamp = 0;
    };

    void updateTable(const vanetza::asn1::Cpm* cpm, int64_t currentTime);
    void updateTable(int64_t currentTime);

    std::pair<ExtendPerceivedObject, bool> getObjInfo(int objID);
    int64_t getLastTimeStamp(){return this->LastTimeStamp;};
    std::set<ExtendPerceivedObject>getExtendPerceptionList(){return this->ExtendPerceptionList;};

    ~ExtendPerceptionTable(){};
};




#endif

