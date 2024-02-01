/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    12/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __ConflictEventTable__
#define __ConflictEventTable__

#include "../../mdEnumTypes/ConflictEventTypes.h"
#include "ConflictEventInfo.h"
#include <stdint.h>
#include <vector>
#include <omnetpp.h>



using namespace std;
using namespace omnetpp;

struct ConflictEventInfoStruct{
    uint32_t ConflictEventID;
    ConflictEventInfo EventInfo;
    double Timestamp;
    ConflictEventTypes::ConflictEventStatus mConflictEventStatus;
    bool operator == (const ConflictEventInfoStruct& e){
        return (this->ConflictEventID == e.ConflictEventID) && (this->EventInfo == e.EventInfo) && (this->Timestamp == e.Timestamp);
    }

    bool operator == (const uint32_t ConflictEventID){
        return (this->ConflictEventID == ConflictEventID);
    }

    ConflictEventInfoStruct operator= (const ConflictEventInfoStruct& e){
        this->ConflictEventID = e.ConflictEventID;
        this->EventInfo = e.EventInfo;
        this->Timestamp = e.Timestamp;
        return *this;
    }
};


class ConflictEventTable
{

    private:
        uint32_t ConflictEventIDGenerator=0;
        
    public:
        vector<ConflictEventInfoStruct> cTable;

        ConflictEventTable(){};

        ConflictEventInfoStruct addElementToConflictEventInfoTable(ConflictEventInfo newConflictEvent); // return the generated Conflict Event Structure

        void delElementToConflictEventInfoTable(ConflictEventInfo ConflictEvent); 
        void delElementToConflictEventInfoTable(uint32_t ConflictEventID); 
        void delElementToConflictEventInfoTable();   // Auto delete all the Noconflict Event Type

        void setConflictEventStatus(uint32_t EventID, ConflictEventTypes::ConflictEventStatus newStatus);

        vector<ConflictEventInfoStruct>  getConflictEventInfoTable();
        ConflictEventInfoStruct  getConflictEventInfoTable(uint32_t EventID);
        ConflictEventInfoStruct  getLastConflictEventInfo();
        tuple<bool,int> IncludeEvent(int ConflictObjectID);
        // tuple<bool,int> IncludeN

        ~ConflictEventTable(){};
            
};


#endif