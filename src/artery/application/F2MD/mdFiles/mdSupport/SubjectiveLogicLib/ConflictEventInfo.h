/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    12/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __ConflictEventInfo__
#define __ConflictEventInfo__

#include "../../mdEnumTypes/ConflictEventTypes.h"
#include <stdint.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>

using namespace std;
using namespace ConflictEventTypes;



class ConflictEventInfo
{
    private:

        uint32_t ConflictObjectID;
        ConflictEventTypes::ConflictType ConflictType;
        vector<uint32_t> ConflictSourceAssociated;
    
    public:
        ConflictEventInfo(){};
        ConflictEventInfo(uint32_t ConflictObjectID, ConflictEventTypes::ConflictType ConflictType, vector<uint32_t> ConflictSourceAssociated);
        
        // template <typename T, typename...Args>
        // void addConflictSourceAssociated(const T& t,const Args& ...ConflictStationID);

        void addConflictSourceAssociated(uint32_t ConflictStationID);

        // void addConflictSourceAssociated(vector<uint32_t> ConflictStationIDList);

        void delConflictSourceAssociated(uint32_t ConflictStationID);

        void setConflictSourceAssociated(vector<uint32_t> newConflictSourceAssociated);

        uint32_t getConflictObjectID();
        ConflictEventTypes::ConflictType getConflictType();
        vector<uint32_t> getConflictSourceAssociated();

        bool include(uint32_t stationID);


        bool operator == (const ConflictEventInfo& e){
            return (this->ConflictObjectID == e.ConflictObjectID) && (this->ConflictType == e.ConflictType) && (this->ConflictSourceAssociated == e.ConflictSourceAssociated);
        }

        ConflictEventInfo& operator = (const ConflictEventInfo& e){
            this->ConflictObjectID = e.ConflictObjectID;
            this->ConflictType = e.ConflictType;
            this->ConflictSourceAssociated = e.ConflictSourceAssociated;
            return* this;
        }

        ~ConflictEventInfo(){};


    protected:
};


#endif