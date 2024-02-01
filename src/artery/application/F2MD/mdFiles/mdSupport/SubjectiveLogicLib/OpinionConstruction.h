/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    12/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __OPINIONCONSTRUCTION__
#define __OPINIONCONSTRUCTION__


#include "ConflictEventTable.h"
#include "ObjectOpinion.h"
#include "../../mdBase/NodeTableCpm.h"
#include <vanetza/asn1/cpm.hpp>
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/Identity.h"
#include <iostream>

using namespace std;
using namespace artery;

struct StationOpinion
{
    uint32_t StationID;
    ObjectOpinion StationOpinionValue;
    bool operator==(const uint32_t e){
        return (this->StationID == e);
    }
};


struct ConflictEventStationOpinions
{
    uint32_t ConflictEventID;
    vector<StationOpinion> EventStationOpinionsTable;

    bool operator==(const uint32_t e){
        return (this->ConflictEventID == e);
    }
};



class OpinionConstruction
{
    private:
        vector<ConflictEventStationOpinions> ConflictEventOpinionsTable;

    public:
        OpinionConstruction(){this->ConflictEventOpinionsTable.clear();};

        void UpdateConflictEventOpinionTable(ConflictEventTable& CET, NodeTableCpm& TableCpm,const VehicleDataProvider* vdp);   
        void UpdateConflictEventOpinionTable(ConflictEventTable& CET, NodeTableCpm& TableCpm,const Identity* mIdentity);

        vector<ConflictEventStationOpinions> getConflictEventOpinionsTable();
        ConflictEventStationOpinions getConflictEventStationOpinions(uint32_t ConflictEventID);
        StationOpinion getStationOpinion(uint32_t ConflictEventID, uint32_t StationID);
        ~OpinionConstruction(){};
};



#endif