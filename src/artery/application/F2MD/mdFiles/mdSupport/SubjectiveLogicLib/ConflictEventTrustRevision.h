/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    02/06/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef ___ConflictEventTrustRevision__
#define ___ConflictEventTrustRevision__

#include "OpinionConstruction.h"
#include "ObjectOpinion.h"
#include "../../mdBase/NodeTrustTable.h"
#include "SubjectiveLogic.h"


struct Vehicle{
    uint32_t stationID;
    ObjectOpinion EgoTrust;
    ObjectOpinion ConflictOpinion;
    ObjectOpinion RevisedTrust;

    bool operator==(const uint32_t e){
        return (this->stationID == e);
    }
};


struct ConflictEventNodeTrustRevision
{
    uint32_t ConflictEventID;
    vector<Vehicle> VehicleList;

    bool operator==(const uint32_t e){
        return (this->ConflictEventID == e);
    }
};



class ConflictEventTrustRevision
{
private:
    vector<ConflictEventNodeTrustRevision> mConflictEventNodeTrustRevision;
    // NodeTrustTable* mNodeTrustTable;
    // OpinionConstruction* mOpinionConstruction;
    void updateStation(uint32_t EventID,NodeTrustTable* NTT,OpinionConstruction& OpinionConstruction);
    void updateConflictEvent(NodeTrustTable* NTT,OpinionConstruction& OpinionConstruction, ConflictEventTable& CET);
    bool findStation(uint32_t EventID,uint32_t stationID);
    Vehicle& getVehicle(uint32_t EventID,uint32_t stationID);
    // void calTR(uint32_t EventID);
    uint32_t EventIndx(uint32_t EventID);


public:
    ConflictEventTrustRevision(){};
    void update(NodeTrustTable* NTT,OpinionConstruction& OpinionConstruction, ConflictEventTable& CET);

    vector<ConflictEventNodeTrustRevision> getConflictEventNodeTrustRevision();
    ConflictEventNodeTrustRevision getConflictEventNodeTrustRevision(uint32_t ConflictEventID);

    ~ConflictEventTrustRevision(){};
};


#endif