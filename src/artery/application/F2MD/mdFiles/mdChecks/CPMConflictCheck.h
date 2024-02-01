/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    30/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __CPMConflictCheck__
#define __CPMConflictCheck__

#include "artery/application/F2MD/F2MD_CPParameters.h"
#include "artery/application/F2MD/mdFiles/mdSupport/SubjectiveLogicLib/ConflictEventTable.h"
#include "artery/application/F2MD/mdFiles/mdChecks/CpmChecks.h"
#include "artery/application/F2MD/mdFiles/mdBase/NodeTableCpm.h"
#include "artery/application/F2MD/mdFiles/mdBase/NodeHistoryCpm.h"
#include "artery/application/F2MD/mdFiles/mdBase/NodeTypeClassification.h"
#include "artery/application/F2MD/mdFiles/mdBase/NodeTrustTable.h"
#include "artery/application/F2MD/mdFiles/mdSupport/SubjectiveLogicLib/ConflictEventTrustRevision.h"
#include "artery/application/F2MD/mdFiles/mdSupport/SubjectiveLogicLib/TrustManagementFunc/TrustManagementFunc.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/Identity.h"
#include "traci/API.h"
#include <vanetza/asn1/cpm.hpp>
#include <xtensor/xarray.hpp>
#include "artery/envmod/Geometry.h"
#include <vector>
#include <tuple>

using namespace omnetpp;
using namespace vanetza;
using namespace artery;
using namespace std;
namespace bg = boost::geometry;


class CPMConflictCheck
{
    private:
        ConflictEventTable mConflictEventTable;
        TrustManagementFunc myTrustManagementFunc;
        tuple<bool,int> IncludeEvent(int ConflictObjectID);
        int InPerceptionFlag=0;
        int InPerceptionFlag2=0;
        int recoverFlag=0;
        bool conflictInclude = false;
        double AssessmentFunction(double n, double a, double b, double y);
        double PenaltyFunction1(double n, double a, double b, double min , double max);
        bool IsPointInPolygon(artery::geometry::Point pt, artery::geometry::Polygon polygon);

        bool CheckConflict=false;


    public:
        CPMConflictCheck(){};
        // void updateConflictCheck(NodeTableCpm& TableCpm);
        
        void updateConflictCheckO(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,const VehicleDataProvider* vdp, 
        NodeTrustTable& NTT, NodeTableCpm& TableCpm, NodeTypeClassification& NTC);
        
        void updateConflictCheckI(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,const VehicleDataProvider* vdp, 
            NodeTrustTable& NTT, NodeTableCpm& TableCpm, NodeTypeClassification& NTC, const traci::VehicleController* VehController);

        void updateConflictCheckRsuIC(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,const VehicleDataProvider* vdp, NodeTrustTable& NTT, 
        NodeTableCpm& TableCpm, NodeTypeClassification& NTC, const traci::VehicleController* VehController);
        
        void updateConflictCheckRsuIR(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem, NodeTrustTable& NTT, 
        NodeTableCpm& TableCpm, NodeTypeClassification& NTC,std::shared_ptr<traci::API>* TraciLiteAPI, const Identity* mIdentity,const GeoPosition* mGeoPosition);

        void updateConflictCheckRsuOC(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,const VehicleDataProvider* vdp, NodeTrustTable& NTT, 
        NodeTableCpm& TableCpm, NodeTypeClassification& NTC, const traci::VehicleController* VehController);

        void updateConflictCheckRsuOR(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem, NodeTrustTable& NTT, 
        NodeTableCpm& TableCpm, NodeTypeClassification& NTC,std::shared_ptr<traci::API>* TraciLiteAPI, const Identity* mIdentity,const GeoPosition* mGeoPosition);


        void TrustRevisionL1(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem, const VehicleDataProvider* mVehicleDataProvider,
            NodeTrustTable& myNodeTrustTable,NodeTableCpm& detectedNodes, CPMConflictCheck& myConflictCheck,
            NodeTypeClassification& myNodeTypeClassification,F2MD_CPParameters* params);

        void TrustRevisionL2(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem, const traci::VehicleController* VehController,
            const VehicleDataProvider* mVehicleDataProvider,NodeTrustTable& myNodeTrustTable, NodeTableCpm& detectedNodes, CPMConflictCheck& myConflictCheck
            ,ConflictEventTable& myConflictEventTable, CpmChecks mCpmChecks, GlobalEnvironmentModel& mGlobalEnvironmentModel);

        void TrustRevisionL3(ConflictEventTable& myConflictEventTable, NodeTableCpm& detectedNodes,const VehicleDataProvider* mVehicleDataProvider,
            NodeTrustTable& myNodeTrustTable,OpinionConstruction& myOpinionConstruction,ConflictEventTrustRevision myConflictEventTrustRevision);


        ConflictEventTable& getConflictEventTable();

        bool InLocalPerception(uint32_t newNode, const LocalEnvironmentModel& lem);
        ~CPMConflictCheck(){};

    protected:

        
};

#endif