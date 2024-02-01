/*******************************************************************************
 * @author  Jiahao ZHANG
 * @email   jiahao.zhang96@gmail.com
 * @date    09/11/2022
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef F2MD_CPFACILITY_H_
#define F2MD_CPFACILITY_H_

#include "F2MD_CPParameters.h"

#include "artery/application/CpmObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/LocalEnvironmentModel.h"



#include "mdFiles/mdAttacks/MDCpmAttack.h"
#include "mdFiles/mdEnumTypes/MbTypes.h"
#include "mdFiles/mdEnumTypes/CpAttackTypes.h"
#include "mdFiles/mdSupport/GeneralLib.h"
#include "mdFiles/mdChecks/CpmChecks.h"
#include "mdFiles/mdBase/NodeTableCpm.h"
#include "mdFiles/mdBase/NodeHistoryCpm.h"
#include "mdFiles/mdBase/CpmCheckList.h"
#include "mdFiles/mdReport/MDReport.h"
#include "mdFiles/mdSupport/JsonWriter.h"


#include "traci/LiteAPI.h"
#include "artery/traci/VehicleController.h"
#include <inet/linklayer/ieee80211/mac/Ieee80211Mac.h>
#include <artery/inet/InetMobility.h>

#include <vanetza/btp/data_interface.hpp>

#include <vanetza/geonet/pdu.hpp>
#include <vanetza/geonet/pdu_conversion.hpp>
#include <vanetza/geonet/pdu_variant.hpp>
#include <vanetza/net/osi_layer.hpp>

#include <inet/common/ModuleAccess.h>
#include <cmath>
#include <libsumo/TraCIDefs.h>
#include <unordered_map>
#include <chrono>

using namespace std::chrono;

// forward declaration
namespace traci { class VehicleController; }
namespace artery { class VehicleDataProvider;}

// Global params
static int LastCPLocalAttackIndex = -1;
static unsigned int accusedNodesCP[MAX_ACCUSED_LENGTH] = {};
static int accusedNodesLengthCP = 0;

// namespace srs = boost::geometry::srs;
namespace bg = boost::geometry;

using namespace omnetpp;
using namespace artery;
//using namespace veins;




class F2MD_CPFacility
{

    public:
        F2MD_CPFacility(){};
        F2MD_CPFacility(const artery::VehicleDataProvider* VehProvider, const traci::VehicleController* VehController, F2MD_CPParameters& params, unsigned long* PseudonymID);
        void initialize();
        void finish();
        void induceAttack(CpmPayload_t* cpm);
        double onCPM(const vanetza::asn1::Cpm& msg, const::vanetza::MacAddress& neighborMAcId, CpmCheckList& mCpmCheckList);
        double getAttackType(){return myAttackType;};
        mbTypes::Mbs getMbType(){return myMdType;};

        vector<AttackedObjInfo> getAttackedObjInfo(){return mAttackedObjInfoList;};


    private:
        GeneralLib genLib = GeneralLib();
        bool CheckConflict;

        F2MD_CPParameters* params;

        const artery::VehicleDataProvider* mVehicleDataProvider;
        const traci::VehicleController* mVehicleController;

        void treatAttackFlags();

        mbTypes::Mbs induceMisbehavior(double localAttacker);
        void addAccusedNode(unsigned int id);
        bool isAccusedNode(unsigned int id);
        unsigned long* myPseudonym;
        std::string printFinalTagtoJson();

        std::string writeV2XPDU(const vanetza::asn1::Cpm& msg,const::vanetza::MacAddress& neighborMAcId);
        std::string writeReport(const vanetza::asn1::Cpm& msg, const::vanetza::MacAddress& neighborMAcId, CpmCheckList& mCpmCheckList, vanetza::asn1::Cpm& lastCpm);

        MDCpmAttack mdAttack;
        mbTypes::Mbs myMdType;
        cpAttackTypes::Attacks myAttackType;
        NodeTableCpm detectedNodes; 

        JsonWriter jwritecpmReport;
        ofstream outputjsonfilecpmReport;

        CpmChecks mCpmChecks;

        vector<AttackedObjInfo> mAttackedObjInfoList;
};

#endif /* F2MD_CPFACILITY_H_ */
