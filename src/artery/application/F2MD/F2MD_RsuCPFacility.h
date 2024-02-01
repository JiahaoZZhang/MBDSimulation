/*******************************************************************************
 * @author  Jiahao ZHANG
 * @email   jiahao.zhang96@gmail.com
 * @date    24/08/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef F2MD_RsuCPFACILITY_H_
#define F2MD_RsuCPFACILITY_H_

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


#include "traci/API.h"
#include "artery/traci/VehicleController.h"
#include <inet/linklayer/ieee80211/mac/Ieee80211Mac.h>
#include <artery/inet/InetMobility.h>

#include <inet/common/ModuleAccess.h>
#include <cmath>
#include <libsumo/TraCIDefs.h>
#include <unordered_map>
#include <chrono>

using namespace omnetpp;
using namespace vanetza;
using namespace traci;
using namespace std::chrono;

// forward declaration
namespace traci { class VehicleController; class API;}
namespace artery { class VehicleDataProvider;}

// Global params
static unsigned int accusedNodesCP[MAX_ACCUSED_LENGTH] = {};
static int accusedNodesLengthCP = 0;

class F2MD_RsuCPFacility
{
    
    public:

        F2MD_RsuCPFacility(){};
        F2MD_RsuCPFacility(std::shared_ptr<const traci::API>* TraciLiteAPI, F2MD_CPParameters& CPparams);
        void initialize();
        double onCPM(const vanetza::asn1::Cpm& msg, CpmCheckList& mCpmCheckList);

        NodeTableCpm& getNodeTableCpm();

    private:
        bool CheckConflict;
        F2MD_CPParameters* params;
        std::shared_ptr<const traci::API>* mLiteAPI;
        NodeTableCpm detectedNodes; 
        CpmChecks mCpmChecks;

        const traci::VehicleController* mVehicleController;

        double MAX_PLAUSIBLE_ACCEL;
        double MAX_PLAUSIBLE_DECEL;
        double MAX_PLAUSIBLE_SPEED;



};







#endif