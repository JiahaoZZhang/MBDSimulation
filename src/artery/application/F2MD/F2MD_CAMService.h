/*******************************************************************************
 * @author  Joseph Kamel, Maxime Georges
 * @email   josephekamel@gmail.com, maxime.georges059@gmail.com
 * @date    10/06/2021
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef F2MD_CAMSERVICE_H_
#define F2MD_CAMSERVICE_H_

#include "F2MD_msgs/BSM_m.h"
#include "F2MD_CAMFacility.h"
#include "F2MD_CAMParameters.h"

#include "traci/LiteAPI.h"
#include "artery/traci/VehicleController.h"
#include "artery/networking/Router.h"
#include "artery/application/ItsG5Service.h"
#include <inet/linklayer/ieee80211/mac/Ieee80211Mac.h>
#include <artery/inet/InetMobility.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/router.hpp>
#include <vanetza/geonet/interface.hpp>
#include <inet/common/ModuleAccess.h>
#include <cmath>
#include <libsumo/TraCIDefs.h>
#include <unordered_map>
#include <chrono>
using namespace std::chrono;

// forward declaration
namespace traci { class VehicleController; }

static F2MD_CAMParameters CAMparams;

using namespace omnetpp;
using namespace veins;

namespace artery {

class F2MD_CAMService : public artery::ItsG5Service
{

    public:
        void trigger() override;
        void finish();

    protected:
        void initialize() override;
        const traci::VehicleController* mVehicleController = nullptr;
        F2MD_CAMFacility mF2MDFacility;
        
        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
        void LocalMisbehaviorDetection(veins::BSM* bsm, int version);
        void TestReception(veins::BSM* BSMrecvd);
        void initParams();

        inet::MACAddress myId;
        cModule* node;

        simtime_t beaconInterval;
        simtime_t lastBeacon;
};
}
#endif /* F2MD_CAMSERVICE_H_ */
