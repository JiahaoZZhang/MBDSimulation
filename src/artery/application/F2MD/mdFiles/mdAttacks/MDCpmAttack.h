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

/*
# This file contains all the methods to describe and launch attacks based on CPM
*/

#ifndef __VEINS_MDCPMATTACK_H_
#define __VEINS_MDCPMATTACK_H_

#include <tuple>
#include <omnetpp.h>

#include "../mdSupport/GeneralLib.h"
#include "../../F2MD_CPParameters.h"
#include "artery/application/CpmObject.h"
#include <vanetza/asn1/its/CpmPayload.h>
#include "traci/LiteAPI.h"
#include "artery/traci/VehicleController.h"

using namespace omnetpp;
using namespace traci;

class MDCpmAttack {
    private:
        GeneralLib genLib = GeneralLib();
        simtime_t* beaconInterval;

        double MaxRadarRange;
        double MaxMapBoundary;

        double ConstDistX;
        double ConstDistY;

        double ConstSpeedX;
        double ConstSpeedY;
        double ConstSpeed;

        double ConstDistOffsetX;
        double ConstDistOffsetY;

        double ConstSpeedOffsetX;
        double ConstSpeedOffsetY;
        double ConstSpeedOffset;

        double maxTimeOfMeasurement;

        F2MD_CPParameters *params;

        uint32_t SelectedStationID = 0;
        void InitSelectedStationID(PerceivedObjectContainer *poc);
        const traci::VehicleController* mVehicleController;

    public:

        MDCpmAttack();

        void init(F2MD_CPParameters *params, const traci::VehicleController* VehController);

        void launchAttack(cpAttackTypes::Attacks myAttackType, CpmPayload_t* cpm);

        void setBeaconInterval(simtime_t* beaconInterval);
};

#endif
