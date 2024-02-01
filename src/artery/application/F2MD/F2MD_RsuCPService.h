/*******************************************************************************
 * @author  Jiahao ZHANG
 * @email   jiahao.zhang96@gmail.com
 * @date    24/08/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/


#ifndef ARTERY_F2MD_RSUCPSERVICE_H_
#define ARTERY_F2MD_RSUCPSERVICE_H_

#include "F2MD_CPParameters.h"
#include "F2MD_RsuCPFacility.h"

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/service/EnvmodPrinter.h"
#include "artery/networking/StationaryPositionProvider.h"
#include "artery/traci/Cast.h"
#include "traci/API.h"
#include "traci/Core.h"
#include "traci/Position.h"
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include <iostream>
#include <fstream>
#include "mdFiles/mdSupport/JsonWriter.h"

namespace traci { class VehicleController; }

static F2MD_CPParameters CPparams;

namespace artery
{

class GeoPosition;
class Identity;
class Timer;
class VehicleDataProvider;
class StationaryPositionProvider;

class F2MD_RsuCPService : public ItsG5BaseService
{
    public:
        F2MD_RsuCPService();
        void initialize() override;
        void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        void trigger() override;

    protected:
        std::string printRXCpmtoJson(const vanetza::asn1::Cpm& msg);
        std::string printTXCpmtoJson(const vanetza::asn1::Cpm& msg) ;
        std::string printperceivedlisttoJson(const std::string& title, const TrackedObjectsFilterRange& objs);
        
    private:
        F2MD_RsuCPFacility mF2MDRsuCPFacility;

        StationaryPositionProvider* mPositionProvider = nullptr;
        traci::Core* mTraciCore = nullptr;
        std::shared_ptr<const traci::API> mTraciAPI; 
        const Identity* mIdentity = nullptr;
        const GeoPosition* mGeoPosition = nullptr;
		const Timer* mTimer = nullptr;
        omnetpp::SimTime mLastCpmTimestamp;
        omnetpp::SimTime mGenerationInterval;

        const LocalEnvironmentModel* mlocalEnvmod;
        const GlobalEnvironmentModel* mglobleEnvmod;
        EnvmodPrinter* mEnvmodprint;
        ofstream outputfilelog;
        ofstream outputjsonfilecpm;
        ofstream outputjsonfilelem;
        ofstream outputjsonfilecpmEmit;
        
        JsonWriter jwritelem;
        JsonWriter jwritecpmRX;
        JsonWriter jwritecpmTX;
        
        CpmCheckList mCpmCheckList;

        using ItsG5BaseService::getFacilities;
        const Timer* getTimer() const;
		void sendCpm(const omnetpp::SimTime&);
        vanetza::asn1::Cpm createCollectivePerceptionMessage(uint16_t genDeltaTime, const LocalEnvironmentModel&);


        void initParams();
        void checkReception(const vanetza::asn1::Cpm&);
        void finish();

        std::string printFinalTagtoJson();
};


} // namespace artery

#endif
