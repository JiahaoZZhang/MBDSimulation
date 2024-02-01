/*******************************************************************************
 * @author  Maxime Georges
 * @email   maxime.georges059@gmail.com
 * @date    10/06/2021
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef ARTERY_CPMSERVICE_H_
#define ARTERY_CPMSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/GlobalPerceptionTable.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace traci { class VehicleController; }

namespace artery
{

using namespace omnetpp;

class Timer;
class VehicleDataProvider;

class CPMService : public ItsG5BaseService
{
    public:
        CPMService();
        void initialize() override;
        void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        void trigger() override;

        using ItsG5BaseService::getFacilities;
        const Timer* getTimer() const;
		void sendCpm(const omnetpp::SimTime&);
        vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider&, uint16_t genDeltaTime, const LocalEnvironmentModel&);

    protected:
        void checkReception(const vanetza::asn1::Cpm&);
        //void perceptionFusion(const vanetza::asn1::Cpm&);

    private:
        const VehicleDataProvider* mVehicleDataProvider = nullptr;
		const Timer* mTimer = nullptr;
        omnetpp::SimTime mLastCpmTimestamp;
		omnetpp::SimTime mGenCpmMax;
        bool mfixedRate;
		omnetpp::SimTime mGenCpm;
        const LocalEnvironmentModel* mlocalEnvmod;      
        GlobalPerceptionTable* mGlobalPerceptionTable = nullptr;


};
} // namespace artery

#endif
