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

#include "artery/application/CpmObject.h"
#include "CPMService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include "artery/envmod/EnvironmentModelObject.h"

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>

namespace artery
{

using namespace omnetpp;

static const auto microdegree = vanetza::units::degree * boost::units::si::micro;
static const auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;
static const auto decidegree = vanetza::units::degree * boost::units::si::deci;
static const auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
    boost::units::quantity<U> v { q };
    return std::round(v.value());
}

/**
   *  @brief builds a Speed Value that suits the ETSI CPM protocol (rounded centimeter value)
*/
SpeedValue_t buildSpeedValuePerceivedObj(const vanetza::units::Velocity& v)
{
	static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
	static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

	SpeedValue_t speed = SpeedValue_unavailable;
	if (v >= upper) {
		speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
	} else if (v >= lower) {
		speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	}
	return speed;
}

Define_Module(CPMService)

CPMService::CPMService(){
}

/**
   *  @brief Initialise CPM Service
*/
void CPMService::initialize()
{
    std::cout << "entering initialize of CPMService at "<<SIMTIME_STR(simTime())<<std::endl;

    ItsG5BaseService::initialize();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();

    mTimer = &getFacilities().get_const<Timer>();
    mlocalEnvmod = &getFacilities().get_const<LocalEnvironmentModel>();
    mGenCpmMax = par("maxInterval");
    mfixedRate = par("fixedRate");
    mLastCpmTimestamp = simTime();

}


/**
   *  @brief  called periodically by the middleware. UpdateFreq is defined in omnetpp.ini
*/
void CPMService::trigger()
{  
	const SimTime& T_GenCpmMax = mGenCpmMax;
    const SimTime T_elapsed = simTime() - mLastCpmTimestamp;

    if (T_elapsed >= T_GenCpmMax)
    {
        std::cout << "packet transmission at "<< SIMTIME_STR(simTime())<<std::endl;
        sendCpm(simTime());
    }   
}


/**
   *  @brief  called by the middleware each time a CPM is received by this node.
*/
void CPMService::indicate(const vanetza::btp::DataIndication& indication, std::unique_ptr<vanetza::UpPacket> packet)
{
    Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
    const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);

    const auto egoStationID = getFacilities().get_const<VehicleDataProvider>().station_id();
    std::cout << egoStationID <<" packet reception at "<< SIMTIME_STR(simTime())<<std::endl;

    if (cpm && cpm->validate()) {

        CpmObject obj = visitor.shared_wrapper;
        //emit(scSignalCpmReceived, &obj);

        const vanetza::asn1::Cpm& msg = obj.asn1();

        // Display important fields contained in the received BSM, to check the reception (Useful for debugging)
        checkReception(msg);
       // perceptionFusion(msg);
        //mGlobalPerceptionTable->updatePerception(obj);
    }
} 

/**
   *  @brief  print in QtEnv what is contained in the cpm received. Useful for debugging. To display fields in the console, use std::cout instead of EV_INFO
   *  @param vanetza::asn1::Cpm& cpm received
*/
void CPMService::checkReception(const vanetza::asn1::Cpm& msg){

    const auto egoStationID = getFacilities().get_const<VehicleDataProvider>().station_id()%100;

    EV_INFO << "station " <<egoStationID <<" received packet at "<< SIMTIME_STR(simTime()) << endl;
    EV_INFO << "sender id : " << msg->header.stationID << std::endl;
    EV_INFO << "station type : " << msg->cpm.cpmParameters.managementContainer.stationType <<std::endl;
    EV_INFO << "numberOfPerceivedObjects : " << msg->cpm.cpmParameters.numberOfPerceivedObjects << std::endl;

    if (msg->cpm.cpmParameters.numberOfPerceivedObjects >0)
    { 
        const PerceivedObjectContainer* poc = msg->cpm.cpmParameters.perceivedObjectContainer;

        for (int i = 0; i < poc->list.count; i++) { 

            PerceivedObject_t *po = poc->list.array[i];
            EV_INFO << "------> Object ID : "<< po->objectID << endl;
            EV_INFO << "time of measurement : "<< po->timeOfMeasurement<< endl;
            EV_INFO << "xDistance.value : " << po->xDistance.value << endl;
            EV_INFO << "yDistance.value : " << po->yDistance.value << endl;
            EV_INFO << "xSpeed.value : " << po->xSpeed.value << endl;
            EV_INFO << "ySpeed.value : " << po->ySpeed.value << endl;
            EV_INFO << "yawAngle.value : " << po->yawAngle->value << endl;
        }

    }
}

const Timer* CPMService::getTimer() const
{
    return mTimer;
}

/**
   *  @brief  create, fill, and send a cpm with correct informations
   *  @param simTime_t current time
*/
void CPMService::sendCpm(const SimTime& T_now)
{

    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));

    // Create a new CPM
    auto cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, genDeltaTimeMod,*mlocalEnvmod);
    mLastCpmTimestamp = T_now;

    using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CPM;
	request.gn.its_aid = aid::CP;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	CpmObject obj(std::move(cpm));

    // emit(scSignalCpmSent, &obj);
	using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CpmByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);

    // Send the CPM to lower layers
	this->request(request, std::move(payload));
}

/**
   *  @brief  create and fill a cpm with correct informations
   *  @param VehicleDataProvider& my vehicle data provider
   *  @param uint16_t genDeltaTime
   *  @param LocalEnvironmentModel& my local env model
*/
vanetza::asn1::Cpm CPMService::createCollectivePerceptionMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime, const LocalEnvironmentModel& lem)
{
    using vanetza::facilities::distance;

    vanetza::asn1::Cpm message;
    ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_cpm;

	header.stationID = vdp.station_id();

    EV_INFO << "-------------- New CPM created by "<< vdp.station_id()<<  " ------------------- "  << endl;

    CollectivePerceptionMessage_t& cpm = (*message).cpm;
	cpm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;

    // Management container
    CpmManagementContainer_t& management = cpm.cpmParameters.managementContainer;
    management.stationType = StationType_passengerCar;
	management.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	management.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	management.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	management.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	management.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	management.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	management.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

    // Data container 
    cpm.cpmParameters.stationDataContainer = vanetza::asn1::allocate<StationDataContainer_t>();
    cpm.cpmParameters.stationDataContainer->present = StationDataContainer_PR_originatingVehicleContainer;

    OriginatingVehicleContainer& data = cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;

    data.speed.speedValue = buildSpeedValuePerceivedObj(vdp.speed());
    data.speed.speedConfidence = SpeedConfidence_unavailable;
    data.heading.headingValue = round(vdp.heading(), vanetza::units::degree) * CartesianAngleValue_oneDegree; // degree from north, clockwise
    data.heading.headingConfidence = HeadingConfidence_unavailable;

    data.yawRate = vanetza::asn1::allocate<YawRate_t>();

    data.yawRate->yawRateValue= round(vdp.yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
    
	if (data.yawRate->yawRateValue < -32766 || data.yawRate->yawRateValue > 32766) {
		data.yawRate->yawRateValue = YawRateValue_unavailable;
	}

    data.longitudinalAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration_t>();
    const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		data.longitudinalAcceleration->longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		data.longitudinalAcceleration->longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}


    // Now the dynamic pereceived list

    
    const auto& allobjects = lem.allObjects();

    const TrackedObjectsFilterRange& objects = filterBySensorCategory(allobjects, "Radar");

    for (const auto& obj : objects)
    {
        std::weak_ptr<EnvironmentModelObject> obj_ptr = obj.first;
        if (obj_ptr.expired()) continue; /*< objects remain in tracking briefly after leaving simulation */
        const auto& vd = obj_ptr.lock()->getVehicleData();
        
        std::cout << "station ID: " << vd.station_id()
         //   << " lon: " << vd.longitude()
           // << " lat: " << vd.latitude()
         //   << " speed: " << vd.speed()
            << " when: " << vd.updated()
            << std::endl;
    }

    /****begin change****/
    uint32_t perceivedobj_id;



    // Perceived objects container
    //cpm.cpmParameters.numberOfPerceivedObjects = objects.size() ;

    //*******Need to test if size >0 or not****/
    cpm.cpmParameters.perceivedObjectContainer = vanetza::asn1::allocate<PerceivedObjectContainer_t>();

    for (const auto& obj : objects) { 
        std::weak_ptr<EnvironmentModelObject> obj_ptr = obj.first;  
        if (obj_ptr.expired()) continue; /*< objects remain in tracking briefly after leaving simulation */
 
        auto object = vanetza::asn1::allocate<PerceivedObject_t>();
        const VehicleDataProvider& vd = obj_ptr.lock()->getVehicleData();   

        object->objectID = vd.getStationId ()% 100;

        int16_t timeofmeasurement = countTaiMilliseconds(mTimer->getTimeFor(vd.updated()));
        double deltaTimeOfMeasurement = genDeltaTime - timeofmeasurement;

        if (abs(deltaTimeOfMeasurement) >= 1500){ /*< CPM ASN1 requirements */
            object->timeOfMeasurement = 1500;
        }
        else{
            object->timeOfMeasurement = deltaTimeOfMeasurement;
        }

        object->xDistance.value = round(abs(vd.position().x-vdp.position().x), boost::units::si::meter) * DistanceValue_oneMeter;
        object->xDistance.confidence = DistanceConfidence_unavailable;

        object->yDistance.value = round(abs(vd.position().y-vdp.position().y), boost::units::si::meter) * DistanceValue_oneMeter;
        object->yDistance.confidence = DistanceConfidence_unavailable;
    
        object->xSpeed.value = buildSpeedValuePerceivedObj(fabs(vd.speed() * cos((PI/2)-vd.heading().value())));
        object->xSpeed.confidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;

        object->ySpeed.value = buildSpeedValuePerceivedObj(fabs(vd.speed() * sin((PI/2)-vd.heading().value())));
        object->ySpeed.confidence = SpeedConfidence_unavailable;
        
        object->yawAngle = vanetza::asn1::allocate<CartesianAngle_t>();

        object->yawAngle->value = round(vd.heading(), vanetza::units::degree) * CartesianAngleValue_oneDegree; // degree from north, clockwise
        object->yawAngle->confidence = HeadingConfidence_unavailable;

        ASN_SEQUENCE_ADD(cpm.cpmParameters.perceivedObjectContainer, object);
       
    }
    
    cpm.cpmParameters.numberOfPerceivedObjects = cpm.cpmParameters.perceivedObjectContainer->list.count;
        std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid CPM message %s", error.c_str());
	}

    std::cout << vdp.station_id()<<" packet transmission at "<< SIMTIME_STR(simTime())<<std::endl;
    return message;
}

} // namespace artery
