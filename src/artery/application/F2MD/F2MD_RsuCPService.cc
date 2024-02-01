/*******************************************************************************
 * @author  Jiahao ZHANG
 * @email   jiahao.zhang96@gmail.com
 * @date    24/08/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/


#include "artery/application/CpmObject.h"
#include "F2MD_RsuCPService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/sensor/Sensor.h"

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>

namespace artery
{

using namespace omnetpp;


Define_Module(F2MD_RsuCPService)

F2MD_RsuCPService::F2MD_RsuCPService(){
}


/**
   *  @brief Initialise CPM Service and CPM Facility for further F2MD Processing
*/
void F2MD_RsuCPService::initialize()
{
    // std::cout << "entering initialize in RsuCPService at "<<SIMTIME_STR(simTime())<<std::endl;

    ItsG5BaseService::initialize();

    mIdentity = &getFacilities().get_const<Identity>();
    mGeoPosition = &getFacilities().get_const<GeoPosition>();
    mTimer = &getFacilities().get_const<Timer>();
    mlocalEnvmod = &getFacilities().get_const<LocalEnvironmentModel>();
    mPositionProvider = &getFacilities().get_mutable<StationaryPositionProvider>();
    mTraciCore = &getFacilities().get_mutable<traci::Core>();
    mTraciAPI = mTraciCore->getAPI();  
    initParams();


    // ------ Initialization of F2MD Module ------ Start
    mF2MDRsuCPFacility = F2MD_RsuCPFacility(&mTraciAPI,CPparams);
    mF2MDRsuCPFacility.initialize();
    // ------ Initialization of F2MD Module ------ End

    // generation rate boundaries
    int myId = mIdentity->application;


    // Test Log file
    string str_id = to_string(myId);
    string outfilename = "output/RSU/Log/log_RSU_";
    outfilename.append(str_id);
    outputfilelog.open(outfilename);

    // RX json Log
    string outjsonfilecpmRX_name = "output/RSU/CPM/RX/logjson_RSU_cpm_";
    outjsonfilecpmRX_name.append(str_id);
    outputjsonfilecpm.open(outjsonfilecpmRX_name);
    jwritecpmRX.openJsonElementList("CPM_RX");

    // TX json Log
    string outjsonfilecpmTX_name = "output/RSU/CPM/TX/logjson_RSU_cpm_";
    outjsonfilecpmTX_name.append(str_id);
    outputjsonfilecpmEmit.open(outjsonfilecpmTX_name);
    jwritecpmTX.openJsonElementList("CPM_TX");

    // LP json log
    string outlemjsonfile_name = "output/RSU/LP/logjson_RSU_lp_";
    outlemjsonfile_name.append(str_id);
    outputjsonfilelem.open(outlemjsonfile_name);
    jwritelem.openJsonElementList("LP");
    
}


/**
   *  @brief  called periodically by the middleware. UpdateFreq is defined in omnetpp.ini
*/
void F2MD_RsuCPService::trigger()
{  
    const SimTime T_elapsed = simTime() - mLastCpmTimestamp;

    // The generation condition needs to be completed
    if (T_elapsed >= CPparams.GenerationInterval)
    {
        std::cout << " <------- packet transmission at "<< SIMTIME_STR(simTime())<<std::endl;

        if (outputfilelog.is_open()){ outputfilelog << " <------- packet transmission at "<< SIMTIME_STR(simTime())<< " \n" << std::endl; }

        sendCpm(simTime());
        std::cout << " <------- RSU sent an CPM"<<std::endl;
    }   
}


/**
   *  @brief  called by the middleware each time a CPM is received by this node.
*/
void F2MD_RsuCPService::indicate(const vanetza::btp::DataIndication& indication, std::unique_ptr<vanetza::UpPacket> packet)
{

    Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
    const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);

    const auto egoStationID = mIdentity->application;
    std::cout << egoStationID <<"packet reception at "<< SIMTIME_STR(simTime())<<std::endl;
    if (outputfilelog.is_open()){ outputfilelog << " -------> packet reception at "<< SIMTIME_STR(simTime())<<std::endl; }
    auto checkCPMvalidate = cpm->validate();
    std::cout << "checkCPMvalidate :" << checkCPMvalidate << std::endl;
    

    if (cpm && cpm->validate()) {

        CpmObject obj = visitor.shared_wrapper;
        const vanetza::asn1::Cpm& msg = obj.asn1();

        // Display important fields contained in the received BSM, to check the reception (Useful for debugging)
        checkReception(msg);

        // Pass the received CPM to the F2MD Processing facility for further analysis (MbD checks,...)
        mCpmCheckList.resetAll();
        double checkfailed = mF2MDRsuCPFacility.onCPM(msg, mCpmCheckList);

    }
} 

/**
   *  @brief  print in QtEnv what is contained in the cpm received. Useful for debugging.
   *  @param vanetza::asn1::Cpm& cpm received
*/
void F2MD_RsuCPService::checkReception(const vanetza::asn1::Cpm& msg){
    auto& allObjects = mlocalEnvmod->allObjects();
    jwritecpmRX.addTagToElement("CPM_RX", printRXCpmtoJson(msg));
    jwritelem.addTagToElement("LP",printperceivedlisttoJson("Radar Sensor Object List", filterBySensorCategory(allObjects, "Radar")));
}


const Timer* F2MD_RsuCPService::getTimer() const
{
    return mTimer;
}

/**
   *  @brief  create, fill, and send a cpm with correct informations
   *  @param simTime_t current time
*/
void F2MD_RsuCPService::sendCpm(const SimTime& T_now)
{

    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getCurrentTime());

    // Create a new CPM
    auto cpm = createCollectivePerceptionMessage(genDeltaTimeMod,*mlocalEnvmod);
    mLastCpmTimestamp = T_now;

    // print each emitted CPM messages.
    jwritecpmTX.addTagToElement("CPM_TX", printTXCpmtoJson(cpm));    
    
    using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CPM;
	request.gn.its_aid = aid::CP;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	CpmObject obj(std::move(cpm));
	using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CpmByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);

    // Send the CPM to lower layers
	this->request(request, std::move(payload));
}

/**
   *  @brief  create and fill a cpm with correct informations
   *  @param uint16_t genDeltaTime
   *  @param LocalEnvironmentModel& my local env model
*/
vanetza::asn1::Cpm F2MD_RsuCPService::createCollectivePerceptionMessage(uint16_t genDeltaTime, const LocalEnvironmentModel& lem)
{
    using vanetza::facilities::distance;

    vanetza::asn1::Cpm message;
    ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageId = MessageId_cpm;

    // Note:no pseudonym change
	header.stationId = mIdentity->application;
    
    CpmPayload_t& CpmPayload = (*message).payload;
    // Create Management Container
    ManagementContainer_t& management = CpmPayload.managementContainer;


    long referenceTime = countTaiMilliseconds(mTimer->getCurrentTime());
    int ret = 0;
    ret += asn_long2INTEGER(&management.referenceTime, referenceTime);
    assert(ret == 0);

	management.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	management.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    const double longitude = mGeoPosition->longitude / vanetza::units::degree;
    long mlongitude = std::round(longitude * 1e7);
    if(mlongitude > (-18e8) && mlongitude < 18e8+1){
	    management.referencePosition.longitude = mlongitude;
    }else if(mlongitude == -18e8){
        management.referencePosition.longitude = Longitude_valueNotUsed;
    }else{
        management.referencePosition.longitude = Longitude_unavailable;
    }

    const double latitude = mGeoPosition->latitude / vanetza::units::degree;
	long mlatitude = std::round(latitude * 1e7);
    if(mlatitude > (-9e8) && mlatitude < 9e8+1){
        management.referencePosition.latitude = mlatitude;
    }else{
        management.referencePosition.latitude = Latitude_unavailable;
    }

    management.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
    management.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    management.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    

    // TraciAPI convert the Static Node (ITS station - RSU) GeoPosition to Carteisian Position
    auto mRsuPosX = mTraciAPI->convert2D({longitude,latitude}).x * boost::units::si::meter;  
    auto mRsuPosY = mTraciAPI->convert2D({longitude,latitude}).y * boost::units::si::meter; 

    
    // double RSUorientationAngle = PI/2;

    ConstraintWrappedCpmContainers_t* containersData = vanetza::asn1::allocate<ConstraintWrappedCpmContainers_t>();

    // -----------------------------------
    // Create OriginatingRSU Container 
    // -----------------------------------
    WrappedCpmContainer_t* WrappedOriginatingRSUContainer = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    WrappedOriginatingRSUContainer->containerId = CpmContainerId_originatingRsuContainer;
    WrappedOriginatingRSUContainer->containerData.present = WrappedCpmContainer__containerData_PR_OriginatingRsuContainer;
    OriginatingRsuContainer_t&  mOriginatingRSUContainer = WrappedOriginatingRSUContainer->containerData.choice.OriginatingRsuContainer;
    // orientationAngle != heading according to EN 302 890-2, but in the simulateur, we consider orientationAngle = heading
    // Wgs84AngleValue, Unit: 0,1 degrees
    // RSU orientationAngle is defined in .ini
    mOriginatingRSUContainer.mapReference->present = MapReference_PR_NOTHING;
    ASN_SEQUENCE_ADD(containersData, WrappedOriginatingRSUContainer);  


    // --------------------------------
    // Create PerceivedObject Container
    // --------------------------------
    // the dynamic pereceived list
    const auto& objects = lem.allObjects();
    WrappedCpmContainer_t* WrappedPerceivedObjectContainer = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    WrappedPerceivedObjectContainer->containerId = CpmContainerId_perceivedObjectContainer;
    WrappedPerceivedObjectContainer->containerData.present = WrappedCpmContainer__containerData_PR_PerceivedObjectContainer;
    PerceivedObjectContainer_t& mPerceivedObjectContainer = WrappedPerceivedObjectContainer->containerData.choice.PerceivedObjectContainer;
    
    int countLostTracingNum=0;
    PerceivedObjects_t* POC_DF = &mPerceivedObjectContainer.perceivedObjects;


    if(objects.size()>0){
        // mPerceivedObjectContainer.perceivedObjects = vanetza::asn1::allocate<PerceivedObjects_t>();
        for (const auto& obj : objects) { 
            std::weak_ptr<EnvironmentModelObject> obj_ptr = obj.first;  
            if (obj_ptr.expired()){
                countLostTracingNum++;
                continue; /*< objects remain in tracking briefly after leaving simulation */
            }
            PerceivedObject_t* object = vanetza::asn1::allocate<PerceivedObject_t>();
            const VehicleDataProvider& vd = obj_ptr.lock()->getVehicleData();   
            
            // Get OMNET++ Vehicle ID
            object->objectId = vanetza::asn1::allocate<Identifier2B_t>();
            long int objID = vd.getStationId();
            *(object->objectId) = objID;
            long timeofmeasurement = countTaiMilliseconds(mTimer->getTimeFor(vd.updated()));
            long deltaTimeOfMeasurement = referenceTime - timeofmeasurement;
        
            
            /*************************************************************************************
            //  DeltaTimeMilliSecondSigned represents a signed difference in time with respect to a reference time
            //  Unit: 0,001 s
            *************************************************************************************/
            if (deltaTimeOfMeasurement > 2046){ /* CPM ASN1 requirements */
                object->measurementDeltaTime = 2047;
            }else if(deltaTimeOfMeasurement < -2048){
                object->measurementDeltaTime = -2048;
            }else{
                object->measurementDeltaTime = deltaTimeOfMeasurement;
            }
            
            /*************************************************************************************
            // need to correct the vd.position and vdp.postion
            // because this position in OMNET++ is not the cartesian position in sumo
            // In OMNET++: y axis is growing to bottom; x axis is growing to right
            // In SUMO: y axis is growing to top; x axis is growing to right
            // the X coordinate value with the associated confidence level
            *************************************************************************************/
            long xCoordinate = std::round((vd.position().x.value()- mRsuPosX.value()) * 1e2); // CartesianCoordinate, Unit:0,01 m
            if (xCoordinate > 131070){
                object->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
            }else if(xCoordinate < -131072){
                object->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
            }else{
                object->position.xCoordinate.value = xCoordinate;
            }
            object->position.xCoordinate.confidence = CoordinateConfidence_unavailable;


            /*************************************************************************************
            // the Y coordinate value with the associated confidence level
            // Y axis is OMNET coordinate system
            *************************************************************************************/
            long yCoordinate = std::round((vd.position().y.value() - mRsuPosY.value()) * 1e2);
            if (yCoordinate > 131070){
                object->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
            }else if(yCoordinate < -131072){
                object->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
            }else{
                object->position.yCoordinate.value = yCoordinate;
            }
            object->position.yCoordinate.confidence = CoordinateConfidence_unavailable;
        
            // the velocity vector of the object within the pre-defined coordinate system. (ENU coordinate system)
            object->velocity = vanetza::asn1::allocate<Velocity3dWithConfidence_t>();
            object->velocity->present = Velocity3dWithConfidence_PR_polarVelocity;
            uint32_t PreceivedObjSpeed = std::round(vd.speed().value() * 1e2); // SpeedValue, Unit:0,01 m/s
            if (PreceivedObjSpeed > 16381){
                object->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
            }else if(PreceivedObjSpeed > 0 && PreceivedObjSpeed < 16382){
                object->velocity->choice.polarVelocity.velocityMagnitude.speedValue = PreceivedObjSpeed;
            }else if(PreceivedObjSpeed == 0){
                object->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
            }else{
                object->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
            }
            object->velocity->choice.polarVelocity.velocityMagnitude.speedConfidence = SpeedConfidence_unavailable;


            /***********************************************************************************************************
            // CartesianAngleValue represents an angle value described in a local Cartesian coordinate system
            // per default counted positive in a right-hand local coordinate system from the abscissa. degree from right (x axis), anti-clockwise
            // need to change the reference of heading()
            // Ines : set it to heading now, we need to fix it with Jiahao
            //object->velocity->choice.polarVelocity.velocityDirection.value = CartesianAngleValue_unavailable;
            ************************************************************************************************************/
            object->velocity->choice.polarVelocity.velocityDirection.value =  std::round(vd.heading().value() * 1800 / PI); // heading(), radian from north, clockwise

            object->velocity->choice.polarVelocity.velocityDirection.confidence = AngleConfidence_unavailable;

            // the acceleration vector of the object within the pre-defined coordinate system. (ENU coordinate system)
            object->acceleration = vanetza::asn1::allocate<Acceleration3dWithConfidence_t>();
            object->acceleration->present = Acceleration3dWithConfidence_PR_polarAcceleration;
            uint32_t PreceivedObjAcceleration = std::round(vd.acceleration().value() * 10); // AccelerationMagnitudeValue, Unit:0,1 m/s
            if (PreceivedObjAcceleration > 159){
                object->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue = AccelerationMagnitudeValue_positiveOutOfRange;
            }else if(PreceivedObjAcceleration > 0 && PreceivedObjAcceleration < 160){
                object->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue = PreceivedObjAcceleration;
            }else{
                object->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue = AccelerationMagnitudeValue_unavailable;
            }
            object->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationConfidence = AccelerationConfidence_unavailable;


            /***********************************************************************************************************
            // CartesianAngleValue represents an angle value described in a local Cartesian coordinate system
            // per default counted positive in a right-hand local coordinate system from the abscissa. degree from right (x axis), anti-clockwise
            // need to change the reference of heading()     
            ***********************************************************************************************************/
            object->acceleration->choice.polarAcceleration.accelerationDirection.value = CartesianAngleValue_unavailable;
            object->acceleration->choice.polarAcceleration.accelerationDirection.confidence = AngleConfidence_unavailable;

            // Euler angles of the object bounding box at the time of measurement.
            object->angles = vanetza::asn1::allocate<EulerAnglesWithConfidence_t>();
            int temp = std::round(900-vd.heading().value()*1800/PI);
            object->angles->zAngle.value = (temp % 3600 + 3600) % 3600;
            object->angles->zAngle.confidence = AngleConfidence_unavailable;
            

            object->zAngularVelocity= vanetza::asn1::allocate<CartesianAngularVelocityComponent_t>();
            int tempzAV = std::round(vd.yaw_rate().value()*180/PI);
            // cout << "zAngularVelocity::" <<  vd.yaw_rate().value() << ";;;" << tempzAV << endl;
            if(tempzAV > 255){
                object->zAngularVelocity->value = 255;
            }else if(tempzAV < -255){
                object->zAngularVelocity->value = -255;
            }else{
                object->zAngularVelocity->value = tempzAV;
            }
            object->zAngularVelocity->confidence = AngularSpeedConfidence_unavailable;
            ASN_SEQUENCE_ADD(POC_DF, object);
        }
        mPerceivedObjectContainer.numberOfPerceivedObjects = objects.size() - countLostTracingNum;
    }else{
        mPerceivedObjectContainer.numberOfPerceivedObjects = objects.size();
    }
    ASN_SEQUENCE_ADD(containersData, WrappedPerceivedObjectContainer);  


    // --------------------------------------
    // Create the SensorInformation Container
    // sensor range, Unit 0.1m
    // sensor open angle, Unit 0.1 degree 
    // --------------------------------------
    WrappedCpmContainer_t* WrappedSensorInformationContainer = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    WrappedSensorInformationContainer->containerId = CpmContainerId_sensorInformationContainer;
    WrappedSensorInformationContainer->containerData.present = WrappedCpmContainer__containerData_PR_SensorInformationContainer;
    SensorInformationContainer_t&  mSensorInformationContainer = WrappedSensorInformationContainer->containerData.choice.SensorInformationContainer;
    const auto sensors = lem.getSensors();
    // std::cout << "sensors size::::" << sensors.size() << std::endl;
    uint sensorID = 0;
    for(const auto sensor:sensors){
        SensorInformation_t* SensorInformation = vanetza::asn1::allocate<SensorInformation_t>();
        SensorInformation->sensorId = sensorID + 1;
        SensorInformation->sensorType = SensorType_radar;
        SensorInformation->shadowingApplies = false;
        SensorInformation->perceptionRegionShape = vanetza::asn1::allocate<Shape_t>();
        SensorInformation->perceptionRegionShape->present = Shape_PR_radial;
        SensorInformation->perceptionRegionShape->choice.radial.range = sensor->getFieldOfView().range.value() * 10;

        // Wgs84AngleValue, Unit: 0,1 degrees
        // getFieldOfView().angle.value(), Unit: 1 degree
        int temp1 = std::round(CPparams.RSUorientationAngle*10 - sensor->getFieldOfView().angle.value()*10/2); 
        int temp2 = std::round(CPparams.RSUorientationAngle*10 + sensor->getFieldOfView().angle.value()*10/2);      
        int OpeningAngleStart =  (temp1 % 3600 + 3600) % 3600;
        int OpeningAngleEnd = (temp2 % 3600 + 3600) % 3600;
        SensorInformation->perceptionRegionShape->choice.radial.stationaryHorizontalOpeningAngleStart = OpeningAngleStart;
        SensorInformation->perceptionRegionShape->choice.radial.stationaryHorizontalOpeningAngleEnd = OpeningAngleEnd;
        
        ASN_SEQUENCE_ADD(&mSensorInformationContainer, SensorInformation);
        sensorID++;
    }
    ASN_SEQUENCE_ADD(containersData, WrappedSensorInformationContainer);  


    // --------------------------------------
    // Create the PerceptionRegion Container
    // --------------------------------------
    WrappedCpmContainer_t* WrappedPerceptionRegionContainer = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    WrappedPerceptionRegionContainer->containerId = CpmContainerId_perceptionRegionContainer;
    WrappedPerceptionRegionContainer->containerData.present = WrappedCpmContainer__containerData_PR_PerceptionRegionContainer;
    PerceptionRegionContainer_t* mPerceptionRegionContainer = &WrappedPerceptionRegionContainer->containerData.choice.PerceptionRegionContainer;
    for(const auto sensor:sensors){
        PerceptionRegion_t* mPerceptionRegion = vanetza::asn1::allocate<PerceptionRegion_t>();

        long timeofmeasurement = countTaiMilliseconds(mTimer->getCurrentTime());
        long deltaTimeOfMeasurement = referenceTime - timeofmeasurement;
        mPerceptionRegion->measurementDeltaTime = deltaTimeOfMeasurement;
        mPerceptionRegion->shadowingApplies = false;
        mPerceptionRegion->perceptionRegionShape.present = Shape_PR_radial;
        mPerceptionRegion->perceptionRegionShape.choice.radial.range = sensor->getFieldOfView().range.value();
        int temp1 = std::round(CPparams.RSUorientationAngle*10 - sensor->getFieldOfView().angle.value()*10/2); 
        int temp2 = std::round(CPparams.RSUorientationAngle*10 + sensor->getFieldOfView().angle.value()*10/2);
        int OpeningAngleStart =  (temp1 % 3600 + 3600) % 3600;
        int OpeningAngleEnd = (temp2 % 3600 + 3600) % 3600;
        mPerceptionRegion->perceptionRegionShape.choice.radial.stationaryHorizontalOpeningAngleStart = OpeningAngleStart;
        mPerceptionRegion->perceptionRegionShape.choice.radial.stationaryHorizontalOpeningAngleEnd = OpeningAngleEnd;
        mPerceptionRegion->perceptionRegionConfidence = ConfidenceLevel_unavailable;
        mPerceptionRegion->shadowingApplies = false;

        ASN_SEQUENCE_ADD(mPerceptionRegionContainer, mPerceptionRegion);
    }
    ASN_SEQUENCE_ADD(containersData, WrappedPerceptionRegionContainer);


    CpmPayload.cpmContainers = *containersData;

    std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid CPM message %s", error.c_str());
	}

    std::cout << mIdentity->application <<" packet transmission at "<< SIMTIME_STR(simTime())<<std::endl;
    return message;
}



/**
   *  @brief  stores the parameters defined in omnetpp.ini in the F2MD_CPMParameters.h file so that they can be used in the simulation.
*/
void F2MD_RsuCPService::initParams(){

    /* Base Setting */
    CPparams.MixCpLocalAttacks = par("MixCpLocalAttacks");
    CPparams.RandomCpLocalMix = par("RandomCpLocalMix");

    CPparams.CP_LOCAL_ATTACKER_PROB = par("CP_LOCAL_ATTACKER_PROB");
    CPparams.CP_LOCAL_ATTACK_TYPE = cpAttackTypes::intAttacks[par("CP_LOCAL_ATTACK_TYPE").intValue()];

    CPparams.MaxRadarAngle = par("MaxRadarAngle");
    CPparams.MaxRadarRange = par("MaxRadarRange");

    CPparams.START_CPM_ATTACK = par("START_CPM_ATTACK");
    CPparams.MAX_CPM_DELTATIME = par("MAX_CPM_DELTATIME");
    CPparams.MAX_CPM_HISTORY_TIME = par("MAX_CPM_HISTORY_TIME");


    /* Attack Parameters Setting */
    CPparams.cpParVar = par("cpParVar");
    CPparams.RandomDistOffsetX = par("RandomDistOffsetX");
    CPparams.RandomDistOffsetY = par("RandomDistOffsetY");
    CPparams.RandomSpeed = par("RandomSpeed");
    CPparams.RandomSpeedOffset = par("RandomSpeedOffset");
    CPparams.RandomAccel = par("RandomAccel");

    /* Detector Parameters Setting */
    CPparams.MaxMapBoundary = par("MaxMapBoundary");
    CPparams.MAX_PLAUSIBLE_ANGLE_CHANGE = par("MAX_PLAUSIBLE_ANGLE_CHANGE");
    CPparams.TOLERANCE_EXCEED_SPEED = par("TOLERANCE_EXCEED_SPEED");
    CPparams.TOLERANCE_DEARTH_SPEED = par("TOLERANCE_DEARTH_SPEED");
    CPparams.SegmentAngle = par("SegmentAngle");

    /* Kalman Parameters Setting */
    CPparams.KALMAN_MIN_POS_RANGE = par("KALMAN_MIN_POS_RANGE");
    CPparams.KALMAN_POS_RANGE = par("KALMAN_POS_RANGE");
    CPparams.MAX_KALMAN_TIME = par("MAX_KALMAN_TIME");
    CPparams.KALMAN_SPEED_RANGE = par("KALMAN_SPEED_RANGE");

    /* RSU Parameters Setting */
    CPparams.RSUorientationAngle = par("RSUorientationAngle");
    CPparams.GenerationInterval = par("GenerationInterval");
    CPparams.RSUintersectionReferenceID = par("RSUintersectionReferenceID");

}



/**
 * @brief print the final tag To json file
 * 
 */
std::string F2MD_RsuCPService::printFinalTagtoJson() {
    std::string tempStr ="";
    JsonWriter jw;
    jw.openJsonElement("final", true);
    return jw.getJsonElement("final");
}



/******************************************************
******   Logging the received CPM information  ********
*******************************************************/
std::string F2MD_RsuCPService::printRXCpmtoJson(const vanetza::asn1::Cpm& msg){

    std::string tempStr ="";

    JsonWriter jw;
   
    jw.openJsonElement("cpm", true);

    tempStr = jw.getSimpleTag("sourceId",std::to_string(msg->header.stationId),true);
    jw.addTagToElement("cpm", tempStr);

    long rfT;
    const auto rftime = msg->payload.managementContainer.referenceTime;
    int res = asn_INTEGER2long(&rftime,&rfT);
    tempStr = jw.getSimpleTag("referenceTime",std::to_string(rfT),true);
    jw.addTagToElement("cpm", tempStr);

    tempStr = jw.getSimpleTag("rx_timestamp",SIMTIME_STR(simTime()),true);
    jw.addFinalTagToElement("cpm", tempStr);
    
    return jw.getJsonElement("cpm");  
}


/******************************************************
******   Logging the transmitted CPM information  ********
*******************************************************/
std::string F2MD_RsuCPService::printTXCpmtoJson(const vanetza::asn1::Cpm& msg) {

    std::string tempStr ="";

    JsonWriter jw;
   
    jw.openJsonElement("cpm", true);

    tempStr = jw.getSimpleTag("sourceId",std::to_string(msg->header.stationId),true);
    jw.addTagToElement("cpm", tempStr);
    
    tempStr = jw.getSimpleTag("tx_timestamp",SIMTIME_STR(simTime()),true);
    jw.addTagToElement("cpm", tempStr);
    
    long rfT;
    const auto rftime = msg->payload.managementContainer.referenceTime;
    int res = asn_INTEGER2long(&rftime,&rfT);
    tempStr = jw.getSimpleTag("referenceTime",std::to_string(rfT),true);
    jw.addTagToElement("cpm", tempStr);

    traci::TraCIGeoPosition geoV;
    geoV.latitude = mGeoPosition->latitude.value();
    geoV.longitude = mGeoPosition->longitude.value();

    auto xyPos = mTraciAPI->convert2D(geoV);
    tempStr = jw.getSimpleTag("Xsumoposition", std::to_string(xyPos.x),true);
    jw.addTagToElement("cpm", tempStr);
    tempStr = jw.getSimpleTag("Ysumoposition", std::to_string(xyPos.y),true);
    jw.addTagToElement("cpm", tempStr);
    tempStr = jw.getSimpleTag("longitude", std::to_string(msg->payload.managementContainer.referencePosition.longitude),true);
    jw.addTagToElement("cpm", tempStr);
    tempStr = jw.getSimpleTag("latitude", std::to_string(msg->payload.managementContainer.referencePosition.latitude),true);
    jw.addTagToElement("cpm", tempStr);
    
    for(int i=0; i< msg->payload.cpmContainers.list.count ; i++){
        if(msg->payload.cpmContainers.list.array[i]->containerId == CpmContainerId_originatingRsuContainer){
                
        }

        if(msg->payload.cpmContainers.list.array[i]->containerId == CpmContainerId_perceivedObjectContainer){
            PerceivedObjectContainer_t* POC = &msg->payload.cpmContainers.list.array[i]->containerData.choice.PerceivedObjectContainer;
            jw.openJsonElement("PerceivedObjectContainer",false);
            tempStr = jw.getSimpleTag("numberOfPerceivedObjects", std::to_string(POC->numberOfPerceivedObjects),true);
            if(POC->numberOfPerceivedObjects > 0){
                jw.addTagToElement("PerceivedObjectContainer", tempStr);
                jw.openJsonElementList("objects");
                //cout << ":::::::::::" << POC->perceivedObjects.list.count << endl;
                for(int p=0; p < POC->perceivedObjects.list.count; p++){
                    jw.openJsonElement("object",true);
                    tempStr = jw.getSimpleTag("objectId", std::to_string(*POC->perceivedObjects.list.array[p]->objectId),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("measurementDeltaTime", std::to_string(POC->perceivedObjects.list.array[p]->measurementDeltaTime),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("position_xCoordinate", std::to_string(POC->perceivedObjects.list.array[p]->position.xCoordinate.value),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("position_yCoordinate", std::to_string(POC->perceivedObjects.list.array[p]->position.yCoordinate.value),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("velocityMagnitudeValue", std::to_string(POC->perceivedObjects.list.array[p]->velocity->choice.polarVelocity.velocityMagnitude.speedValue),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("velocityDirection", std::to_string(POC->perceivedObjects.list.array[p]->velocity->choice.polarVelocity.velocityDirection.value),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("accelerationMagnitudeValue", std::to_string(POC->perceivedObjects.list.array[p]->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("accelerationDirection", std::to_string(POC->perceivedObjects.list.array[p]->acceleration->choice.polarAcceleration.accelerationDirection.value),true);
                    jw.addTagToElement("object", tempStr);     
                    tempStr = jw.getSimpleTag("zAngularVelocity", std::to_string(POC->perceivedObjects.list.array[p]->zAngularVelocity->value),true);
                    jw.addTagToElement("object", tempStr);                                  
                    tempStr = jw.getSimpleTag("zAngle", std::to_string(POC->perceivedObjects.list.array[p]->angles->zAngle.value),true);
                    jw.addFinalTagToElement("object", tempStr);  
                    if (p < POC->perceivedObjects.list.count -1){ 
                        jw.addTagToElement("objects", jw.getJsonElement("object"));
                    }else{
                        jw.addFinalTagToElement("objects", jw.getJsonElement("object"));
                    } 
                }
                
                jw.addFinalTagToElement("PerceivedObjectContainer",jw.getJsonElementList("objects"));
            }else{
                jw.addFinalTagToElement("PerceivedObjectContainer", tempStr);
            }
            jw.addFinalTagToElement("cpm",jw.getJsonElement("PerceivedObjectContainer"));  
       }
    }
    
    return jw.getJsonElement("cpm");  
}


/**********************************************************
******   Logging the local perception information  ********
***********************************************************/
std::string F2MD_RsuCPService::printperceivedlisttoJson(const std::string& title, const TrackedObjectsFilterRange& objs) {
  
    double longitude = mGeoPosition->longitude / vanetza::units::degree;
    double latitude = mGeoPosition->latitude / vanetza::units::degree;
    // TraciAPI convert the Static Node (ITS station - RSU) GeoPosition to Carteisian Position
    auto mRsuPosX = mTraciAPI->convert2D({longitude,latitude}).x;  
    auto mRsuPosY = mTraciAPI->convert2D({longitude,latitude}).y; 
    int i=0;
    int countLostTracingNum=0;
    
    std::string tempStr ="";
    std::string tempStrObj ="";
    JsonWriter jw;
   
    jw.openJsonElement("perceived list", true);

    tempStr = jw.getSimpleTag("timestamp",SIMTIME_STR(simTime()),true);
    jw.addTagToElement("perceived list", tempStr);


    tempStr = jw.getSimpleTag("Xegosumoposition",std::to_string(mRsuPosX),true);
    jw.addTagToElement("perceived list", tempStr);
    
    tempStr = jw.getSimpleTag("Yegopsumoposition",std::to_string(mRsuPosY),true);
    jw.addTagToElement("perceived list", tempStr);

    tempStr = jw.getSimpleTag("longitude",std::to_string(longitude),true);
    jw.addTagToElement("perceived list", tempStr);
    
    tempStr = jw.getSimpleTag("latitude",std::to_string(latitude),true);
    jw.addTagToElement("perceived list", tempStr);

    tempStr = jw.getSimpleTag("orientationangle",std::to_string(CPparams.RSUorientationAngle),true);
    

    int sizeobjs = boost::size(objs);
    
    if (sizeobjs > 0){
        jw.addTagToElement("perceived list", tempStr);
        jw.openJsonElementList("objects");
        for (const auto& obj : objs){
            std::weak_ptr<EnvironmentModelObject> obj_ptr = obj.first;
            if (obj_ptr.expired()){
                countLostTracingNum++;
                continue; /*< objects remain in tracking briefly after leaving simulation */
            }
           
            const VehicleDataProvider& vd = obj_ptr.lock()->getVehicleData();

            jw.openJsonElement("object",true);

            tempStr = jw.getSimpleTag("objectId",std::to_string(vd.station_id()),true);
            jw.addTagToElement("object", tempStr);

            tempStr = jw.getSimpleTag("xdistance",std::to_string(vd.position().x.value() - mRsuPosX),true);
            jw.addTagToElement("object", tempStr);

            tempStr = jw.getSimpleTag("ydistance",std::to_string(vd.position().y.value() - mRsuPosY),true);
            jw.addTagToElement("object", tempStr);
            
            //getting vd.position.x gives omnetpp format
            traci::TraCIGeoPosition geoVobject;
            geoVobject.latitude = vd.latitude().value();
            geoVobject.longitude = vd.longitude().value();
            auto xyPosobject = mTraciAPI.get()->convert2D(geoVobject);

            tempStr = jw.getSimpleTag("Xobjectsumoposition",std::to_string(xyPosobject.x),true);
            jw.addTagToElement("object", tempStr);     

            tempStr = jw.getSimpleTag("Yobjectsumoposition",std::to_string(xyPosobject.y), true);
            jw.addTagToElement("object", tempStr);

            tempStr = jw.getSimpleTag("longitude",std::to_string(vd.longitude().value()),true);
            jw.addTagToElement("object", tempStr);

            tempStr = jw.getSimpleTag("latitude",std::to_string(vd.latitude().value()),true);
            jw.addTagToElement("object", tempStr);
            
            tempStr = jw.getSimpleTag("longspeed",std::to_string(vd.speed().value()),true);
            jw.addTagToElement("object", tempStr);

            tempStr = jw.getSimpleTag("longacceleration",std::to_string(vd.acceleration().value()),true);
            jw.addTagToElement("object", tempStr);

            int tempOrientationAngle = std::round(vd.heading().value() * 1800/PI);
            tempStr = jw.getSimpleTag("orientationangle",std::to_string(tempOrientationAngle),true);
            jw.addTagToElement("object", tempStr);
            
            int tempYawRate = std::round(vd.yaw_rate().value()*180/PI);
            if(tempYawRate > 255){
                tempYawRate = 255;
            }else if(tempYawRate < -255){
                tempYawRate = -255;
            }
            tempStr = jw.getSimpleTag("zangularvelocity",std::to_string(tempYawRate),true);
            jw.addFinalTagToElement("object", tempStr);

            if (i < sizeobjs -1 -countLostTracingNum){ 
                jw.addTagToElement("objects", jw.getJsonElement("object"));
            }else if(i >= sizeobjs -1 -countLostTracingNum){
                jw.addFinalTagToElement("objects", jw.getJsonElement("object"));
            }
            i++;
        }
        jw.addTagToElement("perceived list",jw.getJsonElementList("objects"));
        tempStr = jw.getSimpleTag("NumPerceivedObj", std::to_string(sizeobjs - countLostTracingNum),true);
        jw.addFinalTagToElement("perceived list", tempStr);

    }else{
        jw.addTagToElement("perceived list", tempStr);
        tempStr = jw.getSimpleTag("NumPerceivedObj", std::to_string(boost::size(objs)),true);
        jw.addFinalTagToElement("perceived list", tempStr);
    } 

    return jw.getJsonElement("perceived list");  
} 


void F2MD_RsuCPService::finish(){

    jwritecpmRX.addFinalTagToElement("CPM_RX",printFinalTagtoJson());
    jwritecpmRX.writeHeader();
    jwritecpmRX.addElement(jwritecpmRX.getJsonElementList("CPM_RX"));  
    jwritecpmRX.writeFooter();
    if (outputjsonfilecpm.is_open()){
        outputjsonfilecpm << jwritecpmRX.getOutString()<<std::endl; 
    }  

    jwritecpmTX.addFinalTagToElement("CPM_TX",printFinalTagtoJson());
    jwritecpmTX.writeHeader();
    jwritecpmTX.addElement(jwritecpmTX.getJsonElementList("CPM_TX"));  
    jwritecpmTX.writeFooter();
    if (outputjsonfilecpmEmit.is_open()){
        outputjsonfilecpmEmit << jwritecpmTX.getOutString()<<std::endl; 
    } 

    jwritelem.addFinalTagToElement("LP",printFinalTagtoJson());
    jwritelem.writeHeader();
    jwritelem.addElement(jwritelem.getJsonElementList("LP"));
    jwritelem.writeFooter();
    if (outputjsonfilelem.is_open()){
        outputjsonfilelem << jwritelem.getOutString() <<std::endl; 
    } 

    outputjsonfilecpm.close();
    outputjsonfilelem.close();
    outputjsonfilecpmEmit.close();
    outputfilelog.close();

}

} // namespace artery
