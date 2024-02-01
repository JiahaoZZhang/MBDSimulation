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

#include "artery/application/CpmObject.h"
#include "F2MD_CPService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
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
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h> 


#define RXSocketPORT 65432
#define TXSocketPORT 65433
#define ExtPerSocketPORT 65434

int mypythonsocketRX = 0; 
int mypythonsocketTX = 0; 
int mypythonsocketEX = 0; 
std::map<int,std::string> Artery2SumoID;

namespace artery
{

using namespace omnetpp;
using namespace std;

Define_Module(F2MD_CPService)

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");

F2MD_CPService::F2MD_CPService(){
}

/**
   *  @brief Initialise CPM Service and CPM Facility for further F2MD Processing
*/
void F2MD_CPService::initialize()
{
    // std::cout << "entering initialize in CPService at "<<SIMTIME_STR(simTime())<<std::endl;

    ItsG5BaseService::initialize();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    mTimer = &getFacilities().get_const<Timer>();
    mlocalEnvmod = &getFacilities().get_const<LocalEnvironmentModel>();
    mglobleEnvmod = &getFacilities().get_mutable<GlobalEnvironmentModel>();
    mTraci = mVehicleController->getTraCI();


    // Parameters setting
    mGenCpmMax = par("maxInterval");
    mFixedRate = par("fixedRate");
    Test_mode = par("Test_mode");
    mLastCpmTimestamp = simTime();
    initParams();

    // Initialize Pseudonym Change Policy
    myPcType = CPparams.PC_TYPE;
    pseudoNum = 0;
    //Node Module Acces and Mac Address
    node = inet::findContainingNode(this);

    myMacId = inet::getModuleFromPar<inet::ieee80211::Ieee80211Mac>(par("pathToMacModule"),node)->getAddress();
    auto SUMO_id = mVehicleController->getVehicleId();
    auto vehicle_api = mVehicleController->getTraCI()->vehicle;
    auto coordPosInit = vehicle_api.getPosition(SUMO_id);
    mPCPolicyCPM = PCPolicyCPM(coordPosInit, &CPparams);
    mPCPolicyCPM.setCurPosition(&coordPosInit);
    mPCPolicyCPM.setMyId(&myMacId);
    mPCPolicyCPM.setMyPseudonym(&myPseudonym);
    mPCPolicyCPM.setPseudoNum(&pseudoNum);
    myPseudonym = mPCPolicyCPM.getNextPseudonym()  % 4294967295;
    myPseudonym = myPseudonym % 4294967295;


    // ------ Initialization of F2MD Module ------ Start
    mF2MDFacility = F2MD_CPFacility(mVehicleDataProvider, mVehicleController, CPparams, &myPseudonym);
    mF2MDFacility.initialize();

    mExtendPerceptionTable = ExtendPerceptionTable();
    // ------ Initialization of F2MD Module ------ End


    // generation rate boundaries
	mGenCpm = mGenCpmMax;
    int myId = mVehicleDataProvider->getStationId();
    double attackType = mF2MDFacility.getAttackType();


    string str_id = to_string(myId);
    string outlogfile_name = "output/Log/log_vehicle_";    
    outlogfile_name.append(str_id);
    outputfilelog.open(outlogfile_name,ios::out);

    // For Json
    string suffix = ".json";

    string outjsonfilecpm_name = "output/GENUINE/CPM/RX/logjson_cpm_rx";
    outjsonfilecpm_name.append(str_id);
    outjsonfilecpm_name.append (suffix);
    outputjsonfilecpm.open(outjsonfilecpm_name,ios::out);
    jwritecpmRX.openJsonElementList("CPM_RX");

    string outjsonfilecpmEmit_name = "output/GENUINE/CPM/TX/logjson_cpm_tx";
    outjsonfilecpmEmit_name.append(str_id);
    outjsonfilecpmEmit_name.append (suffix);
    outputjsonfilecpmEmit.open(outjsonfilecpmEmit_name,ios::out);
    jwritecpmTX.openJsonElementList("CPM_TX");

    // Ground Truth
    string outlemjsonfile_name = "output/GENUINE/LP/logjson_lp";
    outlemjsonfile_name.append(str_id);
    outlemjsonfile_name.append (suffix);
    outputjsonfilelem.open(outlemjsonfile_name,ios::out);
    jwritelem.openJsonElementList("LP");

}



/**
   *  @brief  called periodically by the middleware. UpdateFreq is defined in omnetpp.ini
*/
void F2MD_CPService::trigger()
{  
    SimTime& T_GenCpm = mGenCpm;
	const SimTime& T_GenCpmMax = mGenCpmMax;
    const SimTime T_elapsed = simTime() - mLastCpmTimestamp;


    // The condition needs to be completed
    if (T_elapsed >= T_GenCpmMax)
    {
        sendCpm(simTime());
        mExtendPerceptionTable.updateTable(simTime().inUnit(SIMTIME_MS));
    }
}


/**
   *  @brief  called by the middleware each time a CPM is received by this node.
*/
void F2MD_CPService::indicate(const vanetza::btp::DataIndication& indication, std::unique_ptr<vanetza::UpPacket> packet)
{

    Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
 
    auto neighborMAcId = indication.source_position.gn_addr.mid();
    
   string macid="";
    
    for (int i =0; i<neighborMAcId.octets.size();i++)
    {
        macid.append(std::to_string(neighborMAcId.octets[i]));
    } 

   
    const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);
    
    const auto egoStationID = mVehicleDataProvider->station_id();
    // std::cout << egoStationID <<" packet reception at "<< SIMTIME_STR(simTime())<<std::endl;
    if (outputfilelog.is_open()){ outputfilelog << " -------> packet reception at "<< SIMTIME_STR(simTime())<<std::endl; }

    if (cpm && cpm->validate()) {

        CpmObject obj = visitor.shared_wrapper;
        emit(scSignalCpmReceived, &obj);
        const vanetza::asn1::Cpm& msg = obj.asn1();
        
        const CollectivePerceptionMessage_t myMsg = *msg;



        // Pass the received CPM to the F2MD Processing facility for further analysis (MbD checks,...)
        // double checkfailed = 0;
        mCpmCheckList.resetAll();
        ExtendPerceptionTable newExtendPerceptionTable;
        // simple reaction to ignore the CPM when detect a misbehavior
        if(Test_mode == 0){ // active Misbehavior detection
            double checkfailed = mF2MDFacility.onCPM(msg, neighborMAcId, mCpmCheckList);
            if (checkfailed == 0){
                if (MisbehaviorList.find(myMsg.header.stationId) == MisbehaviorList.end()){
                    auto currentTime = simTime().inUnit(SIMTIME_MS);
                    mExtendPerceptionTable.updateTable(&msg,currentTime);
                }
            }else{
                MisbehaviorList.insert(myMsg.header.stationId);
            }
        }else{
            auto currentTime = simTime().inUnit(SIMTIME_MS);
            mExtendPerceptionTable.updateTable(&msg,currentTime);
        }


        newExtendPerceptionTable = this->mExtendPerceptionTable;
        // Display important fields contained in the received BSM, to check the reception (Useful for debugging)
        checkReception(msg, newExtendPerceptionTable);      

        if (outputfilelog.is_open()){ 
            outputfilelog << " -------> station :"<< myMsg.header.stationId << "  >>>> Check List ::: " << endl; 
            outputfilelog << " distance Plausibility :: "<< mCpmCheckList.getdistancePlausibility() << endl;
            outputfilelog << " speed Plausibility :: "<< mCpmCheckList.getspeedPlausibility() << endl;

            outputfilelog << " speed Consistency :: "<< mCpmCheckList.getspeedConsistency() << endl;
            outputfilelog << " positionSpeed Consistency :: "<< mCpmCheckList.getpositionSpeedConsistency() << endl;

            outputfilelog << " kalman position Consistency :: "<< mCpmCheckList.getkalmanPositionConsistency() << endl;
            outputfilelog << " kalman speed Consistency :: "<< mCpmCheckList.getkalmanSpeedConsistency() << endl;

            outputfilelog << " kalman position speed Consistency P :: "<< mCpmCheckList.getkalmanPositionSpeedConsistancyP() << endl;
            outputfilelog << " kalman position speed Consistency S :: "<< mCpmCheckList.getkalmanPositionSpeedConsistancyS() << endl;
            
            outputfilelog << " detected misbehavior objects :: " << endl;
            for(const auto& p: mCpmCheckList.getAttackedObject()){
                outputfilelog << "   >>> Attacked ID :: " << p.ObjectID << endl;
                outputfilelog << "   >>> Checkfailed :: " << p.Checkfailed << endl;
                outputfilelog << " ----------------------------" << endl;
            }
            outputfilelog << " report :: "<< mCpmCheckList.getreport() << "\n" << endl;
        }
    }
} 



/**
   *  @brief  print in QtEnv what is contained in the cpm received. Useful for debugging.
   *  @param vanetza::asn1::Cpm& cpm received
*/
void F2MD_CPService::checkReception(const vanetza::asn1::Cpm& msg, ExtendPerceptionTable newExtendPerceptionTable){

    auto& allObjects = mlocalEnvmod->allObjects();
    jwritecpmRX.addTagToElement("CPM_RX", printRXCpmtoJson(msg,newExtendPerceptionTable));
    // jwriteextp.addTagToElement("EP",printextendedperceivedlisttoJson(mExtendPerceptionTable));    
    jwritelem.addTagToElement("LP",printperceivedlisttoJson("Radar Sensor Object List", filterBySensorCategory(allObjects, "Radar")));

    RXSocketCpmToPython(RXSocketPORT, msg, mVehicleDataProvider);
    EXSocketCpmToPython(ExtPerSocketPORT,printextendedperceivedlisttoJson(newExtendPerceptionTable));

}



const Timer* F2MD_CPService::getTimer() const
{
    return mTimer;
}


/**
   *  @brief  create, fill, and send a cpm with correct informations
   *  @param simTime_t current time
*/
void F2MD_CPService::sendCpm(const SimTime& T_now)
{

    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));

    // Create a new CPM
    vanetza::asn1::Cpm cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, genDeltaTimeMod,*mlocalEnvmod);
    mLastCpmTimestamp = T_now;

    // print each emitted CPM messages.
    jwritecpmTX.addTagToElement("CPM_TX", printTXCpmtoJson(cpm));

    TXSocketCpmToPython(TXSocketPORT, mVehicleController);
    

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

    emit(scSignalCpmSent, &obj);
    
}

/**
   *  @brief  create and fill a cpm with correct informations
   *  @param VehicleDataProvider& my vehicle data provider
   *  @param uint16_t genDeltaTime
   *  @param LocalEnvironmentModel& my local env model
*/
vanetza::asn1::Cpm F2MD_CPService::createCollectivePerceptionMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime, const LocalEnvironmentModel& lem)
{
    
    vanetza::asn1::Cpm message;
    ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageId = MessageId_cpm;

    // Note: with pseudonym change
    // Station ID interval: [0, 4294967295]
    // The ITS-S ID may be a pseudonym. It may change over space and/or over time.
    if (CPparams.EnablePC) {
        auto SUMO_id = mVehicleController->getVehicleId();
        auto vehicle_api = mVehicleController->getTraCI()->vehicle;
        coordPos = vehicle_api.getPosition(SUMO_id);
        mPCPolicyCPM.setCurPosition(&coordPos);
        mPCPolicyCPM.checkPseudonymChange(CPparams.PC_TYPE);
        myPseudonym = myPseudonym % 4294967295;
    }else{
        myPseudonym = vdp.getStationId() % 4294967295;
    }

    header.stationId = myPseudonym;
	//header.stationId = vdp.getStationId() ;// artery allocated ID
    if(Artery2SumoID.find(vdp.getStationId()) == Artery2SumoID.end()){
        auto artery_id = vdp.getStationId();
        auto sumo_id = mVehicleController->getVehicleId();
        Artery2SumoID.insert(std::pair<int,std::string>(artery_id, sumo_id));
    }


    CpmPayload_t& CpmPayload = (*message).payload;
    // Create Management Container
    ManagementContainer_t& management = CpmPayload.managementContainer;
    long referenceTime = countTaiMilliseconds(mTimer->getCurrentTime());
    int ret = 0;
    ret += asn_long2INTEGER(&management.referenceTime, referenceTime);
    assert(ret == 0);
    management.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    management.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    long mlatitude = std::round(vdp.latitude().value() * 1e7);
    long mlongitude = std::round(vdp.longitude().value() * 1e7);
    if(mlatitude > (-9e8) && mlatitude < 9e8+1){
        management.referencePosition.latitude = mlatitude;
    }else{
        management.referencePosition.latitude = Latitude_unavailable;
    }
    if(mlongitude > (-18e8) && mlongitude < 18e8+1){
        management.referencePosition.longitude = mlongitude;
    }else if(mlongitude == -18e8){
        management.referencePosition.longitude = Longitude_valueNotUsed;
    }else{
        management.referencePosition.longitude = Longitude_unavailable;
    }

    management.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
    management.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    management.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
 
    // add all containers
    ConstraintWrappedCpmContainers_t* containersData = vanetza::asn1::allocate<ConstraintWrappedCpmContainers_t>();

    
    // -----------------------------------
    // Create OriginatingVehicle Container 
    // -----------------------------------
    WrappedCpmContainer_t* WrappedOriginatingVehicleContainer = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    WrappedOriginatingVehicleContainer->containerId = CpmContainerId_originatingVehicleContainer;
    WrappedOriginatingVehicleContainer->containerData.present = WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer;
    OriginatingVehicleContainer_t&  mOriginatingVehicleContainer = WrappedOriginatingVehicleContainer->containerData.choice.OriginatingVehicleContainer;
    // orientationAngle != heading according to EN 302 890-2, but in the simulateur, we consider orientationAngle = heading
    // Wgs84AngleValue, Unit: 0,1 degrees
    auto orientationAngleValue = std::round(vdp.heading().value() * 1800 / PI);
    if((orientationAngleValue >= 0) && (orientationAngleValue <= 3600)){
        mOriginatingVehicleContainer.orientationAngle.value = orientationAngleValue; // heading(), radian from north, clockwise
    }else{
        mOriginatingVehicleContainer.orientationAngle.value = Wgs84AngleValue_unavailable;
    }
    // cout << "stationID::" << vdp.getStationId() << "; orientationAngleValue::" << orientationAngleValue << endl;

    mOriginatingVehicleContainer.orientationAngle.confidence = Wgs84AngleConfidence_unavailable;
    ASN_SEQUENCE_ADD(containersData, WrappedOriginatingVehicleContainer);  


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
        for (const auto& obj : objects){ 
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


            // add perceived object into artery sumo id conversion map

            if(Artery2SumoID.find(vd.getStationId()) == Artery2SumoID.end()){
                auto artery_id = vd.getStationId();
                auto sumo_id = vd.sumo_id();
                // cout << "artery_id::" << artery_id <<";;vd_sumo_id::" << sumo_id << endl;
                Artery2SumoID.insert(std::pair<int,std::string>(artery_id, sumo_id));
            }
            // cout << "sumo_id::" << vd.sumo_id() << endl;
            // cout <<  "position x::" << vd.position().x.value() << endl;

            /*************************************************************************************
            // �DeltaTimeMilliSecondSigned represents a signed difference in time with respect to a reference time
            //  Unit: 0,001 s
            *************************************************************************************/
            if (deltaTimeOfMeasurement > 2046){ /* CPM ASN1 requirements */
                object->measurementDeltaTime = 2047;
            }else if(deltaTimeOfMeasurement < -2048){
                object->measurementDeltaTime = -2048;
            }else{
                object->measurementDeltaTime = deltaTimeOfMeasurement;
            }
            // cout << "stationID::" << vdp.getStationId() << "; deltaTimeOfMeasurement::" << deltaTimeOfMeasurement << endl;

            /*************************************************************************************
            // need to correct the vd.position and vdp.postion
            // because this position in OMNET++ is not the cartesian position in sumo
            // In OMNET++: y axis is growing to bottom; x axis is growing to right
            // In SUMO: y axis is growing to top; x axis is growing to right
            // the X coordinate value with the associated confidence level
            *************************************************************************************/
            long xCoordinate = std::round((vd.position().x.value() - vdp.position().x.value()) * 1e2); // CartesianCoordinate, Unit:0,01 m
            if (xCoordinate > 131070){
                object->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
            }else if(xCoordinate < -131072){
                object->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
            }else{
                object->position.xCoordinate.value = xCoordinate;
            }
            // cout << "stationID::" << vdp.getStationId() << "; xCoordinate::" << xCoordinate << endl;
            object->position.xCoordinate.confidence = CoordinateConfidence_unavailable;


            /*************************************************************************************
            // the Y coordinate value with the associated confidence level
            // Y axis is OMNET coordinate system
            *************************************************************************************/
            long yCoordinate = std::round((vdp.position().y.value() - vd.position().y.value()) * 1e2);
            if (yCoordinate > 131070){
                object->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
            }else if(yCoordinate < -131072){
                object->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
            }else{
                object->position.yCoordinate.value = yCoordinate;
            }
            // cout << "stationID::" << vdp.getStationId() << "; yCoordinate::" << yCoordinate << endl;
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
            // cout << "stationID::" << vdp.getStationId() << "; PreceivedObjSpeed::" << PreceivedObjSpeed << endl;

            object->velocity->choice.polarVelocity.velocityMagnitude.speedConfidence = SpeedConfidence_unavailable;


            /***********************************************************************************************************
            // CartesianAngleValue represents an angle value described in a local Cartesian coordinate system
            // per default counted positive in a right-hand local coordinate system from the abscissa. degree from right (x axis), anti-clockwise
            // need to change the reference of heading()
            // Ines : set it to heading now, we need to fix it with Jiahao
            //object->velocity->choice.polarVelocity.velocityDirection.value = CartesianAngleValue_unavailable;
            ************************************************************************************************************/
            auto velocityDirectionValue = std::round(vd.heading().value() * 1800 / PI);
            if((velocityDirectionValue >= 0) && (velocityDirectionValue < 3600)){
                object->velocity->choice.polarVelocity.velocityDirection.value =  velocityDirectionValue; // heading(), radian from north, clockwise
            }else if(velocityDirectionValue == 3600){
                object->velocity->choice.polarVelocity.velocityDirection.value = 0;
            }else{
                object->velocity->choice.polarVelocity.velocityDirection.value = CartesianAngleValue_unavailable;
            }
            // cout << "stationID::" << vdp.getStationId() << "; velocityDirectionValue::" << velocityDirectionValue << endl;

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
            // cout << "stationID::" << vdp.getStationId() << "; PreceivedObjAcceleration::" << PreceivedObjAcceleration << endl;
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
            auto zAngleValue = (temp % 3600 + 3600) % 3600;
            if((zAngleValue >= 0) && (zAngleValue < 3600)){
                object->angles->zAngle.value = zAngleValue;
            }else if(zAngleValue == 3600){
                object->angles->zAngle.value = 0;
            }else{
                object->angles->zAngle.value = CartesianAngleValue_unavailable;
            }
            // cout << "stationID::" << vdp.getStationId() << "; zAngleValue::" << zAngleValue << endl;
            object->angles->zAngle.confidence = AngleConfidence_unavailable;
            

            //added by Ines
            object->zAngularVelocity= vanetza::asn1::allocate<CartesianAngularVelocityComponent_t>();
            int tempzAV = std::round(vd.yaw_rate().value()*180/PI);
            // cout << "zAngularVelocity::" <<  vd.yaw_rate().value() << ";;;" << tempzAV << endl;
            // cout << "stationID::" << vdp.getStationId() << "; tempzAV::" << tempzAV << endl;
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
        // cout << "stationID::" << vdp.getStationId() << "; SensorINFO sensorID::" << SensorInformation->sensorId << endl;

        SensorInformation->sensorType = SensorType_radar;
        SensorInformation->shadowingApplies = false;
        SensorInformation->perceptionRegionShape = vanetza::asn1::allocate<Shape_t>();
        SensorInformation->perceptionRegionShape->present = Shape_PR_radial;
        auto radialRange = std::round(sensor->getFieldOfView().range.value() * 10);
        // cout << "stationID::" << vdp.getStationId() << "; SensorINFO radialRange::" << radialRange << endl;
        if((radialRange >= 0 ) && (radialRange < 4095)){
            SensorInformation->perceptionRegionShape->choice.radial.range = radialRange;
        }else if(radialRange < 0){
            SensorInformation->perceptionRegionShape->choice.radial.range = 0;
        }else{
            SensorInformation->perceptionRegionShape->choice.radial.range = 4095;
        }
        

        // Wgs84AngleValue, Unit: 0,1 degrees
        // getFieldOfView().angle.value(), Unit: 1 degree
        int temp1 = std::round(vdp.heading().value() * 1800 / PI - sensor->getFieldOfView().angle.value()*10/2); 
        int temp2 = std::round(vdp.heading().value() * 1800 / PI + sensor->getFieldOfView().angle.value()*10/2);      
        int OpeningAngleStart =  (temp1 % 3600 + 3600) % 3600;
        int OpeningAngleEnd = (temp2 % 3600 + 3600) % 3600;
        if((OpeningAngleStart >= 0) && (OpeningAngleStart < 3600)){
            SensorInformation->perceptionRegionShape->choice.radial.stationaryHorizontalOpeningAngleStart = OpeningAngleStart;
        }else if(OpeningAngleStart == 3600){
            SensorInformation->perceptionRegionShape->choice.radial.stationaryHorizontalOpeningAngleStart = 0;
        }else{
            SensorInformation->perceptionRegionShape->choice.radial.stationaryHorizontalOpeningAngleStart = Wgs84AngleValue_unavailable;
        }
        // cout << "stationID::" << vdp.getStationId() << "; SensorINFO OpeningAngleStart::" << OpeningAngleStart << endl;

        if((OpeningAngleEnd >= 0) && (OpeningAngleEnd < 3600)){
            SensorInformation->perceptionRegionShape->choice.radial.stationaryHorizontalOpeningAngleEnd = OpeningAngleEnd;
        }else if(OpeningAngleEnd == 3600){
            SensorInformation->perceptionRegionShape->choice.radial.stationaryHorizontalOpeningAngleEnd = 0;
        }else{
            SensorInformation->perceptionRegionShape->choice.radial.stationaryHorizontalOpeningAngleEnd = Wgs84AngleValue_unavailable;
        }
        // cout << "stationID::" << vdp.getStationId() << "; SensorINFO OpeningAngleEnd::" << OpeningAngleEnd << endl;

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

        long timeofmeasurement = countTaiMilliseconds(mTimer->getTimeFor(vdp.updated()));
        long deltaTimeOfMeasurement = referenceTime - timeofmeasurement;
        mPerceptionRegion->measurementDeltaTime = deltaTimeOfMeasurement;
        mPerceptionRegion->shadowingApplies = false;
        mPerceptionRegion->perceptionRegionShape.present = Shape_PR_radial;
        auto radialRange = std::round(sensor->getFieldOfView().range.value() * 10);
        // cout << "stationID::" << vdp.getStationId() << "; radialRange::" << radialRange << endl;
        if((radialRange >= 0 ) && (radialRange < 4095)){
            mPerceptionRegion->perceptionRegionShape.choice.radial.range =radialRange;
        }else if(radialRange < 0){
            mPerceptionRegion->perceptionRegionShape.choice.radial.range = 0;
        }else{
            mPerceptionRegion->perceptionRegionShape.choice.radial.range = 4095;
        }

        int temp1 = std::round(vdp.heading().value() * 1800 / PI - sensor->getFieldOfView().angle.value()*10/2); 
        int temp2 = std::round(vdp.heading().value() * 1800 / PI + sensor->getFieldOfView().angle.value()*10/2);
        int OpeningAngleStart =  (temp1 % 3600 + 3600) % 3600;
        int OpeningAngleEnd = (temp2 % 3600 + 3600) % 3600;
        
        if((OpeningAngleStart >= 0) && (OpeningAngleStart < 3600)){
            mPerceptionRegion->perceptionRegionShape.choice.radial.stationaryHorizontalOpeningAngleStart = OpeningAngleStart;
        }else if(OpeningAngleStart == 3600){
            mPerceptionRegion->perceptionRegionShape.choice.radial.stationaryHorizontalOpeningAngleStart = 0;
        }else{
            mPerceptionRegion->perceptionRegionShape.choice.radial.stationaryHorizontalOpeningAngleStart = Wgs84AngleValue_unavailable;
        }
        // cout << "stationID::" << vdp.getStationId() << "; OpeningAngleStart::" << OpeningAngleStart << endl;

        if((OpeningAngleEnd >= 0) && (OpeningAngleEnd < 3600)){
            mPerceptionRegion->perceptionRegionShape.choice.radial.stationaryHorizontalOpeningAngleEnd = OpeningAngleEnd;
        }else if(OpeningAngleEnd == 3600){
            mPerceptionRegion->perceptionRegionShape.choice.radial.stationaryHorizontalOpeningAngleEnd = 0;
        }else{
            mPerceptionRegion->perceptionRegionShape.choice.radial.stationaryHorizontalOpeningAngleEnd = Wgs84AngleValue_unavailable;
        }
        // cout << "stationID::" << vdp.getStationId() << "; OpeningAngleEnd::" << OpeningAngleEnd << endl;

        mPerceptionRegion->perceptionRegionConfidence = ConfidenceLevel_unavailable;
        mPerceptionRegion->shadowingApplies = false;

        ASN_SEQUENCE_ADD(mPerceptionRegionContainer, mPerceptionRegion);
    }
    ASN_SEQUENCE_ADD(containersData, WrappedPerceptionRegionContainer);


    CpmPayload.cpmContainers = *containersData;





    //*************************************
    //     trigger attacks module
    //*************************************
    mF2MDFacility.induceAttack(&CpmPayload);
    // std::cout << "perceptionData value::" << cpm.cpmParameters.perceptionData <<std::endl;
    // if(cpm.cpmParameters.perceptionData != 0){
    //     FREEMEM(cpm.cpmParameters.perceptionData);
    //     cpm.cpmParameters.perceptionData = nullptr;
    // }


    
    // if (header.stationId == 234){
    //     cout << *CpmPayload.cpmContainers.list.array[1]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[1]->objectId << endl;
    //     cout << CpmPayload.cpmContainers.list.array[1]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count << endl;
    //     cout << CpmPayload.cpmContainers.list.array[1]->containerData.choice.PerceivedObjectContainer.numberOfPerceivedObjects << endl;
    // }

    std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid CPM message %s", error.c_str());
	}


    //std::cout << vdp.station_id()<<" packet transmission at "<< SIMTIME_STR(simTime())<<std::endl;




    return message;
}



/**
   *  @brief  stores the parameters defined in omnetpp.ini in the F2MD_CPMParameters.h file so that they can be used in the simulation.
*/
void F2MD_CPService::initParams(){

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

    CPparams.KeepSameID = par("KeepSameID");
    CPparams.VEHICLE_LENGTH = par("VEHICLE_LENGTH");
    CPparams.VEHICLE_WIDTH = par("VEHICLE_WIDTH");

    CPparams.EnablePC = par("EnablePC");

    /* Attack Parameters Setting */
    CPparams.cpParVar = par("cpParVar");
    CPparams.RandomDistOffsetX = par("RandomDistOffsetX");
    CPparams.RandomDistOffsetY = par("RandomDistOffsetY");
    CPparams.RandomSpeed = par("RandomSpeed");
    CPparams.RandomSpeedOffset = par("RandomSpeedOffset");
    CPparams.RandomAccel = par("RandomAccel");
    CPparams.MaxRadius = par("MaxRadius");

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
    
    /* Pseudonym Parameters */
    CPparams.Period_Change_Time = par("Period_Change_Time");
    CPparams.Tolerance_Buffer = par("Tolerance_Buffer");
    CPparams.Period_Change_Distance = par("Period_Change_Distance");
    CPparams.Random_Change_Chance = par("Random_Change_Chance");
    CPparams.PC_TYPE = pseudoChangeTypes::intPseudoChange[par("PC_TYPE").intValue()];

}

void F2MD_CPService::finish(){

    mF2MDFacility.finish();
    jwritecpmRX.addFinalTagToElement("CPM_RX",printFinalTagtoJson());
    jwritecpmRX.writeHeader();
    jwritecpmRX.addElement(jwritecpmRX.getJsonElementList("CPM_RX"));  
    jwritecpmRX.writeFooter();
    if (outputjsonfilecpm.is_open()){
        outputjsonfilecpm << jwritecpmRX.getOutString()<<std::endl; 
    } 

    jwritelem.addFinalTagToElement("LP",printFinalTagtoJson());
    jwritelem.writeHeader();
    jwritelem.addElement(jwritelem.getJsonElementList("LP"));
    jwritelem.writeFooter();
    if (outputjsonfilelem.is_open()){
        outputjsonfilelem << jwritelem.getOutString() <<std::endl; 
    } 

    jwritecpmTX.addFinalTagToElement("CPM_TX",printFinalTagtoJson());
    jwritecpmTX.writeHeader();
    jwritecpmTX.addElement(jwritecpmTX.getJsonElementList("CPM_TX"));  
    jwritecpmTX.writeFooter();
    if (outputjsonfilecpmEmit.is_open()){
        outputjsonfilecpmEmit << jwritecpmTX.getOutString()<<std::endl; 
    } 


    outputfilelog.close();
    outputjsonfilecpm.close();
    outputjsonfilelem.close();
    outputjsonfilecpmEmit.close();

    cSimpleModule::finish();
}


/**
 * @brief print the final tag To json file
 * 
 */
std::string F2MD_CPService::printFinalTagtoJson() {
    std::string tempStr ="";
    JsonWriter jw;
    jw.openJsonElement("final", true);
    return jw.getJsonElement("final");
}

/**
 * @brief print the receieved CPM information 
 * 
 */
std::string F2MD_CPService::printRXCpmtoJson(const vanetza::asn1::Cpm& msg, ExtendPerceptionTable mExtendPerceptionTable){

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
    jw.addTagToElement("cpm", tempStr);
    
    jw.openJsonElementList("Extend_Perception_List");
    auto mExtendPerceptionList = mExtendPerceptionTable.getExtendPerceptionList();
    for(auto it = mExtendPerceptionList.begin(); it != mExtendPerceptionList.end(); it++){
        jw.openJsonElement("ExtendPerceptionObj",true);
        
        tempStr = jw.getSimpleTag("LastUpdateTime",std::to_string((*it).LastUpdateTime),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("LastRecordTime",std::to_string((*it).LastRecordTime),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("ObserverID",std::to_string((*it).ObserverID),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        // auto info_it = mExtendPerceptionTable.getObjInfo((*it).PerceivedObjectID);
        // cout << "address::" << &(*it) << ";;;; size::" << mExtendPerceptionList.size() << endl;
        // cout << "mySumoId::"<< mVehicleController->getVehicleId() <<endl;
        // cout << "ObserverSumoID::"<< Artery2SumoID.find((*it).ObserverID)->second <<endl;
        // cout << "SumoPerceivedObjectID::"<< Artery2SumoID.find((*it).PerceivedObjectID)->second <<endl;
        // cout << "speedvalue::"<< (*it).ObjectInfo.velocity->choice.polarVelocity.velocityMagnitude.speedValue <<endl;

        tempStr = jw.getSimpleTag("speedValue",std::to_string((*it).ObjectInfo.velocity->choice.polarVelocity.velocityMagnitude.speedValue),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("xCoordinateValue",std::to_string((*it).ObjectInfo.position.xCoordinate.value),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("yCoordinateValue",std::to_string((*it).ObjectInfo.position.yCoordinate.value),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("PerceivedObjectID",std::to_string((*it).PerceivedObjectID),true);
        jw.addFinalTagToElement("ExtendPerceptionObj", tempStr);

        if(it == std::prev(mExtendPerceptionList.end())){
            jw.addFinalTagToElement("Extend_Perception_List",jw.getJsonElement("ExtendPerceptionObj"));
        }else{
            jw.addTagToElement("Extend_Perception_List",jw.getJsonElement("ExtendPerceptionObj"));
        }
    }
    jw.addFinalTagToElement("cpm", jw.getJsonElementList("Extend_Perception_List"));

    return jw.getJsonElement("cpm");  
}


/**
 * @brief print the emitted CPM information 
 * 
 */
std::string F2MD_CPService::printTXCpmtoJson(const vanetza::asn1::Cpm& msg) {

    std::string tempStr ="";

    JsonWriter jw;
   
    jw.openJsonElement("cpm", true);

    tempStr = jw.getSimpleTag("sourcePseudoId",std::to_string(msg->header.stationId),true);
    jw.addTagToElement("cpm", tempStr);
    tempStr = jw.getSimpleTag("PseudonymChangeStrategy",pseudoChangeTypes::PseudoChangeNames[CPparams.PC_TYPE],false);
    jw.addTagToElement("cpm", tempStr); 
    tempStr = jw.getSimpleTag("sourceId",std::to_string(mVehicleDataProvider->getStationId()),true);
    jw.addTagToElement("cpm", tempStr);    
    tempStr = jw.getSimpleTag("sourceSumoId",mVehicleController->getVehicleId(),false);
    jw.addTagToElement("cpm", tempStr);
    
    auto mbtype = mF2MDFacility.getMbType();
    tempStr = jw.getSimpleTag("misbehaviorType",mbTypes::mbNames[mbtype],false);
    jw.addTagToElement("cpm", tempStr);   
    
    tempStr = jw.getSimpleTag("tx_timestamp",SIMTIME_STR(simTime()),true);
    jw.addTagToElement("cpm", tempStr);
    
    long rfT;
    const auto rftime = msg->payload.managementContainer.referenceTime;
    int res = asn_INTEGER2long(&rftime,&rfT);
    tempStr = jw.getSimpleTag("referenceTime",std::to_string(rfT),true);
    jw.addTagToElement("cpm", tempStr);

    //getting vdp.position.x gives omnetpp format
    traci::TraCIGeoPosition geoV;
    geoV.latitude = mVehicleDataProvider->latitude().value();
    geoV.longitude = mVehicleDataProvider->longitude().value();
    auto xyPos = mVehicleController->getTraCI().get()->convert2D(geoV);
    tempStr = jw.getSimpleTag("Xsumoposition", std::to_string(xyPos.x),true);
    jw.addTagToElement("cpm", tempStr);
    tempStr = jw.getSimpleTag("Ysumoposition", std::to_string(xyPos.y),true);
    jw.addTagToElement("cpm", tempStr);
    tempStr = jw.getSimpleTag("longitude", std::to_string(msg->payload.managementContainer.referencePosition.longitude),true);
    jw.addTagToElement("cpm", tempStr);
    tempStr = jw.getSimpleTag("latitude", std::to_string(msg->payload.managementContainer.referencePosition.latitude),true);
    jw.addTagToElement("cpm", tempStr);

    tempStr = jw.getSimpleTag("longspeed", std::to_string(mVehicleDataProvider->speed().value()),true);
    jw.addTagToElement("cpm", tempStr);

    tempStr = jw.getSimpleTag("longacceleration", std::to_string(mVehicleDataProvider->acceleration().value()),true);
    jw.addTagToElement("cpm", tempStr);

    int tempAV = std::round(mVehicleDataProvider->yaw_rate().value()*180/PI);
    tempStr = jw.getSimpleTag("zAngularVelocity", std::to_string(tempAV),true);
    jw.addTagToElement("cpm", tempStr);
    
    for(int i=0; i< msg->payload.cpmContainers.list.count ; i++){
       if(msg->payload.cpmContainers.list.array[i]->containerId == CpmContainerId_originatingVehicleContainer){
            tempStr = jw.getSimpleTag("orientationAngle", std::to_string(msg->payload.cpmContainers.list.array[i]->containerData.choice.OriginatingVehicleContainer.orientationAngle.value),true);
            jw.addTagToElement("cpm", tempStr);
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


/**
 * @brief print the local perception LEM
 * 
 */
std::string F2MD_CPService::printperceivedlisttoJson(const std::string& title, const TrackedObjectsFilterRange& objs) {
    int i=0;
    int countLostTracingNum=0;
    
    std::string tempStr ="";
    std::string tempStrObj ="";
    JsonWriter jw;
   
    jw.openJsonElement("perceived list", true);

    tempStr = jw.getSimpleTag("timestamp",SIMTIME_STR(simTime()),true);
    jw.addTagToElement("perceived list", tempStr);

    //getting vdp.position.x gives omnetpp format
    traci::TraCIGeoPosition geoV;
    geoV.latitude = mVehicleDataProvider->latitude().value();
    geoV.longitude = mVehicleDataProvider->longitude().value();
    auto xyPos = mVehicleController->getTraCI().get()->convert2D(geoV);

    tempStr = jw.getSimpleTag("myStationId",std::to_string(mVehicleDataProvider->getStationId()),true);
    jw.addTagToElement("perceived list", tempStr);
    
    tempStr = jw.getSimpleTag("myPseudonym",std::to_string(myPseudonym),true);
    jw.addTagToElement("perceived list", tempStr);

    tempStr = jw.getSimpleTag("PseudonymChangeStrategy",pseudoChangeTypes::PseudoChangeNames[CPparams.PC_TYPE],false);
    jw.addTagToElement("perceived list", tempStr); 

    tempStr = jw.getSimpleTag("Xegosumoposition",std::to_string(xyPos.x),true);
    jw.addTagToElement("perceived list", tempStr);
    
    tempStr = jw.getSimpleTag("Yegopsumoposition",std::to_string(xyPos.y),true);
    jw.addTagToElement("perceived list", tempStr);

    tempStr = jw.getSimpleTag("longitude",std::to_string(mVehicleDataProvider->longitude().value()),true);
    jw.addTagToElement("perceived list", tempStr);
    
    tempStr = jw.getSimpleTag("latitude",std::to_string(mVehicleDataProvider->latitude().value()),true);
    jw.addTagToElement("perceived list", tempStr);


    tempStr = jw.getSimpleTag("longspeed",std::to_string(mVehicleDataProvider->speed().value()),true);
    jw.addTagToElement("perceived list", tempStr);

    tempStr = jw.getSimpleTag("longacceleration",std::to_string(mVehicleDataProvider->acceleration().value()),true);
    jw.addTagToElement("perceived list", tempStr);

    tempStr = jw.getSimpleTag("orientationangle",std::to_string(std::round(mVehicleDataProvider->heading().value() * 1800 / PI)),true);
    

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

            tempStr = jw.getSimpleTag("xdistance",std::to_string(vd.position().x.value() - mVehicleDataProvider->position().x.value()),true);
            jw.addTagToElement("object", tempStr);

            // Inverse the sign, position() use the OMNET reference system
            tempStr = jw.getSimpleTag("ydistance",std::to_string(mVehicleDataProvider->position().y.value() - vd.position().y.value()),true);
            jw.addTagToElement("object", tempStr);
            
            //getting vdp.position.x gives omnetpp format
            traci::TraCIGeoPosition geoVobject;
            geoVobject.latitude = vd.latitude().value();
            geoVobject.longitude = vd.longitude().value();
            auto xyPosobject = mVehicleController->getTraCI().get()->convert2D(geoVobject);

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
    }else{
        jw.addTagToElement("perceived list", tempStr);
        tempStr = jw.getSimpleTag("NumPerceivedObj", std::to_string(boost::size(objs)),true);
    } 
    jw.addTagToElement("perceived list", tempStr);
    
    jw.openJsonElementList("Extend Perception List");
    auto mExtendPerceptionList = mExtendPerceptionTable.getExtendPerceptionList();
    for(auto it = mExtendPerceptionList.begin(); it != mExtendPerceptionList.end(); it++){
        jw.openJsonElement("ExtendPerceptionObj",true);
        
        tempStr = jw.getSimpleTag("LastUpdateTime",std::to_string((*it).LastUpdateTime),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("speedValue",std::to_string((*it).ObjectInfo.velocity->choice.polarVelocity.velocityMagnitude.speedValue),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("PerceivedObjectID",std::to_string((*it).PerceivedObjectID),true);
        jw.addFinalTagToElement("ExtendPerceptionObj", tempStr);

        if(it == std::prev(mExtendPerceptionList.end())){
            jw.addFinalTagToElement("Extend Perception List",jw.getJsonElement("ExtendPerceptionObj"));
        }else{
            jw.addTagToElement("Extend Perception List",jw.getJsonElement("ExtendPerceptionObj"));
        }
    }

    jw.addFinalTagToElement("perceived list", jw.getJsonElementList("Extend Perception List"));
    return jw.getJsonElement("perceived list");  
} 



std::string F2MD_CPService::printextendedperceivedlisttoJson (ExtendPerceptionTable mExtendPerceptionTable) {
       
    std::string tempStr ="";
    std::string tempStrObj ="";
    JsonWriter jw;

    jw.openJsonElement("Extend Perception List", true);

    tempStr = jw.getSimpleTag("timestamp",SIMTIME_STR(simTime()),true);
    jw.addTagToElement("Extend Perception List", tempStr);

    //getting vdp.position.x gives omnetpp format
    traci::TraCIGeoPosition geoV;
    geoV.latitude = mVehicleDataProvider->latitude().value();
    geoV.longitude = mVehicleDataProvider->longitude().value();
    auto xyPos = mVehicleController->getTraCI().get()->convert2D(geoV);

    tempStr = jw.getSimpleTag("myStationId",std::to_string(mVehicleDataProvider->getStationId()),true);
    jw.addTagToElement("Extend Perception List", tempStr);
    
    tempStr = jw.getSimpleTag("mySumoId",mVehicleController->getVehicleId(),false);
    jw.addTagToElement("Extend Perception List", tempStr);

    tempStr = jw.getSimpleTag("Xegosumoposition",std::to_string(xyPos.x),true);
    jw.addTagToElement("Extend Perception List", tempStr);
    
    tempStr = jw.getSimpleTag("Yegopsumoposition",std::to_string(xyPos.y),true);
    jw.addTagToElement("Extend Perception List", tempStr);

    tempStr = jw.getSimpleTag("longitude",std::to_string(mVehicleDataProvider->longitude().value()),true);
    jw.addTagToElement("Extend Perception List", tempStr);
    
    tempStr = jw.getSimpleTag("latitude",std::to_string(mVehicleDataProvider->latitude().value()),true);
    jw.addTagToElement("Extend Perception List", tempStr);

    tempStr = jw.getSimpleTag("longspeed",std::to_string(mVehicleDataProvider->speed().value()),true);
    jw.addTagToElement("Extend Perception List", tempStr);

    tempStr = jw.getSimpleTag("longacceleration",std::to_string(mVehicleDataProvider->acceleration().value()),true);
    jw.addTagToElement("Extend Perception List", tempStr);

    tempStr = jw.getSimpleTag("orientationangle",std::to_string(std::round(mVehicleDataProvider->heading().value() * 1800 / PI)),true);
    jw.addTagToElement("Extend Perception List", tempStr);

    
    jw.openJsonElementList("ExtendedPerceivedObjects");
    auto mExtendPerceptionList = mExtendPerceptionTable.getExtendPerceptionList();
    for(auto it = mExtendPerceptionList.begin(); it != mExtendPerceptionList.end(); ++it){
        jw.openJsonElement("ExtendPerceptionObj",true);
        
        tempStr = jw.getSimpleTag("PerceivedObjectID",std::to_string((*it).PerceivedObjectID),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("SumoPerceivedObjectID",Artery2SumoID.find((*it).PerceivedObjectID)->second,false);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        //std::cout << "SUMO ID in JSON"<<Artery2SumoID.find((*it).PerceivedObjectID)->second << std::endl;
        tempStr = jw.getSimpleTag("LastUpdateTime",std::to_string((*it).LastUpdateTime),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("Observer",std::to_string((*it).ObserverID),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        traci::TraCIGeoPosition geoVobject;
            geoVobject.latitude = (*it).ReferencePosition.latitude / 1e7;
            geoVobject.longitude = (*it).ReferencePosition.longitude / 1e7;
        auto xyPosobject = mVehicleController->getTraCI().get()->convert2D(geoVobject);

        // SUMO X position exact 
        tempStr = jw.getSimpleTag("Xcoordinate",std::to_string(xyPosobject.x + (*it).ObjectInfo.position.xCoordinate.value / 1e2),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);
        // SUMO Y position exact 
        tempStr = jw.getSimpleTag("Ycoordinate",std::to_string(xyPosobject.y + (*it).ObjectInfo.position.xCoordinate.value / 1e2),true);
        jw.addTagToElement("ExtendPerceptionObj", tempStr);

        auto velocity = (*it).ObjectInfo.velocity->choice.polarVelocity.velocityMagnitude.speedValue /100;
        tempStr = jw.getSimpleTag("Velocity",std::to_string(velocity),true);
        jw.addFinalTagToElement("ExtendPerceptionObj", tempStr);

        // cout << "mySumoId::"<< mVehicleController->getVehicleId() <<endl;
        // cout << "ObserverSumoID::"<< Artery2SumoID.find((*it).ObserverID)->second <<endl;
        // cout << "SumoPerceivedObjectID::"<< Artery2SumoID.find((*it).PerceivedObjectID)->second <<endl;
        // cout << "speedvalue::"<< velocity <<endl;


        /*tempStr = jw.getSimpleTag("xAngle",std::to_string((*it).ObjectInfo.angles->xAngle->value),true);
        jw.addFinalTagToElement("ExtendPerceptionObj", tempStr);

        tempStr = jw.getSimpleTag("yAngle",std::to_string((*it).ObjectInfo.angles->yAngle->value),true);
        jw.addFinalTagToElement("ExtendPerceptionObj", tempStr);*/

        if(it == std::prev(mExtendPerceptionList.end())){
            jw.addFinalTagToElement("ExtendedPerceivedObjects",jw.getJsonElement("ExtendPerceptionObj"));
        }else{
            jw.addTagToElement("ExtendedPerceivedObjects",jw.getJsonElement("ExtendPerceptionObj"));
        }
    }
    jw.addFinalTagToElement("Extend Perception List",jw.getJsonElementList("ExtendedPerceivedObjects"));

    //jw.addFinalTagToElement("perceived list", jw.getJsonElementList("Extend Perception List"));
     
    return jw.getJsonElement("Extend Perception List");  
} 



int F2MD_CPService::RXSocketCpmToPython(int port,const vanetza::asn1::Cpm& cpm, const VehicleDataProvider* mVehicleDataProvider)
{
    struct sockaddr_in serv_addr;
    if (mypythonsocketRX==0)
	{	
		if ((mypythonsocketRX = socket(AF_INET, SOCK_STREAM, 0)) < 0)
		{
			printf("\n Socket creation error \n");
			std::cout<<"\n Socket creation error \n";
			return -1;
		}
		
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(port);
		
		// Convert IPv4 and IPv6 addresses from text to binary form
		if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
		{
			printf("\nInvalid address/ Address not supported \n");

			std::cout<<"\n Invalid address/ Address not supported \n";
			return -1;
		}
	
		if (connect(mypythonsocketRX, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
		{
			printf("\nConnection Failed \n");
			std::cout<<"\n Connection Failed \n";
			close(mypythonsocketRX);
			mypythonsocketRX=0;
			return -1;
		}
	}

	// sending cpm message to carla python socket server
	std::stringstream ss, s1, s2;
    ss << "sender_artery_id:" << (*cpm).header.stationId << "\n";
    ss << "sender_sumo_id:" << Artery2SumoID.find((*cpm).header.stationId)->second << "\n";
    
    TraCIGeoPosition senderPos,receiverPos;
    
    senderPos.longitude = (*cpm).payload.managementContainer.referencePosition.longitude / 1e7;
    senderPos.latitude = (*cpm).payload.managementContainer.referencePosition.latitude / 1e7;

    receiverPos.longitude = mVehicleDataProvider->longitude().value();
    receiverPos.latitude = mVehicleDataProvider->latitude().value();
    
    ss << "sender_long:" << senderPos.longitude << "\n";
    ss << "sender_lat:" << senderPos.latitude << "\n";
    for(int i=0; i < (*cpm).payload.cpmContainers.list.count; i++){
        if((*cpm).payload.cpmContainers.list.array[i]->containerId == CpmContainerId_originatingVehicleContainer){
                auto sender_yaw = (*cpm).payload.cpmContainers.list.array[i]->containerData.choice.OriginatingVehicleContainer.orientationAngle.value;
                sender_yaw /= 10; 
                ss << "sender_yaw:" << sender_yaw << "\n";
        }
    }   
    // cout << "sender_long:" << (*cpm).payload.managementContainer.referencePosition.longitude / 1e7 << endl;
    // cout << "sender_lat:" << (*cpm).payload.managementContainer.referencePosition.latitude / 1e7 << endl;

	ss << "receiver_artery_id:" << mVehicleDataProvider->station_id() << "\n";
	ss << "receiver_sumo_id:" << mVehicleController->getVehicleId() << "\n";
	ss << "receiver_long:" << std::round(mVehicleDataProvider->longitude().value()*1e7)/1e7<< "\n";
	ss << "receiver_lat:" << std::round(mVehicleDataProvider->latitude().value()*1e7)/1e7 << "\n";
	ss << "receiver_speed:" << mVehicleDataProvider->speed().value() << "\n";
    ss << "receiver_yaw:" << angle_cast(mVehicleController->getHeading()).degree << "\n";

    ss << "Vehicle_Length:" << CPparams.VEHICLE_LENGTH << "\n";
    ss << "Vehicle_Width:" << CPparams.VEHICLE_WIDTH << "\n";
    ss << "receiver_type:" << mF2MDFacility.getMbType() << "\n";

    std::string myString = ss.str();
	char nstr[8]; 
	int n= myString.length();
	snprintf(nstr, 7,"%06d", n);
    
	::send(mypythonsocketRX , nstr , 6 , 0 );
    ::send(mypythonsocketRX , myString.c_str() , n , 0 );
	return 0;
}


int F2MD_CPService::TXSocketCpmToPython(int port, const traci::VehicleController* mVehicleController)
{
    struct sockaddr_in serv_addr;
    if (mypythonsocketTX==0)
	{	
		if ((mypythonsocketTX = socket(AF_INET, SOCK_STREAM, 0)) < 0)
		{
			printf("\n Socket creation error \n");
			std::cout<<"\n Socket creation error \n";
			return -1;
		}
		
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(port);
		
		// Convert IPv4 and IPv6 addresses from text to binary form
		if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
		{
			printf("\nInvalid address/ Address not supported \n");

			std::cout<<"\n Invalid address/ Address not supported \n";
			return -1;
		}
	
		if (connect(mypythonsocketTX, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
		{
			printf("\nConnection Failed \n");
			// std::cout<<"\n Connection Failed \n";
			close(mypythonsocketTX);
			mypythonsocketTX=0;
			return -1;
		}
	}

    std::stringstream ss, s1, s2, s3, s4;
    TraCIGeoPosition senderPos;
    senderPos.latitude = std::round(mVehicleController->getGeoPosition().latitude.value() *1e7) /1e7;
    senderPos.longitude = std::round(mVehicleController->getGeoPosition().longitude.value() *1e7) /1e7;
    TraCIPosition senderXY;
    senderXY = mTraci.get()->convert2D(senderPos);

    ss << "sender_type:" << mF2MDFacility.getMbType() << "\n";
	ss << "sender_artery_id:" << mVehicleDataProvider->station_id() << "\n";
	ss << "sender_sumo_id:" << mVehicleController->getVehicleId() << "\n";
    ss << "Vehicle_Length:" << CPparams.VEHICLE_LENGTH << "\n";
    ss << "Vehicle_Width:" << CPparams.VEHICLE_WIDTH << "\n";
    ss << "attack_type:" << mF2MDFacility.getAttackType() <<  "\n";

    try{
        if (mF2MDFacility.getMbType() == mbTypes::LocalAttacker || mF2MDFacility.getMbType() == mbTypes::LocalAttacker_detected){
            if(mF2MDFacility.getAttackType() == cpAttackTypes::AddObj){
                vector<AttackedObjInfo> objInfo = mF2MDFacility.getAttackedObjInfo();
                ss << "attacked_obj_sumo_ids:";
                for(const auto& i:objInfo){
                    std::string obj_sumo_id = "ghost";
                    ss << obj_sumo_id << ",";
                }
                ss << "\n";

                s1 << "attacked_obj_long:";
                s2 << "attacked_obj_lat:";

                for(const auto& i:objInfo){
                    auto objX = senderXY.x + i.PosX / 100;
                    auto objY = senderXY.y + i.PosY / 100;
                    TraCIPosition objP;
                    objP.x = objX;
                    objP.y = objY;
                    // cout << "ghost objX::" << objX << endl;
                    // cout << "ghost objY::" << objY << endl;
                    // cout << "sender objX::" << senderXY.x << endl;
                    // cout << "sender objY::" << senderXY.y << endl;

                    TraCIGeoPosition objGeo = mTraci.get()->convertGeo(objP);
                    s1 << std::round(objGeo.longitude *1e7) /1e7 << ",";
                    s2 << std::round(objGeo.latitude *1e7) /1e7 << ",";
                }
                s1 << "\n";
                s2 << "\n";
                ss << s1.str() << s2.str();

                ss << "attacked_obj_yaw:";
                for(const auto& i:objInfo){
                    auto yaw = ((900 - int(i.Yaw))+3600) % 3600;
                    ss << yaw/10 << ",";
                }
                ss << "\n"; 
            }else{
                vector<AttackedObjInfo> objInfo = mF2MDFacility.getAttackedObjInfo();
                ss << "attacked_obj_sumo_ids:";
                for(const auto& i:objInfo){
                    auto obj_sumo_id = Artery2SumoID.find(i.id)->second;
                    ss << obj_sumo_id << ",";
                }
                ss << "\n";

                s1 << "attacked_obj_long:";
                s2 << "attacked_obj_lat:";
                s3 << "attacked_obj_real_long:";
                s4 << "attacked_obj_real_lat:";

                for(const auto& i:objInfo){
                    auto objX = senderXY.x + i.PosX / 100;
                    auto objY = senderXY.y + i.PosY / 100;
                    TraCIPosition objP, obj_real_pos;
                    objP.x = objX;
                    objP.y = objY;
                    TraCIGeoPosition objGeo = mTraci.get()->convertGeo(objP);

                    TraCIGeoPosition real_objGeo;
                    auto obj_sumo_id = Artery2SumoID.find(i.id)->second;
                    obj_real_pos = mTraci.get()->vehicle.getPosition(obj_sumo_id);
                    real_objGeo = mTraci.get()->convertGeo(obj_real_pos);
                                    
                    // cout << "objX::" << objGeo.longitude << endl;
                    // cout << "objY::" << objGeo.latitude << endl;
                    // cout << "real_objGeolong::" << real_objGeo.longitude << endl;
                    // cout << "real_objGeolat::" << real_objGeo.latitude << endl;
            
                    s1 << std::round(objGeo.longitude *1e7) /1e7 << ",";
                    s2 << std::round(objGeo.latitude *1e7) /1e7 << ",";
                    s3 << std::round(real_objGeo.longitude *1e7) /1e7 << ",";
                    s4 << std::round(real_objGeo.latitude *1e7) /1e7 << ",";
                }
                s1 << "\n";
                s2 << "\n";
                s3 << "\n";
                s4 << "\n";
                ss << s1.str() << s2.str() << s3.str() << s4.str();

                ss << "attacked_obj_yaw:";
                for(const auto& i:objInfo){
                    auto yaw = ((900 - int(i.Yaw))+3600) % 3600;
                    ss << yaw/10 << ",";
                }
                ss << "\n"; 
            }  
        }
    }catch(exception e){
    }


    std::string myString = ss.str();
	char nstr[8]; 
	int n= myString.length();
	snprintf(nstr, 7,"%06d", n);
    
	::send(mypythonsocketTX , nstr , 6 , 0 );
    ::send(mypythonsocketTX , myString.c_str() , n , 0 );
	return 0;
}



int F2MD_CPService::EXSocketCpmToPython(int port,std::string string)
{
    struct sockaddr_in serv_addr;
    if (mypythonsocketEX==0)
	{	
		if ((mypythonsocketEX = socket(AF_INET, SOCK_STREAM, 0)) < 0)
		{
			printf("\n Socket creation error \n");
			std::cout<<"\n Socket creation error \n";
			return -1;
		}
		
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(port);
		
		// Convert IPv4 and IPv6 addresses from text to binary form
		if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
		{
			printf("\nInvalid address/ Address not supported \n");

			std::cout<<"\n Invalid address/ Address not supported \n";
			return -1;
		}
	
		if (connect(mypythonsocketEX, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
		{
			printf("\nConnection Failed \n");
			std::cout<<"\n Connection Failed \n";
			close(mypythonsocketEX);
			mypythonsocketEX=0;
			return -1;
		}
	}

	// sending cpm message to carla python socket server
    std::string myString = string;
	char nstr[8]; 
	int n= myString.length();
	snprintf(nstr, 7,"%06d", n);
    
	::send(mypythonsocketEX , nstr , 6 , 0 );
    ::send(mypythonsocketEX , myString.c_str() , n , 0 );
	return 0;
}

} // namespace artery


