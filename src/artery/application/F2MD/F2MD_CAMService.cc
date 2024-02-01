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

#include "F2MD_CAMService.h"

using namespace omnetpp;
using namespace vanetza;

namespace artery{

Define_Module(F2MD_CAMService)

/**
   *  @brief Initialise CAM Service and CAM Facility for further F2MD Processing
*/
void F2MD_CAMService::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();

    beaconInterval = par("beaconInterval");

    initParams();

    // ------ Node Module Acces and Mac Address ------ Start
    node = inet::findContainingNode(this);
    myId = inet::getModuleFromPar<inet::ieee80211::Ieee80211Mac>(par("pathToMacModule"),node)->getAddress();
    // ------ Node Module Acces and Mac Address ------ End

    // ------ Initialization of F2MD Module ------ Start
    mF2MDFacility = F2MD_CAMFacility(mVehicleController, node, myId, beaconInterval, CAMparams);
    mF2MDFacility.initialize();
    // ------ Initialization of F2MD Module ------ End

    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;
}

/**
   *  @brief The Middleware triggers this service with a time period dedined in omnetpp.ini. Depending on a time condition, a new CAM will be sent or not.
*/
void F2MD_CAMService::trigger()
{
    Enter_Method("F2MD_CAMService trigger");

    if (simTime()-lastBeacon >= beaconInterval){

        // Update the vehicle datas in CAM F2MD Facility, so that the CAM contains datas that are up to date.
        mF2MDFacility.updateVehicleState();

        // Handle basic F2MD tasks (updates colors of vehicles in the simulation)
        mF2MDFacility.treatAttackFlags();
        
        auto packet = mF2MDFacility.createBSM();

        btp::DataRequestB req;
        req.destination_port = host_cast<F2MD_CAMService::port_type>(getPortNumber());
        req.gn.transport_type = geonet::TransportType::SHB;
        req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP1));
        req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

        // Send the packet to lower layers
        request(req, packet);

        lastBeacon = simTime();
    }
}

/**
   *  @brief this method is called when this node receives a BSM
*/
void F2MD_CAMService::indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket* packet)
{
    Enter_Method("F2MD_CAMService indicate");

    // Update the vehicle datas in CAM_Service.h
    mF2MDFacility.updateVehicleState();

	auto BSMrecvd = check_and_cast<veins::BSM*>(packet);
    BSMrecvd->setArrivalTime(simTime());

    // Display important fields contained in the received BSM, to check the reception (Useful for debugging)
    TestReception(BSMrecvd);
	
    // Pass the received BSM to the F2MD Processing facility for further analysis (MbD checks,...)
    mF2MDFacility.onBSM(BSMrecvd);

    delete BSMrecvd;
}

/**
   *  @brief  displays in QTenv the message received. Useful when debugging.
   *  @param  veins::BSM* BSM received.
*/
void F2MD_CAMService::TestReception(veins::BSM* BSMrecvd){

    veins::Coord senderPos = BSMrecvd->getSenderPos();
    veins::Coord senderSpeed = BSMrecvd->getSenderSpeed();
    veins::Coord senderAccel = BSMrecvd->getSenderAccel();
    veins::Coord senderHeading = BSMrecvd->getSenderHeading();
    double senderLength = BSMrecvd->getSenderLength();
    double senderWidth = BSMrecvd->getSenderWidth();
    inet::MACAddress senderAddress = BSMrecvd->getSenderAddress();
    veins::Coord senderGPSCoordinates = BSMrecvd->getSenderGpsCoordinates();
    int senderMbType = BSMrecvd->getSenderMbType();
    int senderAttackType = BSMrecvd->getSenderAttackType();

    const std::string id = mVehicleController->getVehicleId();

    EV_INFO << "------------- DATA received through BSM ----------------" << endl;
    EV_INFO << " Node " << id << "received from node " << BSMrecvd->getSenderAddress() << " who is an " << BSMrecvd->getSenderMbType() << endl;

    EV_INFO << "senderposX : " << senderPos.x << endl;
    EV_INFO << "senderposY : " << senderPos.y << endl;
    EV_INFO << "senderposZ : " << senderPos.z << endl;
    EV_INFO << "GPScoord : " << senderGPSCoordinates.x << senderGPSCoordinates.y << endl;
    EV_INFO << "SpeedX : " << senderSpeed.x << endl;
    EV_INFO << "SpeedX : " << senderSpeed.y << endl;
    EV_INFO << "Angle : " << senderHeading << endl;
    EV_INFO << "senderLength : " << senderLength << endl;
    EV_INFO << "senderWidth : " << senderWidth << endl;
    EV_INFO << "senderAddress : " << senderAddress << endl;
    EV_INFO << "senderMbType : " << senderMbType << endl;
    EV_INFO << "senderAttackType : " << senderAttackType << endl;
    EV_INFO << "------------- DATA received through BSM ----------------" << endl;

}

void F2MD_CAMService::finish(){

    recordScalar("generatedBSMs", mF2MDFacility.generatedBSMs);
    recordScalar("receivedBSMs", mF2MDFacility.receivedBSMs);

    mF2MDFacility.handleReportProtocol(true);
}

/**
   *  @brief  stores the parameters defined in omnetpp.ini in the F2MD_CAMParameters.h file so that they can be used in the simulation.
*/
void F2MD_CAMService::initParams(){

    // ------ Parameters Initialisation ------ Start
    CAMparams.serialNumber = par("serialNumber").stdstringValue();
    CAMparams.savePath = par("savePath").stdstringValue();

    CAMparams.veremiConf = par("veremiConf");
    CAMparams.randomConf = par("randomConf");
    CAMparams.variableConf = par("variableConf");
    CAMparams.confPos = par("confPos");
    CAMparams.confSpd = par("confSpd");
    CAMparams.confHea = par("confHea");
    CAMparams.confAcc = par("confAcc");
    CAMparams.confPrec = par("confPrec");
    CAMparams.minConf = par("minConf");

    CAMparams.SAVE_PERIOD = par("SAVE_PERIOD");
    CAMparams.PRINT_PERIOD = par("PRINT_PERIOD");
    CAMparams.START_SAVE = par("START_SAVE");
    CAMparams.START_ATTACK = par("START_ATTACK");

    CAMparams.START_SAVE = par("START_SAVE");
    CAMparams.START_ATTACK = par("START_ATTACK");

    CAMparams.REPORT_VERSION = reportTypes::intReport[par("REPORT_VERSION").intValue()];
    CAMparams.UseAttacksServer = par("UseAttacksServer");
    CAMparams.MixLocalAttacks = par("MixLocalAttacks");
    CAMparams.RandomLocalMix = par("RandomLocalMix");

    CAMparams.LOCAL_ATTACKER_PROB = par("LOCAL_ATTACKER_PROB");
    CAMparams.LOCAL_ATTACK_TYPE = attackTypes::intAttacks[par("LOCAL_ATTACK_TYPE").intValue()];
    CAMparams.GLOBAL_ATTACKER_PROB = par("GLOBAL_ATTACKER_PROB");
    CAMparams.GLOBAL_ATTACK_TYPE = attackTypes::intAttacks[par("GLOBAL_ATTACK_TYPE")];

    CAMparams.EnablePC = par("EnablePC");
    CAMparams.PC_TYPE = pseudoChangeTypes::intPseudoChange[par("PC_TYPE").intValue()];
    CAMparams.EnableV1 = par("EnableV1");
    CAMparams.EnableV2 = par("EnableV2");
    CAMparams.SaveStatsV1 = par("SaveStatsV1");
    CAMparams.SaveStatsV2 = par("SaveStatsV2");
    CAMparams.checksVersionV1 = mdChecksVersionTypes::intChecksVersion[par("checksVersionV1").intValue()];
    CAMparams.checksVersionV2 = mdChecksVersionTypes::intChecksVersion[par("checksVersionV2").intValue()];

    CAMparams.appTypeV1 = mdAppTypes::intApp[par("appTypeV1").intValue()];
    CAMparams.appTypeV2 = mdAppTypes::intApp[par("appTypeV2").intValue()];

    CAMparams.writeSelfMsg = par("writeSelfMsg");
    CAMparams.writeListSelfMsg = par("writeListSelfMsg");

    CAMparams.writeBsmsV1 = par("writeBsmsV1");
    CAMparams.writeBsmsV2 = par("writeBsmsV2");
    CAMparams.writeListBsmsV1 = par("writeListBsmsV1");
    CAMparams.writeListBsmsV2 = par("writeListBsmsV2");

    CAMparams.writeReportsV1 = par("writeReportsV1");
    CAMparams.writeReportsV2 = par("writeReportsV2");
    CAMparams.writeListReportsV1 = par("writeListReportsV1");
    CAMparams.writeListReportsV2 = par("writeListReportsV2");
    CAMparams.writeVeReMi = par("writeVeReMi");
    CAMparams.VeReMiSliceTime = par("VeReMiSliceTime");

    CAMparams.sendReportsV1 = par("sendReportsV1");
    CAMparams.sendReportsV2 = par("sendReportsV2");
    CAMparams.maHostV1 = par("maHostV1").stdstringValue();
    CAMparams.maHostV2 = par("maHostV2").stdstringValue();
    CAMparams.maPortV1 = par("maPortV1");
    CAMparams.maPortV2 = par("maPortV2");
    CAMparams.enableVarThreV1 = par("enableVarThreV1");
    CAMparams.enableVarThreV2 = par("enableVarThreV2");
    // ------ Parameters Initialisation ------ End


    // ------ Detection Parameters ------ Start
    CAMparams.MAX_PROXIMITY_RANGE_L = par("MAX_PROXIMITY_RANGE_L");
    CAMparams.MAX_PROXIMITY_RANGE_W = par("MAX_PROXIMITY_RANGE_W");
    CAMparams.MAX_PROXIMITY_DISTANCE = par("MAX_PROXIMITY_DISTANCE");
    CAMparams.MAX_CONFIDENCE_RANGE = par("MAX_CONFIDENCE_RANGE");
    CAMparams.MAX_PLAUSIBLE_RANGE = par("MAX_PLAUSIBLE_RANGE");
    CAMparams.MAX_TIME_DELTA = par("MAX_TIME_DELTA");
    CAMparams.MAX_DELTA_INTER = par("MAX_DELTA_INTER");
    CAMparams.MAX_SA_RANGE = par("MAX_SA_RANGE");
    CAMparams.MAX_SA_TIME = par("MAX_SA_TIME");
    CAMparams.MAX_KALMAN_TIME = par("MAX_KALMAN_TIME");
    CAMparams.KALMAN_POS_RANGE = par("KALMAN_POS_RANGE");
    CAMparams.KALMAN_SPEED_RANGE = par("KALMAN_SPEED_RANGE");
    CAMparams.KALMAN_MIN_POS_RANGE = par("KALMAN_MIN_POS_RANGE");
    CAMparams.KALMAN_MIN_SPEED_RANGE = par("KALMAN_MIN_SPEED_RANGE");
    CAMparams.MIN_MAX_SPEED = par("MIN_MAX_SPEED");
    CAMparams.MIN_MAX_ACCEL = par("MIN_MAX_ACCEL");
    CAMparams.MIN_MAX_DECEL = par("MIN_MAX_DECEL");
    CAMparams.MAX_MGT_RNG = par("MAX_MGT_RNG");
    CAMparams.MAX_MGT_RNG_DOWN = par("MAX_MGT_RNG_DOWN");
    CAMparams.MAX_MGT_RNG_UP = par("MAX_MGT_RNG_UP");
    CAMparams.MAX_BEACON_FREQUENCY = par("MAX_BEACON_FREQUENCY");
    CAMparams.MAX_DISTANCE_FROM_ROUTE = par("MAX_DISTANCE_FROM_ROUTE");
    CAMparams.MAX_NON_ROUTE_SPEED = par("MAX_NON_ROUTE_SPEED");
    CAMparams.MAX_HEADING_CHANGE = par("MAX_HEADING_CHANGE");
    CAMparams.DELTA_BSM_TIME = par("DELTA_BSM_TIME");
    CAMparams.DELTA_REPORT_TIME = par("DELTA_REPORT_TIME");
    CAMparams.POS_HEADING_TIME = par("POS_HEADING_TIME");
    // ------ Detection Parameters ------ Start

    // ------ Storage Parameters ------ Start
    CAMparams.MAX_TARGET_TIME = par("MAX_TARGET_TIME");
    CAMparams.MAX_ACCUSED_TIME = par("MAX_ACCUSED_TIME");
    // ------ Storage Parameters ------ End

    // ------ Attacks Parameters ------ Start
    CAMparams.parVar = par("parVar");
    CAMparams.RandomPosOffsetX = par("RandomPosOffsetX");
    CAMparams.RandomPosOffsetY = par("RandomPosOffsetY");
    CAMparams.RandomSpeedX = par("RandomSpeedX");
    CAMparams.RandomSpeedY = par("RandomSpeedY");
    CAMparams.RandomSpeedOffsetX = par("RandomSpeedOffsetX");
    CAMparams.RandomSpeedOffsetY = par("RandomSpeedOffsetY");
    CAMparams.RandomAccelX = par("RandomAccelX");
    CAMparams.RandomAccelY = par("RandomAccelY");
    CAMparams.StopProb = par("StopProb");
    CAMparams.StaleMessages_Buffer = par("StaleMessages_Buffer");
    CAMparams.DosMultipleFreq = par("DosMultipleFreq");
    CAMparams.DosMultipleFreqSybil = par("DosMultipleFreqSybil");
    CAMparams.ReplaySeqNum = par("ReplaySeqNum");
    CAMparams.SybilVehNumber = par("SybilVehNumber");
    CAMparams.SelfSybil = par("SelfSybil");
    CAMparams.SybilDistanceX = par("SybilDistanceX");
    CAMparams.SybilDistanceY = par("SybilDistanceY");
    // ------ Attacks Parameters ------ End

    // ------ Pseudonym Parameters ------ Start
    CAMparams.Period_Change_Time = par("Period_Change_Time");
    CAMparams.Tolerance_Buffer = par("Tolerance_Buffer");
    CAMparams.Period_Change_Distance = par("Period_Change_Distance");
    CAMparams.Random_Change_Chance = par("Random_Change_Chance");
    // ------ Pseudonym Parameters ------ End

    //------ Report Parameters ------ Start
    CAMparams.InitialHistory = par("InitialHistory");
    CAMparams.CollectionPeriod = par("CollectionPeriod");
    CAMparams.UntolerancePeriod = par("UntolerancePeriod");
    //------ Report Parameters ------ End

}

}
