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

#include "F2MD_CAMFacility.h"

using namespace omnetpp;
using namespace vanetza;

/**
   *  @brief  Creates a CAM Processing Facility module, with the given parameters
*/
F2MD_CAMFacility::F2MD_CAMFacility(const traci::VehicleController* VehController, cModule* containingNode, inet::MACAddress address, simtime_t CAMInterval, F2MD_CAMParameters& CAMparams){
    mVehicleController = VehController;
    node = containingNode;
    myId = address;
    beaconInterval = CAMInterval;
    params = &CAMparams;
}

/**
   *  @brief  Initialise the CAM Processing Facility module, with the given parameters
*/
void F2MD_CAMFacility::initialize()
{

    // Initialise Link Control
    linkControl.initialize(params, mVehicleController);
    linkInit = true;

//----------------------------------------- Stage 0 ----------------------------------------------

    beaconInterval = 1;

    // ------ Initialise Vehicle info ------
    updateVehicleState();
    
    curPositionConfidenceOrig = Coord(0, 0, 0);
    curSpeedConfidenceOrig = Coord(0, 0, 0);
    curHeadingConfidenceOrig = Coord(0, 0, 0);
    curAccelConfidenceOrig = Coord(0, 0, 0);

    curPositionConfidence = Coord(0, 0, 0);
    curSpeedConfidence = Coord(0, 0, 0);
    curHeadingConfidence = Coord(0, 0, 0);
    curAccelConfidence = Coord(0, 0, 0);

    myMdType = mbTypes::Genuine;
    myAttackType = attackTypes::Attacks::Genuine;

    reportProtocolEnforcerV1.setParams(params);
    reportProtocolEnforcerV2.setParams(params);

    //mkdir savePath
    struct stat info;
    if ((stat(params->savePath.c_str(), &info) != 0) || !(info.st_mode & S_IFDIR)) {
        mkdir(params->savePath.c_str(), 0777);
    }

    generatedBSMs = 0;
    receivedBSMs = 0;

//----------------------------------------- Stage 1 ----------------------------------------------

    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;

    // ------ Initialise World Info ------
    auto NetBoundary = mVehicleController->getTraCI()->simulation.getNetBoundary();
    ASSERT(NetBoundary.value.size() == 2);

    if (NetBoundary.value.size() == 2){
        MaxRandomPosX = std::max(NetBoundary.value.front().x,NetBoundary.value.back().x);
        MaxRandomPosY = std::max(NetBoundary.value.front().y,NetBoundary.value.back().y);
        MinRandomPosX = std::min(NetBoundary.value.front().x,NetBoundary.value.back().x);
        MinRandomPosY = std::min(NetBoundary.value.front().y,NetBoundary.value.back().y);
    }

    MAX_PLAUSIBLE_ACCEL = mVehicleController->getTraCI()->vehicle.getAccel(id) + 0.1;
    MAX_PLAUSIBLE_DECEL = mVehicleController->getTraCI()->vehicle.getDecel(id) + 0.1;
    MAX_PLAUSIBLE_SPEED = mVehicleController->getTraCI()->vehicle.getMaxSpeed(id) + 0.1;

    // ------ Initialise F2MD  ------

    // Depending on the Attacking Probability, the node will be an attacker or a genuine
    myMdType = induceMisbehavior(params->LOCAL_ATTACKER_PROB, params->GLOBAL_ATTACKER_PROB);
    setMDApp(params->appTypeV1, params->appTypeV2);

    if (params->UseAttacksServer) {
        localAttackServer.pingAttackServer();

        if (localAttackServer.isQueuedAttack()) {
            if (localAttackServer.isInstantAttack()) {
                if (myMdType == mbTypes::Genuine) {
                    myMdType = mbTypes::LocalAttacker;
                }
            }
        }
        else {
            if (myMdType == mbTypes::LocalAttacker) {
                myMdType = mbTypes::Genuine;
            }
        }
    }

    myPcType = params->PC_TYPE;
    pseudoNum = 0;

    auto posInit = vehicle_api.getPosition(id);
    auto coordPosInit = veins::Coord(posInit.x,posInit.y,posInit.z);

    // ------ Initialise Pseudonym Change Policy  ------
    pcPolicy = PCPolicy(coordPosInit, params);

    pcPolicy.setMbType(myMdType);
    pcPolicy.setMdAuthority(&mdStats);
    pcPolicy.setCurPosition(&coordPosInit);
    pcPolicy.setMyId(&myId);
    pcPolicy.setMyPseudonym(&myPseudonym);
    pcPolicy.setPseudoNum(&pseudoNum);

    myPseudonym = pcPolicy.getNextPseudonym();
    
    
    // ------ Initialise RandomConf  ------

    if (params->randomConf) {
        double randConfPos = genLib.RandomDouble(
            params->confPos * params->minConf, params->confPos);
        double randConfSpeed = genLib.RandomDouble(
            params->confSpd * params->minConf, params->confSpd);
        double randConfHeading = genLib.RandomDouble(
            params->confHea * params->minConf, params->confHea);
        double randConfAccel = genLib.RandomDouble(
            params->confAcc * params->minConf, params->confAcc);

        if (params->variableConf) {
            ConfPosMax = Coord(randConfPos * params->minConf, randConfPos * params->minConf,
                0);
            ConfSpeedMax = Coord(randConfSpeed * params->minConf,
                randConfSpeed * params->minConf, 0);
            ConfHeadMax = Coord(randConfHeading * params->minConf,
                randConfHeading * params->minConf, 0);
            ConfAccelMax = Coord(randConfAccel * params->minConf,
                randConfAccel * params->minConf, 0);
        }
        else {
            ConfPosMax = Coord(0.0, 0.0, 0.0);
            ConfSpeedMax = Coord(0.0, 0.0, 0.0);
            ConfHeadMax = Coord(0.0, 0.0, 0.0);
            ConfAccelMax = Coord(0.0, 0.0, 0.0);
        }

        randConfPos = ((int) (randConfPos * params->confPrec + .5) / params->confPrec);
        randConfSpeed = ((int) (randConfSpeed * params->confPrec + .5) / params->confPrec);
        randConfHeading = ((int) (randConfHeading * params->confPrec + .5) / params->confPrec);
        randConfAccel = ((int) (randConfAccel * params->confPrec + .5) / params->confPrec);

        curPositionConfidenceOrig = Coord(randConfPos, randConfPos, 0);
        curSpeedConfidenceOrig = Coord(randConfSpeed, randConfSpeed, 0);
        curHeadingConfidenceOrig = Coord(randConfHeading, randConfHeading,
            0);
        curAccelConfidenceOrig = Coord(randConfAccel, randConfAccel, 0);

        curPositionConfidence = Coord(randConfPos, randConfPos, 0);
        curSpeedConfidence = Coord(randConfSpeed, randConfSpeed, 0);
        curHeadingConfidence = Coord(randConfHeading, randConfHeading, 0);
        curAccelConfidence = Coord(randConfAccel, randConfAccel, 0);
    }
    else if (params->veremiConf) {
        double posE0 = genLib.RandomDouble(3, 5);
        double speedE0 = genLib.GaussianRandomDouble(0, 0.0016);
        double headingE0 = genLib.RandomDouble(0, 20);

        curPositionConfidenceOrig = Coord(posE0, posE0, 0);
        curSpeedConfidenceOrig = Coord(speedE0, speedE0, 0);
        curHeadingConfidenceOrig = Coord(headingE0, headingE0, 0);
        curAccelConfidenceOrig = Coord(0, 0, 0);

        curPositionConfidence = Coord(posE0, posE0, 0);
        curSpeedConfidence = Coord(speedE0, speedE0, 0);
        curHeadingConfidence = Coord(headingE0, headingE0, 0);
        curAccelConfidence = Coord(0, 0, 0);

        lastPositionUpdate = simTime().dbl() - 1.0;
    }
    else {
        if (params->variableConf) {
            ConfPosMax = Coord(params->confPos * params->minConf, params->confPos * params->minConf, 0);
            ConfSpeedMax = Coord(params->confSpd * params->minConf, params->confSpd * params->minConf, 0);
            ConfHeadMax = Coord(params->confHea * params->minConf, params->confHea * params->minConf, 0);
            ConfAccelMax = Coord(params->confAcc * params->minConf, params->confAcc * params->minConf, 0);
        }
        else {
            ConfPosMax = Coord(0.0, 0.0, 0.0);
            ConfSpeedMax = Coord(0.0, 0.0, 0.0);
            ConfHeadMax = Coord(0.0, 0.0, 0.0);
            ConfAccelMax = Coord(0.0, 0.0, 0.0);
        }

        double ConfPosR = ((int) (params->confPos * params->confPrec + .5) / params->confPrec);
        double ConfSpeedR = ((int) (params->confSpd * params->confPrec + .5) / params->confPrec);
        double ConfHeadingR = ((int) (params->confHea * params->confPrec + .5) / params->confPrec);
        double ConfAccelR = ((int) (params->confAcc * params->confPrec + .5) / params->confPrec);

        curPositionConfidenceOrig = Coord(ConfPosR, ConfPosR, 0);
        curSpeedConfidenceOrig = Coord(ConfSpeedR, ConfSpeedR, 0);
        curHeadingConfidenceOrig = Coord(ConfHeadingR, ConfHeadingR, 0);
        curAccelConfidenceOrig = Coord(ConfAccelR, ConfAccelR, 0);

        curPositionConfidence = Coord(ConfPosR, ConfPosR, 0);
        curSpeedConfidence = Coord(ConfSpeedR, ConfSpeedR, 0);
        curHeadingConfidence = Coord(ConfHeadingR, ConfHeadingR, 0);
        curAccelConfidence = Coord(ConfAccelR, ConfAccelR, 0);
    }

    myReportType = params->REPORT_VERSION;


    // ------ Initialise Attack  ------

    switch (myMdType) {
    case mbTypes::Genuine: {
        vehicle_api.setColor(id, libsumo::TraCIColor(0, 255, 0, 255));
        myAttackType = attackTypes::Genuine;
    } break;

    case mbTypes::GlobalAttacker: {
        vehicle_api.setColor(id, libsumo::TraCIColor(0, 255, 0, 255));
        myAttackType = params->GLOBAL_ATTACK_TYPE;

        mdGlobalAttack = MDGlobalAttack();

        mdGlobalAttack.setMyPseudonym(&myPseudonym);
        mdGlobalAttack.setCurHeading(&curHeading);
        mdGlobalAttack.setCurHeadingConfidence(&curHeadingConfidence);
        mdGlobalAttack.setCurPosition(&curPosition);
        mdGlobalAttack.setCurPositionConfidence(&curPositionConfidence);
        mdGlobalAttack.setCurSpeed(&curSpeed);
        mdGlobalAttack.setCurSpeedConfidence(&curSpeedConfidence);
        mdGlobalAttack.setCurAccel(&curAccel);
        mdGlobalAttack.setCurAccelConfidence(&curAccelConfidence);
        mdGlobalAttack.setcurGPSCoordinates(&curGPSCoordinates);

        mdGlobalAttack.init(myAttackType);

    } break;

    case mbTypes::LocalAttacker: {
        if (params->UseAttacksServer) {
            myAttackType = localAttackServer.getNextAttack();
        }
        else if (params->MixLocalAttacks) {
            int AtLiSize = sizeof(params->MixLocalAttacksList) / sizeof(params->MixLocalAttacksList[0]);
            int localAttackIndex = 0;
            if (params->RandomLocalMix) {
                localAttackIndex = genLib.RandomInt(0, AtLiSize - 1);
            }
            else {
                if (LastLocalAttackIndex < (AtLiSize - 1)) {
                    localAttackIndex = LastLocalAttackIndex + 1;
                    LastLocalAttackIndex = localAttackIndex;
                }
                else {
                    localAttackIndex = 0;
                    LastLocalAttackIndex = 0;
                }
            }
            myAttackType = params->MixLocalAttacksList[localAttackIndex];
        }
        else {
            myAttackType = params->LOCAL_ATTACK_TYPE;
        }

        mdAttack = MDAttack();
        mdAttack.setBeaconInterval(&beaconInterval);
        mdAttack.setCurHeading(&curHeading);
        mdAttack.setCurHeadingConfidence(&curHeadingConfidence);
        mdAttack.setCurPosition(&curPosition);
        mdAttack.setCurPositionConfidence(&curPositionConfidence);
        mdAttack.setCurSpeed(&curSpeed);
        mdAttack.setCurSpeedConfidence(&curSpeedConfidence);
        mdAttack.setCurAccel(&curAccel);
        mdAttack.setCurAccelConfidence(&curAccelConfidence);
        mdAttack.setDetectedNodes(&detectedNodes);
        mdAttack.setMyBsm(myBsm);
        mdAttack.setMyBsmNum(&myBsmNum);
        mdAttack.setMyLength(&myLength);
        mdAttack.setMyPseudonym(&myPseudonym);
        mdAttack.setMyWidth(&myWidth);
        mdAttack.setPcPolicy(&pcPolicy);

        mdAttack.init(myAttackType, MaxRandomPosX, MaxRandomPosY, MinRandomPosX, MinRandomPosY, params);

        vehicle_api.setColor(id, libsumo::TraCIColor(255, 0, 0, 255));

    } break;

    default:
        vehicle_api.setColor(id, libsumo::TraCIColor(0, 0, 0, 0));
        break;
    }

    if (!setDate) {
            char dateBuffer[50];
            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            sprintf(dateBuffer, "%d-%d-%d_%d:%d:%d", tm.tm_year + 1900,
                tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
                tm.tm_sec);
            std::string curDateTemp(dateBuffer);
            curDate = curDateTemp;
            setDate = true;
    }

    if (params->writeVeReMi) {
        VeReMi.initVeReMiPrintable(params->savePath, params->serialNumber, myId,
            node->getId(), myAttackType, curDate,
            params->VeReMiSliceTime, VeReMiSliceStartTime, simTime().dbl());
    }
}

/* ----------------------------------------------------------------------------------------------
------------------------------------- F2MD - Attacks & BSMs Generation --------------------------
-----------------------------------------------------------------------------------------------*/

/**
   *  @brief Creates a new BSM with up to date datas. An attack is added if the node is an attacker
   *  @result newly created BSM
*/
veins::BSM* F2MD_CAMFacility::createBSM(){
    auto packet = new veins::BSM();

    packet->setSenderPos(curPosition);
    packet->setSenderSpeed(curSpeed);
    packet->setSenderAccel(curAccel);
    packet->setSenderHeading(curHeading);
    packet->setSenderGpsCoordinates(curGPSCoordinates);

    packet->setSenderWidth(myWidth);
    packet->setSenderLength(myLength);

    packet->setSenderAddress(myId);
    packet->setSenderMbType(myMdType);
    packet->setSenderAttackType(myAttackType);
    packet->setSenderPseudonym(myPseudonym);
        
    packet->setSenderPosConfidence(curPositionConfidence);
    packet->setSenderSpeedConfidence(curSpeedConfidence);
    packet->setSenderHeadingConfidence(curHeadingConfidence);
    packet->setSenderAccelConfidence(curAccelConfidence);
    packet->setByteLength(40);
      
    addMyBsm(*packet);


    if (myMdType == mbTypes::LocalAttacker) {
        if (attackBsm.getSenderPseudonym() != 0) {
            packet->setSenderPseudonym(attackBsm.getSenderPseudonym());

            packet->setSenderPos(attackBsm.getSenderPos());
            packet->setSenderPosConfidence(attackBsm.getSenderPosConfidence());

            auto tracipos = libsumo::TraCIPosition();
            tracipos.x= attackBsm.getSenderPos().x;
            tracipos.y=attackBsm.getSenderPos().y;
            tracipos.z=attackBsm.getSenderPos().z;
            auto geopos = mVehicleController->getTraCI()->convertGeo(tracipos);
            packet->setSenderGpsCoordinates(Coord(geopos.longitude, geopos.latitude,0));

            packet->setSenderSpeed(attackBsm.getSenderSpeed());
            packet->setSenderSpeedConfidence(
                attackBsm.getSenderSpeedConfidence());

            packet->setSenderHeading(attackBsm.getSenderHeading());
            packet->setSenderHeadingConfidence(
                attackBsm.getSenderHeadingConfidence());

            packet->setSenderAccel(attackBsm.getSenderAccel());
            packet->setSenderAccelConfidence(
                attackBsm.getSenderAccelConfidence());

            packet->setSenderWidth(attackBsm.getSenderWidth());
            packet->setSenderLength(attackBsm.getSenderLength());
        }
        else {
            packet->setSenderMbType(mbTypes::Genuine);
            //packet->setSenderAttackType(attackTypes::Genuine);
        }
    }

    if (params->writeVeReMi) {
        VeReMi.serializeGroundTruth(packet);
    }

    if (params->sendReportsV1) {
        std::string reportStr = "curTime:";
        reportStr.append(std::to_string(simTime().dbl()));
        HTTPRequest httpr = HTTPRequest(params->maPortV1, params->maHostV1);
        std::string response = httpr.Request(reportStr);
    }
    if (params->sendReportsV2) {
        std::string reportStr = "curTime:";
        reportStr.append(std::to_string(simTime().dbl()));
        HTTPRequest httpr = HTTPRequest(params->maPortV2, params->maHostV2);
        std::string response = httpr.Request(reportStr);
    }
    if (params->writeSelfMsg) {
        writeSelfBsm(myBsm[0]);
    }

    if (params->writeListSelfMsg) {
        writeSelfListBsm(myBsm[0]);
    }
    if (myReportType == reportTypes::ProtocolReport) {
        handleReportProtocol(false);
    }

    if (params->EnablePC) {
    pcPolicy.checkPseudonymChange(myPcType);
    }

    return packet;
}

static double totalGenuine = 0;
static double totalLocalAttacker = 0;
static double totalGlobalAttacker = 0;

/**
   *  @brief defines the attack type of the node (genuine or attacker)
   *  @param double number of attacker already in the sim
*/
mbTypes::Mbs F2MD_CAMFacility::induceMisbehavior(double localAttacker, double globalAttacker)
{
    /*
    if (simTime().dbl() < params->START_ATTACK) {
        return mbTypes::Genuine;
    }
    */
    if ((totalLocalAttacker + totalGenuine) == 0) {
        totalGenuine++;
        return mbTypes::Genuine;
    }

    double realFactor = totalLocalAttacker / (totalGenuine + totalLocalAttacker);
    if (localAttacker > realFactor) {
        totalLocalAttacker++;
        return mbTypes::LocalAttacker;
    }
    else {
        double realGFactor = totalGlobalAttacker / (totalGenuine + totalGlobalAttacker);
        if (globalAttacker > realGFactor) {
            totalGlobalAttacker++;
            return mbTypes::GlobalAttacker;
        }
        else {
            totalGenuine++;
            return mbTypes::Genuine;
        }
    }
}

/**
   *  @brief turn the color of a node into black if detected
*/
void F2MD_CAMFacility::treatAttackFlags()
{

    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;

    if (myMdType == mbTypes::LocalAttacker) {
        attackBsm = mdAttack.launchAttack(myAttackType, &linkControl);

        if (mdAttack.getTargetNode() >= 0) {
            addTargetNode(mdAttack.getTargetNode());
        }

        if (isAccusedNode(myPseudonym)) {
            vehicle_api.setColor(id, libsumo::TraCIColor(0, 0, 0, 255));
        }
        else {
            vehicle_api.setColor(id, libsumo::TraCIColor(255, 0, 0, 255));
        }
    }
    else {
        if (isTargetNode(myPseudonym)) {
            vehicle_api.setColor(id, libsumo::TraCIColor(255, 255, 0, 255));
        }
        else {
            vehicle_api.setColor(id, libsumo::TraCIColor(0, 255, 0, 255));
        }
        if (isAccusedNode(myPseudonym)) {
            vehicle_api.setColor(id, libsumo::TraCIColor(0, 0, 255, 255));
        }
    }

    if ((simTime().dbl() - targetClearTime) > params->MAX_TARGET_TIME) {
        targetClearTime = simTime().dbl();
        clearTargetNodes();
    }

    if ((simTime().dbl() - accusedClearTime) > params->MAX_ACCUSED_TIME) {
        accusedClearTime = simTime().dbl();
        clearAccusedNodes();
    }
}

/* ----------------------------------------------------------------------------------------------
------------------------------------- F2MD - MbDetection ----------------------------------------
-----------------------------------------------------------------------------------------------*/

/**
   *  @brief perform all the checks on the BSM received to detect a potential attack
   *  @param veins::BSM* BSMrecvd
*/
void F2MD_CAMFacility::onBSM(veins::BSM* BSMrecvd){

    receivedBSMs++;

    if (params->writeVeReMi) {
        VeReMi.serializeBeacon(BSMrecvd);
    }

    unsigned long senderPseudonym = BSMrecvd->getSenderPseudonym();

    const std::string id = mVehicleController->getVehicleId();


    if (params->EnableV1) {
        EV_INFO << " Node " << id << " launches MisbdetectV1 on node " << BSMrecvd->getSenderAddress() << " who is " << BSMrecvd->getSenderMbType() << endl;
        LocalMisbehaviorDetection(BSMrecvd, 1);
    }

    if (params->EnableV2) {
        EV_INFO << " Node " << id << " launches MisbdetectV1 on node " << BSMrecvd->getSenderAddress() << " who is " << BSMrecvd->getSenderMbType() << endl;
        LocalMisbehaviorDetection(BSMrecvd, 2);
    }

    if (!detectedNodes.includes(senderPseudonym)) {
        NodeHistory newNode(senderPseudonym);
        newNode.addBSM(*BSMrecvd);
        MDMHistory newMDM(senderPseudonym);
        if (params->EnableV1) {
            newMDM.addBsmCheck(bsmCheckV1, 1);
            newMDM.initKalman(BSMrecvd, 1);
        }
        if (params->EnableV2) {
            newMDM.addBsmCheck(bsmCheckV2, 2);
            newMDM.initKalman(BSMrecvd, 2);
        }
        detectedNodes.put(senderPseudonym, newNode, newMDM,
            &reportProtocolEnforcerV1, &reportProtocolEnforcerV2);
    }
    else {
        NodeHistory* existingNode = detectedNodes.getNodeHistoryAddr(
            senderPseudonym);
        existingNode->addBSM(*BSMrecvd);
        MDMHistory* existingMDM = detectedNodes.getMDMHistoryAddr(
            senderPseudonym);
        if (params->EnableV1) {
            existingMDM->addBsmCheck(bsmCheckV1, 1);
        }
        if (params->EnableV2) {
            existingMDM->addBsmCheck(bsmCheckV2, 2);
        }
        //detectedNodes.put(senderPseudonym, *existingNode, *existingMDM);
    }
}

/**
   *  @brief local Misbehavior Detection on the received BSM
   *  @param veins::BSM* received BSM, on which the MbDetection must be performed
   *  @param int version of the test
*/
void F2MD_CAMFacility::LocalMisbehaviorDetection(veins::BSM* BSMrecvd, int version){

    unsigned long senderPseudo = BSMrecvd->getSenderPseudonym();

    switch (version) {
    case 1: {
        std::string mdv = "V1";
        switch (params->checksVersionV1) {

        case mdChecksVersionTypes::LegacyChecks: {
            LegacyChecks mdm(version, myPseudonym, curPosition, curSpeed,
                curHeading, Coord(myWidth, myLength),
                Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                    MAX_PLAUSIBLE_DECEL),
                &linkControl, &realDynamicMap, myId, params);
            bsmCheckV1 = mdm.CheckBSM(BSMrecvd, &detectedNodes);
        } break;
        case mdChecksVersionTypes::CatchChecks: {
            CaTChChecks mdm(version, myPseudonym, curPosition,
                curPositionConfidence, curHeading, curHeadingConfidence,
                Coord(myWidth, myLength),
                Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                    MAX_PLAUSIBLE_DECEL),
                &linkControl, &realDynamicMap, myId, params);
            bsmCheckV1 = mdm.CheckBSM(BSMrecvd, &detectedNodes);
        } break;
        case mdChecksVersionTypes::ExperiChecks: {
            ExperiChecks mdm(version, myPseudonym, curPosition,
                curPositionConfidence, curHeading, curHeadingConfidence,
                Coord(myWidth, myLength),
                Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                    MAX_PLAUSIBLE_DECEL),
                &linkControl, &realDynamicMap, myId, params);
            bsmCheckV1 = mdm.CheckBSM(BSMrecvd, &detectedNodes);
        } break;
        default: {
            LegacyChecks mdm(version, myPseudonym, curPosition, curSpeed,
                curHeading, Coord(myWidth, myLength),
                Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                    MAX_PLAUSIBLE_DECEL),
                &linkControl, &realDynamicMap, myId, params);
            bsmCheckV1 = mdm.CheckBSM(BSMrecvd, &detectedNodes);
        } break;
        }

        bool result = false;
        auto startV1 = high_resolution_clock::now();
        result = AppV1->CheckNodeForReport(myPseudonym, BSMrecvd, &bsmCheckV1,
            &detectedNodes);
        auto endV1 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(endV1 - startV1);
        meanTimeV1 = ((double) numTimeV1 * meanTimeV1 + (duration.count())) / ((double) numTimeV1 + 1.0);
        numTimeV1 = numTimeV1 + 1;

        if (params->enableVarThreV1) {
            varThrePrintableV1.registerMessage(
                mbTypes::intMbs[BSMrecvd->getSenderMbType()],
                AppV1->getMinFactor());
        }

        // Generate reports for the Misbehavior Authority
        if (result && (myMdType == mbTypes::Genuine)) {
            MDReport reportBase;
            reportBase.setGenerationTime(simTime().dbl());
            reportBase.setSenderPseudo(myPseudonym);
            reportBase.setReportedPseudo(senderPseudo);

            reportBase.setSenderRealId(myId);
            reportBase.setReportedRealId(BSMrecvd->getSenderAddress());

            reportBase.setMbType(mbTypes::mbNames[BSMrecvd->getSenderMbType()]);
            reportBase.setAttackType(
                attackTypes::AttackNames[BSMrecvd->getSenderAttackType()]);
            reportBase.setSenderGps(Coord(curGPSCoordinates.x, curGPSCoordinates.y));
            reportBase.setReportedGps(BSMrecvd->getSenderGpsCoordinates());

            char nameV1[32] = "mdaV1";
            mdStats.getReport(nameV1, reportBase);
            if (params->writeReportsV1 && myBsmNum > 0) {
                writeReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
            }

            if (params->writeListReportsV1 && myBsmNum > 0) {
                writeListReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
            }

            if (params->sendReportsV1 && myBsmNum > 0) {
        
                sendReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
                

            }
        }
        else if (myMdType == mbTypes::GlobalAttacker) {
            MDReport reportBase = mdGlobalAttack.launchAttack(myAttackType,
                BSMrecvd);

            if (params->writeReportsV1) {
                writeReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
            }

            if (params->writeListReportsV1) {
                writeListReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
            }

            if (params->sendReportsV1) {
                sendReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
            }
        }

        if (params->writeBsmsV1) {
            writeMdBsm(mdv, bsmCheckV1, BSMrecvd);
        }
        if (params->writeListBsmsV1) {
            writeMdListBsm(mdv, bsmCheckV1, BSMrecvd);
        }

        if (!initV1) {
            AppV1->resetAllFlags();
            //mdAuthority.resetAll();
            initV1 = true;
        }

        if ((simTime().dbl() - deltaTV1) > params->SAVE_PERIOD) {
            bool printOut = false;
            if ((simTime().dbl() - deltaTVS1) > params->PRINT_PERIOD) {
                deltaTVS1 = simTime().dbl();
                printOut = true;
                std::cout << "-_-_-_-_-_-_-_-_-_-_-_-_-"
                          << " meanTimeV1:"
                          << meanTimeV1 << " μs " << numTimeV1 << "\n";
            }

            deltaTV1 = simTime().dbl();

            if ((simTime().dbl() > params->START_SAVE) && params->SaveStatsV1) {

                AppV1->saveLine(params->savePath, params->serialNumber,
                    (double)mVehicleController->getTraCI()->vehicle.getIDCount(),
                    deltaTV1, printOut);

                mdStats.saveLine(params->savePath, params->serialNumber, deltaTV1, printOut);
                if (params->enableVarThreV1) {
                    varThrePrintableV1.saveFile(params->savePath, params->serialNumber,
                        printOut);
                }
            }
            AppV1->resetInstFlags();
        }
        if (result) {
            // Add the node in the list of accused nodes
            addAccusedNode(senderPseudo);
        }

        break;

    }
    case 2: {

        std::string mdv = "V2";

        switch (params->checksVersionV2) {
        case mdChecksVersionTypes::LegacyChecks: {
            LegacyChecks mdm(version, myPseudonym, curPosition, curSpeed,
                curHeading, Coord(myWidth, myLength),
                Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                    MAX_PLAUSIBLE_DECEL),
                &linkControl, &realDynamicMap, myId, params);
            bsmCheckV2 = mdm.CheckBSM(BSMrecvd, &detectedNodes);
        } break;
        case mdChecksVersionTypes::CatchChecks: {
            CaTChChecks mdm(version, myPseudonym, curPosition,
                curPositionConfidence, curHeading, curHeadingConfidence,
                Coord(myWidth, myLength),
                Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                    MAX_PLAUSIBLE_DECEL),
                &linkControl, &realDynamicMap, myId, params);
            bsmCheckV2 = mdm.CheckBSM(BSMrecvd, &detectedNodes);
        } break;
        case mdChecksVersionTypes::ExperiChecks: {
            ExperiChecks mdm(version, myPseudonym, curPosition,
                curPositionConfidence, curHeading, curHeadingConfidence,
                Coord(myWidth, myLength),
                Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                    MAX_PLAUSIBLE_DECEL),
                &linkControl, &realDynamicMap, myId, params);
            bsmCheckV2 = mdm.CheckBSM(BSMrecvd, &detectedNodes);
        } break;
        default: {
            LegacyChecks mdm(version, myPseudonym, curPosition, curSpeed,
                curHeading, Coord(myWidth, myLength),
                Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                    MAX_PLAUSIBLE_DECEL),
                &linkControl, &realDynamicMap, myId, params);
            bsmCheckV2 = mdm.CheckBSM(BSMrecvd, &detectedNodes);
        } break;
        }

        bool result = false;
        auto startV2 = high_resolution_clock::now();
        result = AppV2->CheckNodeForReport(myPseudonym, BSMrecvd, &bsmCheckV2,
            &detectedNodes);
        auto endV2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(endV2 - startV2);
        meanTimeV2 = ((double) numTimeV2 * meanTimeV2 + (duration.count())) / ((double) numTimeV2 + 1.0);
        numTimeV2 = numTimeV2 + 1;

        if (params->enableVarThreV2) {
            varThrePrintableV2.registerMessage(
                mbTypes::intMbs[BSMrecvd->getSenderMbType()],
                AppV2->getMinFactor());
        }

        if (result && (myMdType == mbTypes::Genuine)) {
            MDReport reportBase;
            reportBase.setGenerationTime(simTime().dbl());
            reportBase.setSenderPseudo(myPseudonym);
            reportBase.setReportedPseudo(senderPseudo);

            reportBase.setSenderRealId(myId);
            reportBase.setReportedRealId(BSMrecvd->getSenderAddress());

            reportBase.setMbType(mbTypes::mbNames[BSMrecvd->getSenderMbType()]);
            reportBase.setAttackType(
                attackTypes::AttackNames[BSMrecvd->getSenderAttackType()]);
            reportBase.setSenderGps(Coord(curGPSCoordinates.x, curGPSCoordinates.y));
            reportBase.setReportedGps(BSMrecvd->getSenderGpsCoordinates());

            char nameV2[32] = "mdaV2";
            mdStats.getReport(nameV2, reportBase);

            if (params->writeReportsV2 && myBsmNum > 0) {
                writeReport(reportBase, version, mdv, bsmCheckV2, BSMrecvd);
            }

            if (params->writeListReportsV2 && myBsmNum > 0) {
                writeListReport(reportBase, version, mdv, bsmCheckV2, BSMrecvd);
            }

            if (params->sendReportsV2 && myBsmNum > 0) {
                sendReport(reportBase, version, mdv, bsmCheckV2, BSMrecvd);
            }
        }
        else if (myMdType == mbTypes::GlobalAttacker) {
            MDReport reportBase = mdGlobalAttack.launchAttack(myAttackType,
                BSMrecvd);

            if (params->writeReportsV2) {
                writeReport(reportBase, version, mdv, bsmCheckV2, BSMrecvd);
            }

            if (params->writeListReportsV2) {
                writeListReport(reportBase, version, mdv, bsmCheckV2, BSMrecvd);
            }

            if (params->sendReportsV2) {
                sendReport(reportBase, version, mdv, bsmCheckV2, BSMrecvd);
            }
        }

        if (params->writeBsmsV2) {
            writeMdBsm(mdv, bsmCheckV2, BSMrecvd);
        }

        if (params->writeListBsmsV2) {
            writeMdListBsm(mdv, bsmCheckV2, BSMrecvd);
        }

        if (!initV2) {
            AppV2->resetAllFlags();
            //mdAuthority.resetAll();
            initV2 = true;
        }

        if ((simTime().dbl() - deltaTV2) > params->SAVE_PERIOD) {
            bool printOut = false;
            if ((simTime().dbl() - deltaTVS2) > params->PRINT_PERIOD) {
                deltaTVS2 = simTime().dbl();
                printOut = true;
                std::cout << "-_-_-_-_-_-_-_-_-_-_-_-_-"
                          << " meanTimeV2:"
                          << meanTimeV2 << " μs " << numTimeV2 << "\n";
            }

            deltaTV2 = simTime().dbl();

            if ((simTime().dbl() > params->START_SAVE) && params->SaveStatsV2) {
                AppV2->saveLine(params->savePath, params->serialNumber,
                    (double)mVehicleController->getTraCI()->vehicle.getIDCount(),
                    deltaTV2, printOut);

                mdStats.saveLine(params->savePath, params->serialNumber, deltaTV2, printOut);
                if (params->enableVarThreV2) {
                    varThrePrintableV2.saveFile(params->savePath, params->serialNumber,
                        printOut);
                }
            }
            AppV2->resetInstFlags();
        }

        if (result) {
            addAccusedNode(senderPseudo);
        }
        break;
    }

    default:
        break;
    }
}

/**
   *  @brief Handles the report protocol
*/
void F2MD_CAMFacility::handleReportProtocol(bool lastTimeStep)
{
    if (myReportType == reportTypes::ProtocolReport) {
        unsigned long pseudosList[MAX_REP_PSEUDOS];
        if (params->EnableV1) {
            int pseudoNum = 0;
            if (lastTimeStep) {
                pseudoNum = reportProtocolEnforcerV1.getAllReportPseudoes(
                    simTime().dbl(), pseudosList);
            }
            else {
                pseudoNum = reportProtocolEnforcerV1.getReportPseudoes(
                    simTime().dbl(), pseudosList);
            }

            for (int var = 0; var < pseudoNum; ++var) {
                if (!detectedNodes.includes(pseudosList[var])) {
                    BSM* reportedBsm =
                        detectedNodes.getNodeHistoryAddr(pseudosList[var])->getLatestBSMAddr();

                    BsmCheck* reportedCheck = detectedNodes.getMDMHistoryAddr(
                                                            pseudosList[var])
                                                ->getLatestBsmCheckAddr(1);

                    MDReport reportBase;
                    reportBase.setGenerationTime(simTime().dbl());
                    reportBase.setSenderPseudo(myPseudonym);
                    reportBase.setReportedPseudo(pseudosList[var]);

                    reportBase.setSenderRealId(myId);
                    reportBase.setReportedRealId(reportedBsm->getSenderAddress());

                    reportBase.setMbType(
                        mbTypes::mbNames[reportedBsm->getSenderMbType()]);
                    reportBase.setAttackType(
                        attackTypes::AttackNames[reportedBsm->getSenderAttackType()]);
                    reportBase.setSenderGps(Coord(curGPSCoordinates.x, curGPSCoordinates.y));
                    reportBase.setReportedGps(
                        reportedBsm->getSenderGpsCoordinates());

                    ProtocolReport hir = ProtocolReport(reportBase);
                    hir.addEvidence(myBsm[0], *reportedCheck, *reportedBsm,
                        &detectedNodes, simTime().dbl(),
                        params->CollectionPeriod, 1);

                    if (params->writeReportsV1 && myBsmNum > 0) {
                        hir.writeStrToFile(params->savePath, params->serialNumber, "V1",
                            hir.getReportPrintableJson(), curDate);
                    }

                    if (params->writeListReportsV1 && myBsmNum > 0) {
                        hir.writeStrToFileList(params->savePath, params->serialNumber, "V1",
                            hir.getReportPrintableJson(), curDate);
                    }

                    if (params->sendReportsV1 && myBsmNum > 0) {
                        HTTPRequest httpr = HTTPRequest(params->maPortV1, params->maHostV1);
                        std::string response = httpr.Request(
                            hir.getReportPrintableJson());
                    }
                }
                else {
                    reportProtocolEnforcerV1.removeReportedPseudo(pseudosList[var]);
                }
            }
        }

        if (params->EnableV2) {

            int pseudoNum = 0;
            if (lastTimeStep) {
                pseudoNum = reportProtocolEnforcerV2.getAllReportPseudoes(
                    simTime().dbl(), pseudosList);
            }
            else {
                pseudoNum = reportProtocolEnforcerV2.getReportPseudoes(
                    simTime().dbl(), pseudosList);
            }

            for (int var = 0; var < pseudoNum; ++var) {
                if (detectedNodes.includes(pseudosList[var])) {
                    BSM* reportedBsm =
                        detectedNodes.getNodeHistoryAddr(pseudosList[var])->getLatestBSMAddr();
                    BsmCheck* reportedCheck = detectedNodes.getMDMHistoryAddr(
                                                            pseudosList[var])
                                                ->getLatestBsmCheckAddr(2);
                    if (reportedBsm->getSenderPseudonym() == 0) {
                        std::cout << pseudosList[var] << "\n";
                        std::cout << "Problem Found \n";
                        exit(0);
                    }

                    MDReport reportBase;
                    reportBase.setGenerationTime(simTime().dbl());
                    reportBase.setSenderPseudo(myPseudonym);
                    reportBase.setReportedPseudo(pseudosList[var]);

                    reportBase.setSenderRealId(myId);
                    reportBase.setReportedRealId(reportedBsm->getSenderAddress());

                    reportBase.setMbType(
                        mbTypes::mbNames[reportedBsm->getSenderMbType()]);
                    reportBase.setAttackType(
                        attackTypes::AttackNames[reportedBsm->getSenderAttackType()]);
                    reportBase.setSenderGps(Coord(curGPSCoordinates.x, curGPSCoordinates.y));
                    reportBase.setReportedGps(
                        reportedBsm->getSenderGpsCoordinates());

                    ProtocolReport hir = ProtocolReport(reportBase);

                    hir.addEvidence(myBsm[0], *reportedCheck, *reportedBsm,
                        &detectedNodes, simTime().dbl(),
                        params->CollectionPeriod, 2);

                    if (params->writeReportsV2 && myBsmNum > 0) {
                        hir.writeStrToFile(params->savePath, params->serialNumber, "V2",
                            hir.getReportPrintableJson(), curDate);
                    }

                    if (params->writeListReportsV2 && myBsmNum > 0) {
                        hir.writeStrToFileList(params->savePath, params->serialNumber, "V2",
                            hir.getReportPrintableJson(), curDate);
                    }

                    if (params->sendReportsV2 && myBsmNum > 0) {
                        HTTPRequest httpr = HTTPRequest(params->maPortV2, params->maHostV2);
                        std::string response = httpr.Request(
                            hir.getReportPrintableJson());
                    }
                }
                else {
                    reportProtocolEnforcerV2.removeReportedPseudo(pseudosList[var]);
                }
            }
        }
    }
}

/**
   *  @brief Set the MisbehaviorDetection App Type
*/
void F2MD_CAMFacility::setMDApp(mdAppTypes::App appTypeV1, mdAppTypes::App appTypeV2)
{
    switch (appTypeV1) {
    case mdAppTypes::ThresholdApp:
        AppV1 = &ThreV1;
        break;
    case mdAppTypes::AggrigationApp:
        AppV1 = &AggrV1;
        break;
    case mdAppTypes::BehavioralApp:
        AppV1 = &BehaV1;
        break;
    case mdAppTypes::CooperativeApp:
        AppV1 = &CoopV1;
        break;
    case mdAppTypes::ExperiApp:
        AppV1 = &ExperV1;
        break;
    case mdAppTypes::MachineLearningApp:
        AppV1 = &PybgV1;
        PybgV1.setMyId(myId);
        break;
    default:
        AppV1 = &ThreV1;
        break;
    }
    switch (appTypeV2) {
    case mdAppTypes::ThresholdApp:
        AppV2 = &ThreV2;
        break;
    case mdAppTypes::AggrigationApp:
        AppV2 = &AggrV2;
        break;
    case mdAppTypes::BehavioralApp:
        AppV2 = &BehaV2;
        break;
    case mdAppTypes::CooperativeApp:
        AppV2 = &CoopV2;
        break;
    case mdAppTypes::ExperiApp:
        AppV2 = &ExperV2;
        break;
    case mdAppTypes::MachineLearningApp:
        AppV2 = &PybgV2;
        PybgV2.setMyId(myId);
        break;
    default:
        AppV2 = &ThreV2;
        break;
    }
}

// ---------------------------------- F2MD - Utilities ----------------------------------

/**
   *  @brief Update the vehicle datas in CAM F2MD Facility, so that the CAM contains datas that are up to date.
*/
void F2MD_CAMFacility::updateVehicleState(){

    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;

    auto pos = vehicle_api.getPosition(id);
    auto GPScoord = mVehicleController->getTraCI()->convertGeo(pos);
    double speed = vehicle_api.getSpeed(id);
    double angle = vehicle_api.getAngle(id);
    double angle_rad = angle*M_PI/180;
    double accel = vehicle_api.getAcceleration(id);

    auto& lanescope = mVehicleController->getTraCI()->lane;

    auto newPosition = veins::Coord(pos.x,pos.y,pos.z);
    auto newSpeed = veins::Coord(speed*cos(angle_rad-M_PI/2),speed*sin(angle_rad-M_PI/2),0);
    auto newHeading = veins::Coord(cos(angle_rad-M_PI/2), -sin(angle_rad-M_PI/2));
    auto newAccel = veins::Coord(accel*cos(angle_rad-M_PI/2),accel*sin(angle_rad-M_PI/2),0);

    curGPSCoordinates = veins::Coord(GPScoord.longitude,GPScoord.latitude,0);
    myWidth = vehicle_api.getWidth(id);
    myLength = vehicle_api.getLength(id);

    realDynamicMap[myId.getInt()] = newPosition;

    RelativeOffsetConf relativeOffsetConfidence = RelativeOffsetConf(
        &ConfPosMax, &ConfSpeedMax, &ConfHeadMax, &ConfAccelMax,
        &deltaConfPos, &deltaConfSpeed, &deltaConfHead, &deltaConfAccel);

    if (params->veremiConf) {
        double posEtx = genLib.GaussianRandomDouble(
            (curPositionConfidence.x + curPositionConfidenceOrig.x) / 2,
            0.03 * curPositionConfidenceOrig.x);
        double posEty = genLib.GaussianRandomDouble(
            (curPositionConfidence.y + curPositionConfidenceOrig.y) / 2,
            0.03 * curPositionConfidenceOrig.y);

        curPositionConfidence = Coord(posEtx, posEty, 0);

        curHeadingConfidence = Coord(
            curHeadingConfidenceOrig.x * exp(-0.1 * newSpeed.x),
            curHeadingConfidenceOrig.y * exp(-0.1 * newSpeed.y), 0);

        Coord oldSpeedConfidence = Coord(curSpeedConfidence.x,
            curSpeedConfidence.y, curSpeedConfidence.z);
        curSpeedConfidence = Coord(curSpeed.x * curSpeedConfidenceOrig.x,
            curSpeed.y * curSpeedConfidenceOrig.y, 0);

        curAccelConfidence = Coord(
            fabs((curSpeedConfidence.x - oldSpeedConfidence.x)) / (simTime().dbl() - lastPositionUpdate),
            fabs((curSpeedConfidence.y - oldSpeedConfidence.y)) / (simTime().dbl() - lastPositionUpdate),
            fabs((curSpeedConfidence.z - oldSpeedConfidence.z)) / (simTime().dbl() - lastPositionUpdate));

        lastPositionUpdate = simTime().dbl();

        //curHeadingConfidence = Coord(0, 0, 0);

        RelativeOffset relativeOffset = RelativeOffset(&curPositionConfidence,
            &curSpeedConfidence, &curHeadingConfidence, &curAccelConfidence,
            &deltaRPosition, &deltaThetaPosition, &deltaSpeed,
            &deltaHeading, &deltaAccel);

        curPosition = relativeOffset.OffsetPosition(
            newPosition);
        curSpeed = relativeOffset.OffsetSpeed(newSpeed);
        curHeading = relativeOffset.OffsetHeading(
            newHeading);
        curAccel = relativeOffset.OffsetAccel(newAccel);
    }
    else {
        curPositionConfidence = relativeOffsetConfidence.OffsetPosConf(
            curPositionConfidenceOrig);
        curSpeedConfidence = relativeOffsetConfidence.OffsetSpeedConf(
            curSpeedConfidenceOrig);
        curHeadingConfidence = relativeOffsetConfidence.OffsetHeadingConf(
            curHeadingConfidenceOrig);
        curAccelConfidence = relativeOffsetConfidence.OffsetAccelConf(
            curAccelConfidenceOrig);

        RelativeOffset relativeOffset = RelativeOffset(&curPositionConfidence,
            &curSpeedConfidence, &curHeadingConfidence, &curAccelConfidence,
            &deltaRPosition, &deltaThetaPosition, &deltaSpeed,
            &deltaHeading, &deltaAccel);

        //    GaussianRandom relativeOffset = GaussianRandom(&curPositionConfidence,
        //            &curSpeedConfidence, &curHeadingConfidence);

        curPosition = relativeOffset.OffsetPosition(
            newPosition);
        curSpeed = relativeOffset.OffsetSpeed(newSpeed);
        curHeading = relativeOffset.OffsetHeading(
            newHeading);
        curAccel = relativeOffset.OffsetAccel(newAccel);
    }
    if (params->writeVeReMi) {
        VeReMi.serializeRawData(&curPosition, &curPositionConfidence, &curSpeed,
            &curSpeedConfidence, &curHeading, &curHeadingConfidence,
            &curAccel, &curAccelConfidence);
    }

    printVehData();
}

/**
   *  @brief write a new report
*/
void F2MD_CAMFacility::writeReport(MDReport reportBase, int version,
    std::string maversion, BsmCheck bsmCheck, BSM* bsm)
{
    switch (myReportType) {
    case reportTypes::BasicCheckReport: {
        BasicCheckReport bcr = BasicCheckReport(reportBase);
        bcr.setReportedCheck(bsmCheck);
        bcr.writeStrToFile(params->savePath, params->serialNumber, maversion,
            bcr.getReportPrintableJson(), curDate);
    } break;

    case reportTypes::OneMessageReport: {
        OneMessageReport omr = OneMessageReport(reportBase);
        omr.setReportedBsm(*bsm);
        omr.setReportedCheck(bsmCheck);
        omr.writeStrToFile(params->savePath, params->serialNumber, maversion,
            omr.getReportPrintableJson(), curDate);
    } break;
    case reportTypes::EvidenceReport: {
        EvidenceReport evr = EvidenceReport(reportBase);
        if (myBsmNum > 0) {
            evr.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes);
            evr.writeStrToFile(params->savePath, params->serialNumber, maversion,
                evr.getReportPrintableJson(), curDate);
        }
        else {
            OneMessageReport omr = OneMessageReport(reportBase);
            omr.setReportedBsm(*bsm);
            omr.setReportedCheck(bsmCheck);
            omr.writeStrToFile(params->savePath, params->serialNumber, maversion,
                omr.getReportPrintableJson(), curDate);
        }
    } break;
    case reportTypes::ProtocolReport: {
        bool newNode = false;
        switch (version) {
        case 1: {
            newNode = reportProtocolEnforcerV1.addMisbehavingPseudo(
                bsm->getSenderPseudonym(), simTime().dbl());
        } break;
        case 2: {
            newNode = reportProtocolEnforcerV2.addMisbehavingPseudo(
                bsm->getSenderPseudonym(), simTime().dbl());
        } break;
        }
        if (newNode) {
            ProtocolReport hir = ProtocolReport(reportBase);
            hir.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes,
                simTime().dbl(), params->InitialHistory, version);
            hir.writeStrToFile(params->savePath, params->serialNumber, maversion,
                hir.getReportPrintableJson(), curDate);
        }
    } break;
    default:
        break;
    }
}

void F2MD_CAMFacility::writeListReport(MDReport reportBase, int version,
    std::string maversion, BsmCheck bsmCheck, BSM* bsm)
{

    switch (myReportType) {
    case reportTypes::BasicCheckReport: {
        BasicCheckReport bcr = BasicCheckReport(reportBase);
        bcr.setReportedCheck(bsmCheck);
        bcr.writeStrToFileList(params->savePath, params->serialNumber, maversion,
            bcr.getReportPrintableJson(), curDate);
    } break;

    case reportTypes::OneMessageReport: {
        OneMessageReport omr = OneMessageReport(reportBase);
        omr.setReportedBsm(*bsm);
        omr.setReportedCheck(bsmCheck);
        omr.writeStrToFileList(params->savePath, params->serialNumber, maversion,
            omr.getReportPrintableJson(), curDate);
    } break;
    case reportTypes::EvidenceReport: {
        EvidenceReport evr = EvidenceReport(reportBase);
        if (myBsmNum > 0) {
            evr.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes);
            evr.writeStrToFileList(params->savePath, params->serialNumber, maversion,
                evr.getReportPrintableJson(), curDate);
        }
        else {
            OneMessageReport omr = OneMessageReport(reportBase);
            omr.setReportedBsm(*bsm);
            omr.setReportedCheck(bsmCheck);
            omr.writeStrToFileList(params->savePath, params->serialNumber, maversion,
                omr.getReportPrintableJson(), curDate);
        }
    } break;
    case reportTypes::ProtocolReport: {
        bool newNode = false;
        switch (version) {
        case 1: {
            newNode = reportProtocolEnforcerV1.addMisbehavingPseudo(
                bsm->getSenderPseudonym(), simTime().dbl());
        } break;
        case 2:
            newNode = reportProtocolEnforcerV2.addMisbehavingPseudo(
                bsm->getSenderPseudonym(), simTime().dbl());
            break;
        }
        if (newNode) {
            ProtocolReport hir = ProtocolReport(reportBase);
            hir.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes,
                simTime().dbl(), params->InitialHistory, version);
            hir.writeStrToFileList(params->savePath, params->serialNumber, maversion,
                hir.getReportPrintableJson(), curDate);
        }
    } break;
    default:
        break;
    }
}

void F2MD_CAMFacility::sendReport(MDReport reportBase, int version,
    std::string maversion, BsmCheck bsmCheck, BSM* bsm)
{
    std::string reportStr = "";

    switch (myReportType) {
    case reportTypes::BasicCheckReport: {
        BasicCheckReport bcr = BasicCheckReport(reportBase);
        bcr.setReportedCheck(bsmCheck);
        reportStr = bcr.getReportPrintableJson();
    } break;

    case reportTypes::OneMessageReport: {
        OneMessageReport omr = OneMessageReport(reportBase);
        omr.setReportedBsm(*bsm);
        omr.setReportedCheck(bsmCheck);
        reportStr = omr.getReportPrintableJson();
    } break;
    case reportTypes::EvidenceReport: {
        EvidenceReport evr = EvidenceReport(reportBase);
        if (myBsmNum > 0) {
            evr.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes);
            reportStr = evr.getReportPrintableJson();
        }
        else {
            OneMessageReport omr = OneMessageReport(reportBase);
            omr.setReportedBsm(*bsm);
            omr.setReportedCheck(bsmCheck);
            reportStr = omr.getReportPrintableJson();
        }
    } break;
    case reportTypes::ProtocolReport: {
        bool newNode = false;
        switch (version) {
        case 1: {
            newNode = reportProtocolEnforcerV1.addMisbehavingPseudo(
                bsm->getSenderPseudonym(), simTime().dbl());
        } break;
        case 2:

            newNode = reportProtocolEnforcerV2.addMisbehavingPseudo(
                bsm->getSenderPseudonym(), simTime().dbl());
            break;
        }

        if (newNode) {
            ProtocolReport hir = ProtocolReport(reportBase);
            hir.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes,
                simTime().dbl(), params->InitialHistory, version);
            reportStr = hir.getReportPrintableJson();
        }
        else {
            //do not report now
            return;
        }
    } break;
    default:
        reportStr = "ERROR myReportType";
        break;
    }

    //std::cout << reportStr << "\n";
    //exit(0);

    if (!maversion.compare("V1")) {
        HTTPRequest httpr = HTTPRequest(params->maPortV1, params->maHostV1);
        std::string response = httpr.Request(reportStr);
    }

    if (!maversion.compare("V2")) {
        HTTPRequest httpr = HTTPRequest(params->maPortV2, params->maHostV2);
        std::string response = httpr.Request(reportStr);
    }
}

void F2MD_CAMFacility::writeMdBsm(std::string version, BsmCheck bsmCheck,
    BSM* bsm)
{
    BsmPrintable bsmPrint = BsmPrintable();
    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(*bsm);
    bsmPrint.setBsmCheck(bsmCheck);
    bsmPrint.writeStrToFile(params->savePath, params->serialNumber, version,
        bsmPrint.getBsmPrintableJson(), curDate);
}

void F2MD_CAMFacility::writeMdListBsm(std::string version, BsmCheck bsmCheck,
    BSM* bsm)
{
    BsmPrintable bsmPrint = BsmPrintable();
    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(*bsm);
    bsmPrint.setBsmCheck(bsmCheck);
    bsmPrint.writeStrToFileList(params->savePath, params->serialNumber, version,
        bsmPrint.getBsmPrintableJson(), curDate);
}

void F2MD_CAMFacility::writeSelfBsm(BSM bsm)
{
    BsmPrintable bsmPrint = BsmPrintable();
    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(bsm);
    bsmPrint.writeSelfStrToFile(params->savePath, params->serialNumber,
        bsmPrint.getSelfBsmPrintableJson(myVType), curDate);
}

void F2MD_CAMFacility::writeSelfListBsm(BSM bsm)
{
    BsmPrintable bsmPrint = BsmPrintable();
    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(bsm);
    bsmPrint.writeSelfStrToFileList(params->savePath, params->serialNumber,
        bsmPrint.getSelfBsmPrintableJson(myVType), curDate);
}

void F2MD_CAMFacility::addTargetNode(unsigned long id)
{
    bool found = false;
    for (int var = 0; var < targetNodesLength; ++var) {
        if (targetNodes[var] == id) {
            found = true;
        }
    }

    if (!found) {
        targetNodes[targetNodesLength] = id;
        targetNodesLength++;
    }
}

void F2MD_CAMFacility::addAccusedNode(unsigned long id)
{
    bool found = false;
    for (int var = 0; var < accusedNodesLength; ++var) {
        if (accusedNodes[var] == id) {
            found = true;
        }
    }

    if (!found) {
        accusedNodes[accusedNodesLength] = id;
        accusedNodesLength++;
    }
}

bool F2MD_CAMFacility::isAccusedNode(unsigned long id)
{
    for (int var = 0; var < accusedNodesLength; ++var) {
        if (accusedNodes[var] == id) {
            return true;
        }
    }
    return false;
}

bool F2MD_CAMFacility::isTargetNode(unsigned long id)
{
    for (int var = 0; var < targetNodesLength; ++var) {
        if (targetNodes[var] == id) {
            return true;
        }
    }
    return false;
}

void F2MD_CAMFacility::clearAccusedNodes()
{
    accusedNodesLength = 0;
}

void F2MD_CAMFacility::clearTargetNodes()
{
    targetNodesLength = 0;
}

void F2MD_CAMFacility::addMyBsm(BSM bsm)
{
    if (myBsmNum < MYBSM_SIZE) {
        myBsmNum++;
    }
    for (int var = myBsmNum - 1; var > 0; --var) {
        myBsm[var] = myBsm[var - 1];
    }

    myBsm[0] = bsm;
}

void F2MD_CAMFacility::printVehData(){

    EV_INFO << "posX : " << curPosition.x << endl;
    EV_INFO << "posY : " << curPosition.y << endl;
    EV_INFO << "posZ : " << curPosition.z << endl;
    EV_INFO << "GPScoord : " << curGPSCoordinates.x << curGPSCoordinates.y << endl;
    EV_INFO << "SpeedX : " << curSpeed.x << endl;
    EV_INFO << "SpeedX : " << curSpeed.y << endl;
    EV_INFO << "Angle : " << curHeading << endl;
    EV_INFO << "Width : " << myLength << endl;
    EV_INFO << "Length : " << myWidth << endl;
    EV_INFO << "MacAddress : " << myId << endl;
}
