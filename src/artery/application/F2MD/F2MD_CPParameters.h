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

#ifndef __VEINS_F2MD_CPParameters_H_
#define __VEINS_F2MD_CPParameters_H_

#include "mdFiles/mdEnumTypes/AttackTypes.h"
#include "mdFiles/mdEnumTypes/CpAttackTypes.h"
#include "mdFiles/mdEnumTypes/MbTypes.h"
#include "mdFiles/mdEnumTypes/MdAppTypes.h"
#include "mdFiles/mdEnumTypes/MdChecksVersionTypes.h"
#include "mdFiles/mdEnumTypes/PseudoChangeTypes.h"
#include "mdFiles/mdEnumTypes/ReportTypes.h"

#define MAX_TARGET_LENGTH 1000
#define MAX_ACCUSED_LENGTH 1000
#define MAX_REP_PSEUDOS 1000
#define MAX_NODES_LENGTH 200
#define MAX_DETECTED_NODES 5000
#define MAX_DETECTED_NODES_COOP 100000
#define MAX_PSEUDO_LIST 100

#define MAX_CPM_LENGTH 40
#define MAX_HISTORY_TIME 15


class F2MD_CPParameters {
public:
    //Simulation Parameters
    std::string serialNumber = "IRT-DEMO";
    std::string savePath = "../../../../mdmSave/";

    /*##########################################################
    #             /!\ F2MD CPM PARAMETERS  /!\                 #
    ##########################################################*/

    // -------------- CPM ATTACKS PARAMETERS --------------

    bool MixCpLocalAttacks = false;
    bool RandomCpLocalMix = false;

    double CP_LOCAL_ATTACKER_PROB = 0.00;

    cpAttackTypes::Attacks CP_LOCAL_ATTACK_TYPE = cpAttackTypes::Genuine;
    cpAttackTypes::Attacks MixCpLocalAttacksList[19] = {cpAttackTypes::DropObj,
        cpAttackTypes::DropAllObj, cpAttackTypes::AddObj,
        cpAttackTypes::SingleRandomDist, cpAttackTypes::SingleRandomSpeed,
        cpAttackTypes::MultiConstDist, cpAttackTypes::MultiConstSpeed,
        cpAttackTypes::MultiRandomDist, cpAttackTypes::MultiRandomSpeed,
        cpAttackTypes::SingleConstDistOffset, cpAttackTypes::SingleConstSpeedOffset,
        cpAttackTypes::SingleRandomDistOffset, cpAttackTypes::SingleRandomSpeedOffset,
        cpAttackTypes::MultiConstDistOffset, cpAttackTypes::MultiConstSpeedOffset,
        cpAttackTypes::MultiRandomDistOffset, cpAttackTypes::MultiRandomSpeedOffset,
        cpAttackTypes::SingleConstDist, cpAttackTypes::SingleConstSpeed};

    // -------------- CPM ADVANCED PARAMETERS --------------
    double START_CPM_ATTACK = 5; // attack start delay in seconds
    double MAX_CPM_DELTATIME = 5;
    double MAX_CPM_HISTORY_TIME = 15;
    bool KeepSameID = false;
    
    double cpParVar = 0.65;
    double RandomDistOffsetX = 40.0;
    double RandomDistOffsetY = 40.0;
    double RandomSpeed =28.0;
    double RandomSpeedOffset = 7.0;
    double RandomAccel = 2.0;

    double MAX_PLAUSIBLE_ANGLE_CHANGE = 90; // Unit 1 degree/s
    double TOLERANCE_EXCEED_SPEED = 4.0;
    double TOLERANCE_DEARTH_SPEED = 5.0;

    double MAX_PLAUSIBLE_SPEED = 70.0;
    double MAX_PLAUSIBLE_ACCEL = 3.0;
    double MAX_PLAUSIBLE_DECEL = 4.5;


    // -------------- CPM KALMAN SETTING --------------
    double KALMAN_MIN_POS_RANGE = 1.0;
    double MAX_KALMAN_TIME = 3.1;
    double KALMAN_POS_RANGE = 4.0;    
    double KALMAN_MIN_SPEED_RANGE = 1.0;
    double KALMAN_SPEED_RANGE = 4.0;

    // ------------- CPM SENSOR SETTING ------------------
    double MaxRadarRange = 200.0;
    double MaxRadarAngle = 20.0;
    double MaxMapBoundary = 200.0;
    int SegmentAngle = 4;
    int Tolenrance_angle = 2;


    // ------------- CPM RSU SETTING ---------------------
    double GenerationInterval = 1.0;
    double RSUorientationAngle = 0.0;
    int RSUintersectionReferenceID = 0;

    // ------------- VEHICLE CHARACTERISTICS SETTING ------------
    double VEHICLE_LENGTH = 5.0;
    double VEHICLE_WIDTH = 3.6;

    // ------ PSEUDONYM PARAMETERS
    double Period_Change_Time = 240;
    int Tolerance_Buffer = 10;
    double Period_Change_Distance = 80;
    double Random_Change_Chance = 0.1;
    
    bool EnablePC = true;
    pseudoChangeTypes::PseudoChange PC_TYPE = pseudoChangeTypes::Car2car;
    // Periodical, Disposable, DistanceBased, Random, Car2car

};

#endif
