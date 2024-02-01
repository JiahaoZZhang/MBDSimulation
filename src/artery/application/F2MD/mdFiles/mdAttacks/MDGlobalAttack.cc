/*******************************************************************************
 * @author  Joseph Kamel 
 * @email   josephekamel@gmail.com
 * @date    28/11/2018
 * @version 2.0
 *
 * SCA (Secure Cooperative Autonomous systems)
 * Copyright (c) 2013, 2018 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "MDGlobalAttack.h"
#include "F2MD_msgs/BSM_m.h"
#include "../mdEnumTypes/AttackTypes.h"

MDGlobalAttack::MDGlobalAttack() {
}

void MDGlobalAttack::init(attackTypes::Attacks myAttackType) {

}

/**
   *  @brief  launch the attack specified in the parameters.
   *  @param  AttackTypes AttackType.
*/
MDReport MDGlobalAttack::launchAttack(attackTypes::Attacks myAttackType, BSM* reportedBsm) {

    MDReport reportBase = MDReport();

    switch (myAttackType) {
    case attackTypes::MAStress: {
        reportBase.setGenerationTime(simTime().dbl());
        reportBase.setSenderPseudo(*myPseudonym);
        reportBase.setReportedPseudo(reportedBsm->getSenderPseudonym());
        reportBase.setMbType(mbTypes::mbNames[reportedBsm->getSenderMbType()]);
        reportBase.setAttackType(attackTypes::AttackNames[reportedBsm->getSenderAttackType()]);
        reportBase.setSenderGps(*curGPSCoordinates);
        reportBase.setReportedGps(reportedBsm->getSenderGpsCoordinates());
    }
        break;
    }

    return reportBase;
}


void MDGlobalAttack::setMyPseudonym(unsigned long * myPseudonym) {
    this->myPseudonym = myPseudonym;
}

void MDGlobalAttack::setCurHeading(Coord* curHeading) {
    this->curHeading = curHeading;
}

void MDGlobalAttack::setCurHeadingConfidence(Coord* curHeadingConfidence) {
    this->curHeadingConfidence = curHeadingConfidence;
}

void MDGlobalAttack::setCurPosition(Coord* curPosition) {
    this->curPosition = curPosition;
}

void MDGlobalAttack::setCurPositionConfidence(Coord* curPositionConfidence) {
    this->curPositionConfidence = curPositionConfidence;
}

void MDGlobalAttack::setCurSpeed(Coord* curSpeed) {
    this->curSpeed = curSpeed;
}

void MDGlobalAttack::setCurSpeedConfidence(Coord* curSpeedConfidence) {
    this->curSpeedConfidence = curSpeedConfidence;
}

void MDGlobalAttack::setCurAccel(Coord* curAccel) {
    this->curAccel = curAccel;
}

void MDGlobalAttack::setCurAccelConfidence(Coord* curAccelConfidence) {
    this->curAccelConfidence = curAccelConfidence;
}

void MDGlobalAttack::setcurGPSCoordinates(Coord* curGPSCoordinates) {
    this->curGPSCoordinates = curGPSCoordinates;
}
