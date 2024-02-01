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

#include "PCPolicyCPM.h"

/**
   *  @brief Init Pseudonym Change Policy
*/
PCPolicyCPM::PCPolicyCPM() {
    realPseudoNum = 0;
    messageToleranceBuffer = 0;
    lastChangeTime = simTime().dbl();
    cumulativeDistance = 0;
    lastPos.x = 0;
    lastPos.y = 0;
}
/**
   *  @brief Init Pseudonym Change Policy using the given parameters
*/
PCPolicyCPM::PCPolicyCPM(libsumo::TraCIPosition curPos, F2MD_CPParameters* params) {
    realPseudoNum = 0;
    messageToleranceBuffer = 0;
    lastChangeTime = simTime().dbl();
    cumulativeDistance = 0;
    lastPos = curPos;
    this->params = params;
}

void PCPolicyCPM::setCurPosition(libsumo::TraCIPosition* curPosition) {
    this->curPosition = curPosition;
}

void PCPolicyCPM::setMyId(inet::MACAddress* myId) {
    this->myId = myId;
}

void PCPolicyCPM::setMyPseudonym(unsigned long* myPseudonym) {
    this->myPseudonym = myPseudonym;
}

void PCPolicyCPM::setPseudoNum(int* pseudoNum) {
    this->pseudoNum = pseudoNum;
}

/**
   *  @result returns the next Pseudonym for this node
*/
unsigned long PCPolicyCPM::getNextPseudonym() {
    if (realPseudoNum < MAX_PSEUDO_LIST) {
        (*pseudoNum)++;
        double simTimeDbl = simTime().dbl();
        while (simTimeDbl > 9) {
            simTimeDbl = simTimeDbl / 10;
        }
        simTimeDbl = (int) simTimeDbl;
        unsigned long pseudo = (*myId).getInt() * 10 + simTimeDbl;
        unsigned long digitNumber = (unsigned long) (log10(pseudo) + 1);
        unsigned long pseudoNumAdd = (*pseudoNum) * pow(10, digitNumber + 1);
        pseudo = pseudo + pseudoNumAdd;

        myPseudonymList[realPseudoNum] = pseudo;
        realPseudoNum++;
        return pseudo;
    } else {
        realPseudoNum++;
        return myPseudonymList[(realPseudoNum - 1) % MAX_PSEUDO_LIST];
    }
}

/**
   *  @brief if necessary, a new pseudonym is determined for this node, depending on the PseudoChange type given as an input
*/
void PCPolicyCPM::checkPseudonymChange(pseudoChangeTypes::PseudoChange myPcType) {
    switch (myPcType) {
    case pseudoChangeTypes::Periodical:
        periodicalPCP();
        break;
    case pseudoChangeTypes::Disposable:
        disposablePCP();
        break;
    case pseudoChangeTypes::DistanceBased:
        distanceBasedPCP();
        break;
    case pseudoChangeTypes::Random:
        randomPCP();
        break;
    case pseudoChangeTypes::Car2car:
        car2carPCP();
        break;
    default:
        break;
    }
}

void PCPolicyCPM::periodicalPCP() {
    if ((simTime().dbl() - lastChangeTime) > params->Period_Change_Time) {
        lastChangeTime = simTime().dbl();
        (*myPseudonym) = getNextPseudonym();
    }
}

void PCPolicyCPM::disposablePCP() {
    if (messageToleranceBuffer > params->Tolerance_Buffer) {
        messageToleranceBuffer = 0;
        (*myPseudonym) = getNextPseudonym();
    } else {
        messageToleranceBuffer++;
    }
}

void PCPolicyCPM::distanceBasedPCP() {
    double stepDistance = mdmLib.calculateDistance(lastPos, (*curPosition));
    lastPos = (*curPosition);

    cumulativeDistance = cumulativeDistance + stepDistance;
    if (cumulativeDistance > params->Period_Change_Distance) {
        (*myPseudonym) = getNextPseudonym();
        cumulativeDistance = 0;
    }
}

void PCPolicyCPM::car2carPCP() {
    double stepDistance = mdmLib.calculateDistance(lastPos, (*curPosition));
    lastPos = (*curPosition);
    cumulativeDistance = cumulativeDistance + stepDistance;
    if (firstChange) {
        if (!randDistanceSet) {
            randDistance = genLib.RandomDouble(800, 1500);
            randDistanceSet = true;
        }
        if (cumulativeDistance > randDistance) {
            (*myPseudonym) = getNextPseudonym();
            cumulativeDistance = 0;
            firstChange = false;
        }
    } else {
        if (cumulativeDistance > 800) {
            if (!randTimeSet) {
                randTime = genLib.RandomDouble(120, 360);
                changeTimerStart = simTime().dbl();
                randTimeSet = true;
            }
            if ((simTime().dbl() - changeTimerStart) > randTime) {
                (*myPseudonym) = getNextPseudonym();
                cumulativeDistance = 0;
                randTimeSet = false;
            }
        }
    }
}

void PCPolicyCPM::randomPCP() {
    double rnd = genLib.RandomDouble(0, 1);
    if (rnd < params->Random_Change_Chance) {
        (*myPseudonym) = getNextPseudonym();
    }
}
