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

#include <stdio.h>
#include <stdlib.h>     /* atof */
#include <boost/algorithm/string.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include "CooperativeApp.h"
#include <iostream>
#include <string>
#include <vector>


using namespace std;
using namespace boost;

CooperativeApp::CooperativeApp(int version,
        double Threshold) :
        MDApplication(version) {
    this->Threshold = Threshold;
}

/**
   *  @brief checks if a Node must send a report, based on the results of the previous checks
   *  @param long my pseudonym
   *  @param BSM* BSM received by the node, on which the checks were performed
   *  @param BsmCheck* results of the checks
   *  @result True if a report must be sent, False if not
*/
bool CooperativeApp::CheckNodeForReport(unsigned long myPseudonym,
        BSM * bsm, BsmCheck * bsmCheck, NodeTable * detectedNodes) {
    double tempFactor = 0;

    prntApp->incAll(mbTypes::intMbs[bsm->getSenderMbType()]);
    prntAppInst->incAll(mbTypes::intMbs[bsm->getSenderMbType()]);
    calculateMinFactor(bsmCheck);
    incrementDetailedFlags(bsm,bsmCheck,Threshold);

    unsigned long senderId = bsm->getSenderPseudonym();

    bool checkFailed = false;

    int indexTMO = getIndexTMO(senderId);

    if (minFactor < 1) {

        if (indexTMO == -1) {
            indexTMO = addPseudoTMO(senderId);
        }

        double TMOadd = 0.0005*exp(10 * (1 - minFactor));

        TimeOut[indexTMO] = TimeOut[indexTMO] + TMOadd;
        UpdatedTMO[TimeOutNum] = simTime().dbl();
    }


    if (indexTMO >= 0) {
        if (TimeOut[indexTMO] > 0) {
            if(TimeOut[indexTMO] > 5.5066){
                prntApp->incCumulFlags(mbTypes::intMbs[bsm->getSenderMbType()]);
                prntAppInst->incCumulFlags(mbTypes::intMbs[bsm->getSenderMbType()]);
                bsmCheck->setReported(true);
                checkFailed = true;
            }
            TimeOut[indexTMO] = TimeOut[indexTMO] - 1;
            if(TimeOut[indexTMO]<=0){
                TimeOut[indexTMO] == 0;
            }
        }
        if (TimeOut[indexTMO] <=0) {
            removePseudoTMO(indexTMO);
        }
    }
    return checkFailed;
}

int CooperativeApp::addPseudoTMO(unsigned long pseudo) {
    if (TimeOutNum >= MAX_DETECTED_NODES_COOP) {
        removeOldestPseudoTMO();
    }
    TimeOut[TimeOutNum] = 0;
    UpdatedTMO[TimeOutNum] = simTime().dbl();
    PseudonymsToTMO[TimeOutNum] = pseudo;
    TimeOutNum++;

    //std::cout<<"TimeOutNum:"<<TimeOutNum<<"\n";

    return TimeOutNum - 1;
}

void CooperativeApp::removeOldestPseudoTMO() {
    int oldestIndex = 0;
    double oldestTime = UpdatedTMO[oldestIndex];
    for (int var = 0; var < TimeOutNum; ++var) {
        if (oldestTime > UpdatedTMO[var]) {
            oldestTime = UpdatedTMO[var];
            oldestIndex = var;
        }
    }
    removePseudoTMO(oldestIndex);
}

void CooperativeApp::removePseudoTMO(int index) {
    for (int var = index; var < TimeOutNum; ++var) {
        TimeOut[var] = TimeOut[var + 1];
    }
    TimeOutNum--;
}

int CooperativeApp::getIndexTMO(unsigned long pseudo) {
    for (int var = 0; var < TimeOutNum; ++var) {
        if (PseudonymsToTMO[var] == pseudo) {
            return var;
        }
    }
    return -1;
}

/**
   *  @result return the lowest result factor (most critical) of the checks performed
*/
double CooperativeApp::getMinFactor() {
    return minFactor;
}

