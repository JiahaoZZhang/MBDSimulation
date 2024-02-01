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
#include "ThresholdApp.h"
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace boost;

ThresholdApp::ThresholdApp(int version, double Threshold) :
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
bool ThresholdApp::CheckNodeForReport(unsigned long myPseudonym,
        BSM * bsm, BsmCheck * bsmCheck,
        NodeTable * detectedNodes) {

    bool checkFailed = false;

    prntApp->incAll(mbTypes::intMbs[bsm->getSenderMbType()]);
    prntAppInst->incAll(mbTypes::intMbs[bsm->getSenderMbType()]);

    calculateMinFactor(bsmCheck);
    incrementDetailedFlags(bsm,bsmCheck,Threshold);

    InterTest inter = bsmCheck->getIntersection();
    for (int var = 0; var < inter.getInterNum(); ++var) {
        if (inter.getInterValue(var) <= Threshold) {
            checkFailed = true;
            break;
        }
    }

    if (checkFailed || bsmCheck->getProximityPlausibility() <= Threshold
    || bsmCheck->getRangePlausibility() <= Threshold
            || bsmCheck->getPositionConsistancy() <= Threshold
            || bsmCheck->getPositionSpeedConsistancy() <= Threshold
            || bsmCheck->getPositionSpeedMaxConsistancy() <= Threshold
            || bsmCheck->getSpeedConsistancy() <= Threshold
            || bsmCheck->getSpeedPlausibility() <= Threshold
            || bsmCheck->getPositionPlausibility() <= Threshold
            || bsmCheck->getBeaconFrequency() <= Threshold
            || bsmCheck->getPositionHeadingConsistancy() <= Threshold
            || bsmCheck->getKalmanPSCP() <= Threshold
            || bsmCheck->getKalmanPSCS() <= Threshold
            || bsmCheck->getKalmanPSCSP() <= Threshold
            || bsmCheck->getKalmanPSCSS() <= Threshold
            || bsmCheck->getKalmanPCC() <= Threshold
            || bsmCheck->getKalmanPACS() <= Threshold
            || bsmCheck->getKalmanSCC() <= Threshold) {
        checkFailed = true;
    }


    if (checkFailed) {
        prntApp->incCumulFlags(mbTypes::intMbs[bsm->getSenderMbType()]);
        prntAppInst->incCumulFlags(mbTypes::intMbs[bsm->getSenderMbType()]);

//        if(bsm->getSenderMbType() == 0){
//            prntApp->printOutDebug();
//        }

    }

    return checkFailed;
}

/**
   *  @result return the lowest result factor (most critical) of the checks performed
*/
double ThresholdApp::getMinFactor() {
    return minFactor;
}
