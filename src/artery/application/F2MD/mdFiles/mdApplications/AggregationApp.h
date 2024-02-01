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

#ifndef __VEINS_AggrigationApp_H_
#define __VEINS_AggrigationApp_H_

#include <tuple>
#include <omnetpp.h>
#include "MDApplication.h"
#include "../mdEnumTypes/MdChecksTypes.h"
#include "../mdEnumTypes/MbTypes.h"

using namespace veins;
using namespace omnetpp;

class AggrigationApp: public MDApplication {
public:

    double Threshold = 0.5;
    double devValue = 0.5;
    double deltaTrustTime = 10;
    int maxBsmTrustNum = 5;

    AggrigationApp(int version, double Threshold, double devValue ,double deltaTrustTime,
            int maxBsmTrustNum);

    bool CheckNodeForReport(unsigned long myPseudonym,
            BSM * bsm, BsmCheck * bsmCheck, NodeTable * detectedNodes);

    double AggregateFactorsListDouble(double curFactor, double *factorList,
            int factorListSize);

    double getMinFactor();
};

#endif
