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

#ifndef __VEINS_ExperiApp_H_
#define __VEINS_ExperiApp_H_

#include <tuple>
#include <omnetpp.h>
#include "MDApplication.h"
#include "../mdEnumTypes/MdChecksTypes.h"
#include "../mdEnumTypes/MbTypes.h"
using namespace veins;
using namespace omnetpp;

class ExperiApp: public MDApplication {
public:

    double Threshold = 0.5;
    double deltaTrustTime = 10;
    int maxBsmTrustNum = 5;
    double Augmentation = 1;
    double minFactor = 1;


    ExperiApp(int version, double deltaTrustTime,
            int maxBsmTrustNum, double Augmentation);


    bool CheckNodeForReport(unsigned long myPseudonym,
            BSM * bsm, BsmCheck * bsmCheck, NodeTable * detectedNodes);


//    bool AggregateFactorsList(double curFactor, double *factorList,
//            int factorListSize, double zeroSum, int zeroCount);
    double AggregateFactorsListDouble(double curFactor, double *factorList,
            int factorListSize, double zeroSum, int zeroCount);

    std::tuple<double, int> getZeroNumber(BSM * bsm, BsmCheck * bsmCheck,
            NodeTable * detectedNodes);

    double getMinFactor();
};

#endif
