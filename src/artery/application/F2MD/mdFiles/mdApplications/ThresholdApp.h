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

#ifndef __VEINS_ThresholdApp_H_
#define __VEINS_ThresholdApp_H_

#include <tuple>
#include <omnetpp.h>
#include "MDApplication.h"
#include "../mdEnumTypes/MdChecksTypes.h"
#include "../mdEnumTypes/MbTypes.h"

using namespace veins;
using namespace omnetpp;

class ThresholdApp: public MDApplication {
public:

    double Threshold = 0;

    ThresholdApp(int version,double Threshold);

    bool CheckNodeForReport(unsigned long myPseudonym,
            BSM * bsm, BsmCheck * bsmCheck, NodeTable * detectedNodes);

    double getMinFactor();
};

#endif
