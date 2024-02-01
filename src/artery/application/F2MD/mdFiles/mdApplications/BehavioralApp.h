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

#ifndef __VEINS_BehavioralApp_H_
#define __VEINS_BehavioralApp_H_

#include <tuple>
#include <omnetpp.h>
#include "MDApplication.h"
#include "../mdEnumTypes/MdChecksTypes.h"
#include "../mdEnumTypes/MbTypes.h"

#include "../../F2MD_CAMParameters.h"

using namespace veins;
using namespace omnetpp;

class BehavioralApp: public MDApplication {
public:

    double Threshold = 0.5;


    unsigned long PseudonymsToTMO[MAX_DETECTED_NODES];
    double TimeOut[MAX_DETECTED_NODES];
    double UpdatedTMO[MAX_DETECTED_NODES];
    int TimeOutNum = 0;

    BehavioralApp(int version, double Threshold);

    bool CheckNodeForReport(unsigned long myPseudonym, BSM * bsm,
            BsmCheck * bsmCheck, NodeTable * detectedNodes);

    int addPseudoTMO(unsigned long pseudo);
    void removeOldestPseudoTMO();
    void removePseudoTMO(int index);
    int getIndexTMO(unsigned long pseudo);

    double getMinFactor();
};

#endif