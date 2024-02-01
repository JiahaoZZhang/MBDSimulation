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

#ifndef __VEINS_MDApplication_H_
#define __VEINS_MDApplication_H_

#include <tuple>
#include <omnetpp.h>
#include "../mdBase/NodeMDMHistory.h"
#include "../mdStats/MDStatistics.h"
#include "../mdSupport/AppPrintable.h"
#include "../mdBase/NodeTable.h"
#include "../mdBase/InterTest.h"
#include "../mdBase/BsmCheck.h"
#include "../mdBase/InterTest.h"
#include "../mdSupport/MDMLib.h"

#include "../mdReport/MDReport.h"

using namespace veins;
using namespace omnetpp;


static AppPrintable prntAppV1;
static AppPrintable prntAppInstV1;

static AppPrintable prntAppV2;
static AppPrintable prntAppInstV2;


class MDApplication {

protected:
    MDMLib mdmLib;

    int version;

    double minFactor = 1;


    char const *AppV1Name = "AppV1";
    char const *AppV2Name = "AppV2";

    AppPrintable* prntApp;
    AppPrintable* prntAppInst;
public:

    MDApplication(int version);

    virtual bool CheckNodeForReport(unsigned long myPseudonym,
            BSM * bsm, BsmCheck * bsmCheck,
            NodeTable * detectedNodes)= 0;


    void calculateMinFactor(BsmCheck * bsmCheck);
    double getMinFactor();
    void incrementDetailedFlags(BSM * bsm,BsmCheck * bsmCheck, double Threshold);

    void saveLine( std::string path, std::string serial, double density,
            double deltaT, bool printOut);
    void resetInstFlags();
    void resetAllFlags();
};



#endif
