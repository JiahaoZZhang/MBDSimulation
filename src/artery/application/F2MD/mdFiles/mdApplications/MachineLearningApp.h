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

#ifndef __VEINS_MachineLearningApp_H_
#define __VEINS_MachineLearningApp_H_

#include <tuple>
#include <omnetpp.h>
#include <stdio.h>
#include <stdlib.h>     /* atof */
#include <boost/algorithm/string.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include "MDApplication.h"
#include "../mdSupport/BsmPrintable.h"
#include "../mdSupport/HTTPRequest.h"
#include <iostream>
#include <string>
#include <vector>


using namespace veins;
using namespace omnetpp;

class MachineLearningApp: public MDApplication {
public:

    int port = 8888;
    std::string host = "localhost";
    double minFactor = 1;

    inet::MACAddress myId;

    HTTPRequest httpr = HTTPRequest(8888, "localhost");

    BsmPrintable bsmPrint;

    MachineLearningApp(int version ,int port, std::string host);

    bool CheckNodeForReport(unsigned long myPseudonym,
            BSM * bsm, BsmCheck * bsmCheck, NodeTable * detectedNodes);

    void setMyId(inet::MACAddress myId);

    double getMinFactor();
};

#endif
