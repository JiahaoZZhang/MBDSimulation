/*******************************************************************************
 * @author  Maxime Georges
 * @email   maxime.georges059@gmail.com
 * @date    10/06/2021
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

/*
# Is used to store the MAX_CPM_LENGTH last CPM send by a node in a simulation. Useful to perform
# tests that requires an historic of the CPM sent.
*/

#ifndef __VEINS_NODEHISTORYCPM_H_
#define __VEINS_NODEHISTORYCPM_H_

#include <omnetpp.h>
#include "../../F2MD_CPParameters.h"
#include <vanetza/asn1/cpm.hpp>

using namespace omnetpp;

class NodeHistoryCpm {
    private:
        int nodeID;
        int cpmNum;
        double lastUpdate;
        vanetza::asn1::Cpm cpmList[MAX_CPM_LENGTH];

    public:
        NodeHistoryCpm();
        NodeHistoryCpm(int);
        NodeHistoryCpm(int, vanetza::asn1::Cpm);
        void addCPM(vanetza::asn1::Cpm cpm);

        vanetza::asn1::Cpm* getCPMAddr(int index);
        vanetza::asn1::Cpm* getLatestCPMAddr();

        //double getSenderPos(int);
        //double getSenderSpeed(int);
        //double getDeltaTime(int, int);
        int getCPMNum();
        int getNodeID();
        double getLastUpdate();
        //double getArrivalTime(int);
    };

#endif
