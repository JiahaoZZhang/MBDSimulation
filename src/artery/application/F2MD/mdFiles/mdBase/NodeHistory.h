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

#ifndef __VEINS_NODEHISTORY_H_
#define __VEINS_NODEHISTORY_H_

#include <omnetpp.h>
#include "../../F2MD_CAMParameters.h"
#include "F2MD_msgs/BSM_m.h"

using namespace omnetpp;
using namespace veins;

class NodeHistory {
    private:
        unsigned long nodePseudonym;
        int bsmNum;
        BSM bsmList[MAX_BSM_LENGTH];

    public:
        NodeHistory();
        NodeHistory(unsigned long);
        NodeHistory(unsigned long, BSM);
        void addBSM(BSM bsm);

        BSM* getBSMAddr(int index);
        BSM* getLatestBSMAddr();

        veins::Coord getSenderPos(int);
        veins::Coord getSenderSize(int);
        double getSenderSpeed(int);
        veins::Coord getSenderHeading(int);
        double getDeltaTime(int, int);
        int getBSMNum();
        double getArrivalTime(int);
    };

#endif
