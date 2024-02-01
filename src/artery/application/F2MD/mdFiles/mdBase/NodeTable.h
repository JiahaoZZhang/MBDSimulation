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

#ifndef __VEINS_NODETABLE_H_
#define __VEINS_NODETABLE_H_

#include "NodeHistory.h"
#include "NodeMDMHistory.h"
#include "F2MD_msgs/BSM_m.h"
#include "../mdSupport/GeneralLib.h"
#include "../mdSupport/MDMLib.h"
#include "../mdReport/ProtocolEnforcer.h"

class NodeTable {

private:
    int nodesNum;
    NodeHistory nodeHistoryList[MAX_NODES_LENGTH];
    MDMHistory mdmHistoryList[MAX_NODES_LENGTH];
    unsigned long nodePseudos[MAX_NODES_LENGTH];

    NodeHistory nullNode = NodeHistory();
    MDMHistory nullMDMNode = MDMHistory();

    int getOldestNode(ProtocolEnforcer*, ProtocolEnforcer*);

public:
    NodeTable();
    int getNodesNum();
    NodeHistory* getNodeHistoryList();
    unsigned long getNodePseudo(int);
    void put(unsigned long, NodeHistory, MDMHistory, ProtocolEnforcer*, ProtocolEnforcer*);
    NodeHistory* getNodeHistoryAddr(unsigned long nodePseudonym);
    MDMHistory* getMDMHistoryAddr(unsigned long nodePseudonym);

    bool includes(unsigned long nodePseudonym);

    double getDeltaTime(unsigned long nodePseudo1, unsigned long nodePseudo2);

    BSM* getRandomBSM();
    BSM* getNextAttackedBsm( veins::Coord myPosition, unsigned long bsmNode, double bsmTime);
};

#endif
