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
# Table used to store all the NodeHistory lists
# Each node owns one NodeTable
*/

#ifndef __VEINS_NODETABLECPM_H_
#define __VEINS_NODETABLECPM_H_

#include "NodeHistoryCpm.h"
#include <vanetza/asn1/cpm.hpp>
#include "../mdSupport/GeneralLib.h"
#include "../mdSupport/MDMLib.h"

class NodeTableCpm {

private:
    int nodesNum;
    NodeHistoryCpm nodeHistoryList[MAX_NODES_LENGTH];
    //MDMHistory mdmHistoryList[MAX_NODES_LENGTH];
    int nodeIDs[MAX_NODES_LENGTH];

    NodeHistoryCpm nullNode = NodeHistoryCpm();
    //MDMHistory nullMDMNode = MDMHistory();

    int getOldestNode();

public:
    NodeTableCpm();
    int getNodesNum();
    NodeHistoryCpm* getNodeHistoryList();
    int getNodeID(int);
    void put(int, NodeHistoryCpm, double);
    NodeHistoryCpm* getNodeHistoryAddr(int nodeID);
    //MDMHistory* getMDMHistoryAddr(unsigned long nodePseudonym);

    bool includes(int nodeID);
};

#endif
