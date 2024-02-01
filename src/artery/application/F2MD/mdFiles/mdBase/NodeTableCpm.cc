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

#include "NodeTableCpm.h"

/**
   *  @brief  creates a node table containing all the nodes history tables
*/
NodeTableCpm::NodeTableCpm() {
    nodesNum = 0;
}

/**
   *  @result  returns the number of nodes that do have a history table in this node table
*/
int NodeTableCpm::getNodesNum() {
    return nodesNum;
}

/**
   *  @result  returns the list of all the Node History Tables contained in this node table
*/
NodeHistoryCpm* NodeTableCpm::getNodeHistoryList() {
    return nodeHistoryList;
}

/**
   *  @brief  get the ID of the node stored at the given index.
*/
int NodeTableCpm::getNodeID(int index) {
    return nodeIDs[index];
}

/**
   *  @brief  adds a node history corresponding to the given NodeID.
   *  @param  int NodeID
   *  @param  NodeHistoryCpm Node History that must be added
*/
void NodeTableCpm::put(int ID, NodeHistoryCpm nodeHistory, double maxHistoryTime) {

    bool included = false;
    int nodeKey;

    // check if the node is already included in the table
    for (int var = 0; var < nodesNum; ++var) {
        if (ID == nodeIDs[var]) {
            included = true;
            nodeKey = var;
            break;
        }
    }

    if (included) {
        nodeHistoryList[nodeKey] = nodeHistory;
    } else {
        nodeKey = getOldestNode();
        // If the oldest updated node in the history passed the maximum delay allowed, simply replace it
        if (fabs(simTime().dbl() - nodeHistoryList[nodeKey].getLastUpdate()) > maxHistoryTime){
            nodeIDs[nodeKey] = ID;
            nodeHistoryList[nodeKey] = nodeHistory;
        }
        else if (nodesNum < MAX_NODES_LENGTH) {
            nodeIDs[nodesNum] = ID;
            nodeHistoryList[nodesNum] = nodeHistory;
            nodesNum++;
        } else {
            nodeIDs[nodeKey] = ID;
            nodeHistoryList[nodeKey] = nodeHistory;
        }
    }
}

/**
   *  @brief  get the index of the oldest history list.
*/
int NodeTableCpm::getOldestNode() {
    int oldestNodeIndex = 0;
    double oldestNodeTime =
            nodeHistoryList[0].getLastUpdate();
    double currentNodeTime = 0;

    auto currentID =
            nodeHistoryList[0].getNodeID();

    for (int var = 0; var < nodesNum; ++var) {
        currentNodeTime =
                nodeHistoryList[var].getLastUpdate();
        currentID =
                nodeHistoryList[var].getNodeID();

        if (currentNodeTime < oldestNodeTime) {
            oldestNodeTime = currentNodeTime;
            oldestNodeIndex = var;
        }

    }
    return oldestNodeIndex;
}

/**
   *  @result  returns the pointer of the Node History corresponding to the given pseudo
*/
NodeHistoryCpm* NodeTableCpm::getNodeHistoryAddr(int nodeID) {
    for (int var = 0; var < nodesNum; ++var) {
        if (nodeID == nodeIDs[var]) {
            return &nodeHistoryList[var];
        }
    }
    // std::cout<<"nodePseudonym:"<<nodeID<<"\n";
    // std::cout<<"ERROR: getNodeHistoryAddr no node found \n";
    // exit(0);

    return &nullNode;
}

/**
   *  @result  returns True if this table contains a NodeHistory for this pseudonym, False if not
*/
bool NodeTableCpm::includes(int nodeID) {
    for (int var = 0; var < nodesNum; ++var) {
        if (nodeID == nodeIDs[var]) {
            return true;
        }
    }
    return false;
}

