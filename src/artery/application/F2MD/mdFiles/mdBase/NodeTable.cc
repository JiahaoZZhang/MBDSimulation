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

#include "NodeTable.h"

/**
   *  @brief  creates a node table containing all the nodes history tables
*/
NodeTable::NodeTable() {
    nodesNum = 0;
}

/**
   *  @result  returns the number of nodes that do have a history table in this node table
*/
int NodeTable::getNodesNum() {
    return nodesNum;
}

/**
   *  @result  returns the list of all the Node History Tables contained in this node table
*/
NodeHistory* NodeTable::getNodeHistoryList() {
    return nodeHistoryList;
}

/**
   *  @result  returns the node pseudo coressponding to the history table located a the given index
*/
unsigned long NodeTable::getNodePseudo(int index) {
    return nodePseudos[index];
}

/**
   *  @result  adds a new node history to this node table
   *  @param long pseudo correspong to the given history
   *  @param NodeHistory node History to be added in the node table
*/
void NodeTable::put(unsigned long pseudo, NodeHistory nodeHistory,
        MDMHistory mdmHistory, ProtocolEnforcer * reportProtocolEnforcerV1,
        ProtocolEnforcer * reportProtocolEnforcerV2) {

    bool included = false;
    int nodeKey;

    for (int var = 0; var < nodesNum; ++var) {
        if (pseudo == nodePseudos[var]) {
            included = true;
            nodeKey = var;
            break;
        }
    }

    if (included) {
        nodeHistoryList[nodeKey] = nodeHistory;
        mdmHistoryList[nodeKey] = mdmHistory;
    } else {
        if (nodesNum < MAX_NODES_LENGTH) {
            nodePseudos[nodesNum] = pseudo;
            nodeHistoryList[nodesNum] = nodeHistory;
            mdmHistoryList[nodesNum] = mdmHistory;
            nodesNum++;
        } else {
            nodeKey = getOldestNode(reportProtocolEnforcerV1,
                    reportProtocolEnforcerV2);
            nodePseudos[nodeKey] = pseudo;
            nodeHistoryList[nodeKey] = nodeHistory;
            mdmHistoryList[nodeKey] = mdmHistory;
        }
    }
}

/**
   *  @result  returns index corresponding to the oldest Node history
*/
int NodeTable::getOldestNode(ProtocolEnforcer * reportProtocolEnforcerV1,
        ProtocolEnforcer * reportProtocolEnforcerV2) {
    int oldestNodeIndex = 0;
    double oldestNodeTime =
            nodeHistoryList[0].getLatestBSMAddr()->getSendingTime().dbl();
    double currentNodeTime = 0;

    double currentPseudo =
            nodeHistoryList[0].getLatestBSMAddr()->getSenderPseudonym();

    for (int var = 0; var < nodesNum; ++var) {
        currentNodeTime =
                nodeHistoryList[var].getLatestBSMAddr()->getSendingTime().dbl();
        currentPseudo =
                nodeHistoryList[var].getLatestBSMAddr()->getSenderPseudonym();

        if ((currentNodeTime < oldestNodeTime)
                && !reportProtocolEnforcerV1->isReported(currentPseudo)
                && !reportProtocolEnforcerV2->isReported(currentPseudo)) {
            oldestNodeTime = currentNodeTime;
            oldestNodeIndex = var;
        }

    }
    return oldestNodeIndex;
}

/**
   *  @result  returns the pointer of the Node History corresponding to the given pseudo
*/
NodeHistory* NodeTable::getNodeHistoryAddr(unsigned long nodePseudo) {
    for (int var = 0; var < nodesNum; ++var) {
        if (nodePseudo == nodePseudos[var]) {
            return &nodeHistoryList[var];
        }
    }
    std::cout<<"nodePseudonym:"<<nodePseudo<<"\n";
    std::cout<<"ERROR: getNodeHistoryAddr no node found \n";
    exit(0);

    return &nullNode;
}
/**
   *  @result  returns the pointer of the Node MDMHistory corresponding to the given pseudo
*/
MDMHistory* NodeTable::getMDMHistoryAddr(unsigned long nodePseudonym) {
    for (int var = 0; var < nodesNum; ++var) {
        if (nodePseudonym == nodePseudos[var]) {
            return &mdmHistoryList[var];
        }
    }
    std::cout<<"nodePseudonym:"<<nodePseudonym<<"\n";
    std::cout<<"ERROR: getMDMHistoryAddr no node found \n";
    exit(0);

    return &nullMDMNode;
}

/**
   *  @result  returns True if this table contains a NodeHistory for this pseudonym, False if not
*/
bool NodeTable::includes(unsigned long nodePseudonym) {
    for (int var = 0; var < nodesNum; ++var) {
        if (nodePseudonym == nodePseudos[var]) {
            return true;
        }
    }
    return false;
}

/**
   *  @result  returns the delta time between the latest History update of the given pseudonyms
*/
double NodeTable::getDeltaTime(unsigned long nodePseudo1,
        unsigned long nodePseudo2) {
    bool includes1 = includes(nodePseudo1);
    bool includes2 = includes(nodePseudo2);
    if(includes1&&includes2){
        return fabs(
                getNodeHistoryAddr(nodePseudo1)->getArrivalTime(0)
                        - getNodeHistoryAddr(nodePseudo2)->getArrivalTime(0));
    }else if(includes1){
        return fabs(getNodeHistoryAddr(nodePseudo1)->getArrivalTime(0));
    }else if(includes2){
        return fabs(getNodeHistoryAddr(nodePseudo2)->getArrivalTime(0));
    }else{
        return 0;
    }
}

BSM* NodeTable::getRandomBSM() {
    GeneralLib genLib = GeneralLib();
    int randNode = genLib.RandomInt(0, nodesNum - 1);
    int randBSM = genLib.RandomInt(0,
            nodeHistoryList[randNode].getBSMNum() - 1);
    return nodeHistoryList[randNode].getBSMAddr(randBSM);
}

BSM* NodeTable::getNextAttackedBsm(veins::Coord myPosition,
        unsigned long bsmNode, double bsmTime) {
    if (bsmNode == 0 || (simTime().dbl() - bsmTime) > 1.1) {
        double minDistance = 10000000;
        int index = -1;
        MDMLib mdmLib = MDMLib();
        for (int var = 0; var < nodesNum; ++var) {
            double distance = mdmLib.calculateDistancePtr(&myPosition,
                    &nodeHistoryList[var].getLatestBSMAddr()->getSenderPos());
            if (minDistance > distance) {
                minDistance = distance;
                index = var;
            }
        }
        return nodeHistoryList[index].getLatestBSMAddr();
    } else {
        return getNodeHistoryAddr(bsmNode)->getLatestBSMAddr();
    }
}

