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

#include "NodeHistoryCpm.h"

/**
   *  @brief  creates a node History that will save for a defined period all the CPM received from a node to perform checks based on history
*/
NodeHistoryCpm::NodeHistoryCpm() {
    nodeID = 0;
    cpmNum = 0;
    lastUpdate = simTime().dbl();
    }

/**
   *  @brief  creates a node History that will save for a defined period all the CPM received from a node to perform checks based on history
*/
NodeHistoryCpm::NodeHistoryCpm(int ID) {
    nodeID = ID;
    cpmNum = 0;
    lastUpdate = simTime().dbl();
}

/**
   *  @brief  creates a node History that will save for a defined period all the CPM received from a node to perform checks based on history. Also adds the given CPM to the new history table
*/
NodeHistoryCpm::NodeHistoryCpm(int ID, vanetza::asn1::Cpm cpm) {
    nodeID = ID;
    cpmNum = 0;
    lastUpdate = simTime().dbl();
    addCPM(cpm);
}

/**
   *  @brief  return a pointer to the latest CPM sent by this node.
   *  @return pointer to the CPMobject.
*/
vanetza::asn1::Cpm* NodeHistoryCpm::getLatestCPMAddr() {
    return &cpmList[0];  
}

/**
   *  @brief  return a pointer to the CPM located at this index on the history list.
   *  @return pointer to the CPM object.
*/
vanetza::asn1::Cpm* NodeHistoryCpm::getCPMAddr(int index) {
    return &cpmList[index];
}

/**
   *  @brief  get the number of CPM stored in the historyList.
   *  @return number of CPM.
*/
int NodeHistoryCpm::getCPMNum() {
    return cpmNum;
}

/**
   *  @brief  return the last time when the history list was updated.
*/
double NodeHistoryCpm::getLastUpdate() {
    return lastUpdate;
}

int NodeHistoryCpm::getNodeID() {
    return nodeID;
}

/**
   *  @brief  add a CPM to the history list.
   *  @param  vanetzaAsn1Cpm cpm to add in the table
*/
void NodeHistoryCpm::addCPM(vanetza::asn1::Cpm cpm) {
    if (cpmNum < MAX_CPM_LENGTH) {
        cpmNum++;
    }
    for (int var = cpmNum - 1; var > 0; --var) {
        cpmList[var] = cpmList[var - 1];
    }
    cpmList[0] = cpm;
    lastUpdate = simTime().dbl();
}
