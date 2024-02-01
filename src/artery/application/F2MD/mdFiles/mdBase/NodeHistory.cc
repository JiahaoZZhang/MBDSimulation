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

#include "NodeHistory.h"

/**
   *  @brief  creates a node History that will save for a defined period all the BSM received from a node to perform checks based on history
*/
NodeHistory::NodeHistory() {
    nodePseudonym = 0;
    bsmNum = 0;
}

/**
   *  @brief  creates a node History that will save for a defined period all the BSM received from a node to perform checks based on history
*/
NodeHistory::NodeHistory(unsigned long pseudo) {
    nodePseudonym = pseudo;
    bsmNum = 0;
}

/**
   *  @brief  creates a node History that will save for a defined period all the BSM received from a node to perform checks based on history. Also adds the given BSM to the new history
*/
NodeHistory::NodeHistory(unsigned long pseudo, BSM bsm) {
    nodePseudonym = pseudo;
    bsmNum = 0;
    addBSM(bsm);
}

/**
   *  @result  returns the pointer of the latest received BSM contained in the history table
*/
BSM* NodeHistory::getLatestBSMAddr() {
    return &bsmList[0];
}

/**
   *  @result  returns the pointer of the received BSM corresponding to the given index
*/
BSM* NodeHistory::getBSMAddr(int index) {
    return &bsmList[index];
}

/**
   *  @result  returns the arrival time of the received BSM corresponding to the given index
*/
double NodeHistory::getArrivalTime(int index) {
    return bsmList[index].getArrivalTime().dbl();
}

/**
   *  @result  returns the sender position of the received BSM corresponding to the given index
*/
veins::Coord NodeHistory::getSenderPos(int index) {
    return bsmList[index].getSenderPos();
}

/**
   *  @result  returns the sender size of the received BSM corresponding to the given index
*/
veins::Coord NodeHistory::getSenderSize(int index) {
    return veins::Coord(bsmList[index].getSenderWidth(),
            bsmList[index].getSenderLength());
}

/**
   *  @result  returns the sender speed of the received BSM corresponding to the given index
*/
double NodeHistory::getSenderSpeed(int index) {
    return sqrt(
            pow(bsmList[index].getSenderSpeed().x, 2.0)
                    + pow(bsmList[index].getSenderSpeed().y, 2.0)
                    + pow(bsmList[index].getSenderSpeed().z, 2.0));
}

/**
   *  @result  returns the sender heading of the received BSM corresponding to the given index
*/
veins::Coord NodeHistory::getSenderHeading(int index) {
    return bsmList[index].getSenderHeading();
}

/**
   *  @result  returns the delta time between the received BSMs corresponding to the given indexes
*/
double NodeHistory::getDeltaTime(int index1, int index2) {
    return fabs(
            bsmList[index1].getArrivalTime().dbl()
                    - bsmList[index2].getArrivalTime().dbl());
}

/**
   *  @result  returns the number of BSMs contained in the history table
*/
int NodeHistory::getBSMNum() {
    return bsmNum;
}

/**
   *  @brief Adds a new BSM to the history table
   *  @param BSM Bsm to add in the table
*/
void NodeHistory::addBSM(BSM bsm) {
    if (bsmNum < MAX_BSM_LENGTH) {
        bsmNum++;
    }
    for (int var = bsmNum - 1; var > 0; --var) {
        bsmList[var] = bsmList[var - 1];
    }
    bsmList[0] = bsm;
}
