// /*******************************************************************************
//  * @author  Jiahao ZHANG
//  * @email   jiahao.zhang96@gmail.com
//  * @date    24/08/2022
//  *
//  * TAM (Trusted Autonomous Mobility)
//  * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
//  * All rights reserved.
//  *******************************************************************************/

#include "F2MD_RsuCPFacility.h"

F2MD_RsuCPFacility::F2MD_RsuCPFacility(std::shared_ptr<const traci::API>* TraciLiteAPI, F2MD_CPParameters& CPparams){
    mLiteAPI = TraciLiteAPI;
    params = &CPparams;
}

void F2MD_RsuCPFacility::initialize(){
    detectedNodes = NodeTableCpm();

    // ------ Initialize F2MD  ------
    // ------ Initialize detectors ----
    mCpmChecks = CpmChecks(params);
}


double F2MD_RsuCPFacility::onCPM(const vanetza::asn1::Cpm& msg, CpmCheckList& mCpmCheckList){

    vanetza::asn1::Cpm lastCpm;
    int senderID = msg->header.stationId;

    // run checks
    double checkfailed = mCpmChecks.CpmChecksum(msg, &detectedNodes, mCpmCheckList, mLiteAPI);

// ------------------------------------------------------------------------------------------------------
    // add the cpm received to the HistoryNodeTable
    if (!detectedNodes.includes(senderID)) {
        NodeHistoryCpm newNode(senderID);
        newNode.addCPM(msg);
        detectedNodes.put(senderID, newNode, params->MAX_CPM_HISTORY_TIME);
    }
    else {
        NodeHistoryCpm* existingNode = detectedNodes.getNodeHistoryAddr(
            senderID);

        lastCpm = *(existingNode->getLatestCPMAddr());
        existingNode->addCPM(msg);
    }

// ----------------------------------------------------------------------------------------------------
    // if (checkfailed > 0) {
    //     addAccusedNode(senderID);
    // }

    // treatAttackFlags(senderID);

    return checkfailed;
}



NodeTableCpm& F2MD_RsuCPFacility::getNodeTableCpm(){
    return this->detectedNodes;
};
