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

#include "MachineLearningApp.h"

using namespace std;
using namespace boost;

MachineLearningApp::MachineLearningApp(int version, int port, std::string host) :
        MDApplication(version) {
    this->port = port;
    this->host = host;
    httpr = HTTPRequest(port, host);
    bsmPrint = BsmPrintable();
}

void MachineLearningApp::setMyId(inet::MACAddress myId) {
    this->myId = myId;
}

/**
   *  @brief checks if a Node must send a report, based on the results of the previous checks
   *  @param long my pseudonym
   *  @param BSM* BSM received by the node, on which the checks were performed
   *  @param BsmCheck* results of the checks
   *  @result True if a report must be sent, False if not
*/
bool MachineLearningApp::CheckNodeForReport(unsigned long myPseudonym,
        BSM * bsm, BsmCheck * bsmCheck,
        NodeTable * detectedNodes) {

    prntApp->incAll(mbTypes::intMbs[bsm->getSenderMbType()]);
    prntAppInst->incAll(mbTypes::intMbs[bsm->getSenderMbType()]);
    calculateMinFactor(bsmCheck);
    incrementDetailedFlags(bsm,bsmCheck,0.5);

    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(*bsm);
    bsmPrint.setBsmCheck(*bsmCheck);

    std::string s = bsmPrint.getBsmPrintableJson();

    s.erase(std::remove(s.begin(), s.end(), '\t'), s.end());
    s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());

    std::string response = httpr.Request(s);

    //std::cout << "response:" << response << "\n";

    if (!response.compare("True")) {
        prntApp->incCumulFlags(mbTypes::intMbs[bsm->getSenderMbType()]);
        prntAppInst->incCumulFlags(mbTypes::intMbs[bsm->getSenderMbType()]);
        return true;
    }

    return false;
}
/**
   *  @result return the lowest result factor (most critical) of the checks performed
*/
double MachineLearningApp::getMinFactor() {
    return minFactor;
}
