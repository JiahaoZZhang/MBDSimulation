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

#ifndef __VEINS_ProtocolReport_H_
#define __VEINS_ProtocolReport_H_

#include <omnetpp.h>
#include "F2MD_msgs/BSM_m.h"
#include "../mdReport/MDReport.h"
#include "../mdReport/ReportPrintable.h"
#include "../mdSupport/XmlWriter.h"
#include "../mdSupport/JsonWriter.h"
#include "../mdBase/NodeTable.h"

using namespace omnetpp;

#define MAX_EVI_BSM_PROTO 30

class ProtocolReport: public MDReport {

private:
    void setReportedCheck(BsmCheck reportedCheck);
    void addBsmToList(BSM bsm, BsmCheck check);

    BsmCheck reportedCheck;
    BSM bsmList[MAX_EVI_BSM_PROTO];
    BsmCheck checksList[MAX_EVI_BSM_PROTO];
    int bsmListNum;

public:
    ProtocolReport(MDReport baseReport);
    void addEvidence(BSM myBsm, BsmCheck reportedCheck, BSM receivedBsm,
            NodeTable * detectedNodes, double curTime, double deltaTime, int version);
    std::string getReportPrintableJson();
};

#endif
