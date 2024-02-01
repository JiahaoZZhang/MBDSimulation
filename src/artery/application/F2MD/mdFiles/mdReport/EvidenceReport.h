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

#ifndef __VEINS_EvidenceReport_H_
#define __VEINS_EvidenceReport_H_

#include <omnetpp.h>
#include "F2MD_msgs/BSM_m.h"
#include "../mdReport/MDReport.h"
#include "../mdReport/ReportPrintable.h"
#include "../mdSupport/XmlWriter.h"
#include "../mdSupport/JsonWriter.h"
#include "../mdBase/NodeTable.h"

using namespace omnetpp;

#define MAX_EVI_BSM 30

class EvidenceReport: public MDReport {

private:
    void setReportedCheck(BsmCheck reportedCheck);
    void addBsmToList(BSM bsm);

    BsmCheck reportedCheck;
    BSM bsmList[MAX_EVI_BSM];
    int bsmListNum;

public:
    EvidenceReport(MDReport baseReport);
    void addEvidence(BSM myBsm, BsmCheck reportedCheck, BSM receivedBsm,
            NodeTable * detectedNodes);
    std::string getReportXml();
    std::string getReportPrintableJson();
};

#endif
