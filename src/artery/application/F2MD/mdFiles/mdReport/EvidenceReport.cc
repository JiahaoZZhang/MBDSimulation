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

#include "EvidenceReport.h"

/**
   *  @brief creates an evidence report
*/
EvidenceReport::EvidenceReport(MDReport baseReport) {
    setBaseReport(baseReport);
    bsmListNum = 0;
}

void EvidenceReport::setReportedCheck(BsmCheck reportedCheck) {
    this->reportedCheck = reportedCheck;
}

/**
   *  @brief adds a BSM to the BSM list of the report
   *  @param BSM bsm to be added
*/
void EvidenceReport::addBsmToList(BSM bsm) {
    this->bsmList[bsmListNum] = bsm;
    bsmListNum++;
}

/**
   *  @brief adds a new evidence to the report
*/
void EvidenceReport::addEvidence(BSM myBsm,
        BsmCheck reportedCheck, BSM receivedBsm,
        NodeTable * detectedNodes) {
    int eviNum = 0;
    bool myBsmAdded = false;
    setReportedCheck(reportedCheck);


    if (reportedCheck.getProximityPlausibility() < 1) {
        addBsmToList(myBsm);
        myBsmAdded=true;
        if (eviNum < 1) {
            eviNum = 1;
        }
    }

    if (reportedCheck.getRangePlausibility() < 1) {
        if(!myBsmAdded){
            addBsmToList(myBsm);
            myBsmAdded=true;
        }
        if (eviNum < 1) {
            eviNum = 1;
        }
    }

    if (reportedCheck.getPositionPlausibility() < 1) {
        if (eviNum < 1) {
            eviNum = 1;
        }
    }

    if (reportedCheck.getSpeedPlausibility() < 1) {
        if (eviNum < 1) {
            eviNum = 1;
        }
    }

    if (reportedCheck.getPositionConsistancy() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getSpeedConsistancy() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getPositionSpeedConsistancy() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getPositionSpeedMaxConsistancy() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getPositionHeadingConsistancy() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getSuddenAppearence() < 1) {
        if (eviNum < 1) {
            eviNum = 1;
        }
    }

    if (reportedCheck.getBeaconFrequency() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getKalmanPACS() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getKalmanPCC() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getKalmanPSCP() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getKalmanPSCS() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getKalmanPSCSP() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getKalmanPSCSS() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getKalmanSCC() < 1) {
        if (eviNum < 2) {
            eviNum = 2;
        }
    }

    if (reportedCheck.getIntersection().getInterNum() > 0) {
        if (eviNum < 1) {
            eviNum = 1;
        }
    }

    if (eviNum > 0) {
        addBsmToList(receivedBsm);
    }

    for (int var = 0; var < eviNum - 1; ++var) {
        addBsmToList(*detectedNodes->getNodeHistoryAddr(reportedPseudo)->getBSMAddr(var));
    }

    for (int var = 0; var < reportedCheck.getIntersection().getInterNum();
            var++) {
        if (reportedCheck.getIntersection().getInterValue(var) < 1) {

            if (senderPseudonym == reportedCheck.getIntersection().getInterId(var)) {
                if(!myBsmAdded){
                    addBsmToList(myBsm);
                }
            } else {
                if(detectedNodes->includes(reportedCheck.getIntersection().getInterId(var))){
                    addBsmToList(
                            *detectedNodes->getNodeHistoryAddr(
                                    reportedCheck.getIntersection().getInterId(var))->getLatestBSMAddr());
                }
            }
        }
    }
}

/**
   *  @result returns an XML version of the report
*/
std::string EvidenceReport::getReportXml() {

    ReportPrintable rp;

    XmlWriter xml;
    xml.init();
    xml.writeHeader();

    std::string tempStr = "Type=\"";
    tempStr = tempStr + "EvidenceReport";
    tempStr = tempStr + "\"";

    xml.writeOpenTagWithAttribute("Report", tempStr);

    xml.writeWholeElement(getBaseReportXml());

    xml.writeWholeElement(rp.getCheckXml(reportedCheck));

    for (int var = 0; var < bsmListNum; ++var) {
        xml.writeWholeElement(rp.getBsmXml(bsmList[var]));
    }

    xml.writeCloseTag();

    return xml.getOutString();
}

/**
   *  @result returns a JSON version of the report
*/
std::string EvidenceReport::getReportPrintableJson() {

    ReportPrintable rp;

    JsonWriter jw;
    jw.writeHeader();
    jw.openJsonElement("Report",false);

    jw.addTagToElement("Report",getBaseReportJson("EvidenceReport"));

    jw.addTagToElement("Report",rp.getCheckJson(reportedCheck));

    jw.openJsonElementList("BSMs");

    for (int var = 0; var < bsmListNum; ++var) {
        if(var < bsmListNum-1){
            jw.addTagToElement("BSMs",rp.getBsmJson(bsmList[var]));
        }else{
            jw.addFinalTagToElement("BSMs",rp.getBsmJson(bsmList[var]));
        }
    }

    jw.addFinalTagToElement("Report",jw.getJsonElementList("BSMs"));
    jw.addElement(jw.getJsonElement("Report"));
    jw.writeFooter();

    return jw.getOutString();
}
