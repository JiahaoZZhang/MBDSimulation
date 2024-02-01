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

#include "OneMessageReport.h"

/**
   *  @brief creates a OneMessage report
*/
OneMessageReport::OneMessageReport(MDReport baseReport) {
    setBaseReport(baseReport);
}
void OneMessageReport::setReportedCheck(BsmCheck reportedCheck) {
    this->reportedCheck = reportedCheck;
}

void OneMessageReport::setReportedBsm(BSM reportedBsm) {
    this->reportedBsm = reportedBsm;
}
/**
   *  @result returns an XML version of the report
*/
std::string OneMessageReport::getReportPrintableXml() {

    ReportPrintable rp;

    XmlWriter xml;
    xml.init();
    xml.writeHeader();

    std::string tempStr = "Type=\"";
    tempStr = tempStr + "OneMessageReport";
    tempStr = tempStr + "\"";
    xml.writeOpenTagWithAttribute("Report", tempStr);

    xml.writeWholeElement(getBaseReportXml());
    xml.writeWholeElement(rp.getCheckXml(reportedCheck));

    xml.writeWholeElement(rp.getBsmXml(reportedBsm));

    xml.writeCloseTag();

    return xml.getOutString();
}
/**
   *  @result returns a JSON version of the report
*/
std::string OneMessageReport::getReportPrintableJson() {
    ReportPrintable rp;

    JsonWriter jw;
    jw.writeHeader();
    jw.openJsonElement("Report",false);
    jw.addTagToElement("Report",getBaseReportJson("OneMessageReport"));
    jw.addTagToElement("Report",rp.getCheckJson(reportedCheck));

    jw.openJsonElementList("BSMs");
    jw.addFinalTagToElement("BSMs",rp.getBsmJson(reportedBsm));
    jw.addFinalTagToElement("Report",jw.getJsonElementList("BSMs"));
    jw.addElement(jw.getJsonElement("Report"));
    jw.writeFooter();

    return jw.getOutString();
}


