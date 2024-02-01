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

#include "BasicCheckReport.h"

/**
   *  @brief creates a basic check report
*/
BasicCheckReport::BasicCheckReport(MDReport baseReport) {
    setBaseReport(baseReport);
}
void BasicCheckReport::setReportedCheck(BsmCheck reportedCheck) {
    this->reportedCheck = reportedCheck;
}

/**
   *  @result returns an XML printable version of the report
*/
std::string BasicCheckReport::getReportPrintableXml() {

    ReportPrintable rp;

    XmlWriter xml;
    xml.init();
    xml.writeHeader();

    std::string tempStr = "Type=\"";
    tempStr = tempStr + "BasicCheckReport";
    tempStr = tempStr + "\"";
    xml.writeOpenTagWithAttribute("Report", tempStr);

    xml.writeWholeElement(getBaseReportXml());
    xml.writeWholeElement(rp.getCheckXml(reportedCheck));

    xml.writeCloseTag();

    return xml.getOutString();
}

/**
   *  @result returns a JSON printable version of the report
*/
std::string BasicCheckReport::getReportPrintableJson() {
    ReportPrintable rp;

    JsonWriter jw;
    jw.writeHeader();
    jw.openJsonElement("Report",false);
    jw.addTagToElement("Report",getBaseReportJson("BasicCheckReport"));
    jw.addFinalTagToElement("Report",rp.getCheckJson(reportedCheck));

    jw.addElement(jw.getJsonElement("Report"));
    jw.writeFooter();

    return jw.getOutString();
}


