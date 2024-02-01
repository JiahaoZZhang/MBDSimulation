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

#ifndef __VEINS_BasicCheckReport_H_
#define __VEINS_BasicCheckReport_H_

#include <omnetpp.h>
#include "F2MD_msgs/BSM_m.h"
#include "MDReport.h"
#include "ReportPrintable.h"
#include "../mdSupport/XmlWriter.h"


using namespace omnetpp;

class BasicCheckReport: public MDReport {

    private:
        BsmCheck reportedCheck;

    public:
        BasicCheckReport(MDReport baseReport);
        void setReportedCheck(BsmCheck reportedCheck);
        std::string getReportPrintableXml();
        std::string getReportPrintableJson();

    };

#endif
