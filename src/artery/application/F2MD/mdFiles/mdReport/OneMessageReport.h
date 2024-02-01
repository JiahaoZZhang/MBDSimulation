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

#ifndef __VEINS_OneMessageReport_H_
#define __VEINS_OneMessageReport_H_

#include <omnetpp.h>
#include "F2MD_msgs/BSM_m.h"
#include "../mdReport/MDReport.h"
#include "../mdReport/ReportPrintable.h"
#include "../mdSupport/XmlWriter.h"


using namespace omnetpp;

class OneMessageReport: public MDReport {

    private:
        BsmCheck reportedCheck;
        BSM reportedBsm;

    public:
        OneMessageReport(MDReport baseReport);
        void setReportedCheck(BsmCheck reportedCheck);
        void setReportedBsm(BSM reportedBsm);
        std::string getReportPrintableXml();
        std::string getReportPrintableJson();

    };

#endif
