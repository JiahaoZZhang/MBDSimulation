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

#ifndef __VEINS_ReportTypes_H_
#define __VEINS_ReportTypes_H_

#include <iostream>

namespace reportTypes {

enum Report {
    OneMessageReport = 0,
    EvidenceReport,
    ProtocolReport,
    BasicCheckReport,
    SIZE_OF_ENUM
};

static const char* ReportNames[] = { "OneMessageReport", "EvidenceReport", "ProtocolReport", "BasicCheckReport"};

static_assert(sizeof(reportTypes::ReportNames)/sizeof(char*) == reportTypes::SIZE_OF_ENUM
        , "sizes dont match");

static const reportTypes::Report intReport[] = {OneMessageReport,EvidenceReport,ProtocolReport,BasicCheckReport};

static_assert(sizeof(reportTypes::intReport)/sizeof(reportTypes::Report) == reportTypes::SIZE_OF_ENUM
        , "sizes dont match");

}
#endif
