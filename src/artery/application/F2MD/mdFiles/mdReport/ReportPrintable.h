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

#ifndef __VEINS_ReportPrintable_H_
#define __VEINS_ReportPrintable_H_

#include <omnetpp.h>
#include "F2MD_msgs/BSM_m.h"
#include "../mdReport/MDReport.h"
#include "../mdSupport/XmlWriter.h"
#include "../mdSupport/JsonWriter.h"
#include "../mdBase/BsmCheck.h"

using namespace omnetpp;
using namespace veins;

class ReportPrintable {

private:

public:


    std::string getCheckXml(BsmCheck Check);
    std::string getBsmXml(BSM bsm);

    std::string getCheckJson(BsmCheck Check);
    std::string getCheckJsonList(BsmCheck Check);
    std::string getBsmJson(BSM bsm);

};

#endif
