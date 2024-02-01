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

#ifndef __VEINS_MDReport_H_
#define __VEINS_MDReport_H_

#include <tuple>
#include <omnetpp.h>
#include "veins/veins.h"
#include <time.h>
//#include "veins/base/utils/SimpleAddress.h"
#include "veins/base/utils/Coord.h"
#include <inet/linklayer/ieee80211/mac/Ieee80211Mac.h>

using namespace veins;
using namespace omnetpp;

//#include "../mdBase/BsmCheck.h"
#include <sys/stat.h>
#include "../mdSupport/JsonWriter.h"
#include "../mdSupport/XmlWriter.h"



class MDReport {

protected:
    double generationTime;
    unsigned long senderPseudonym;
    unsigned long reportedPseudo;

    inet::MACAddress senderRealId;
    inet::MACAddress reportedRealId;

    std::string mbType;
    std::string attackType;

    veins::Coord senderGps;
    veins::Coord reportedGps;


public:
    MDReport();

    void setBaseReport(MDReport baseReport);

    void setGenerationTime(double time);
    void setSenderPseudo(unsigned long pseudo);
    void setReportedPseudo(unsigned long pseudo);
    void setMbType(std::string type);
    void setAttackType(std::string type);

    void setSenderGps(veins::Coord senderGps);
    void setReportedGps(veins::Coord reportedGps);

    void setSenderRealId(inet::MACAddress senderRealId);
    void setReportedRealId(inet::MACAddress reportedRealId);

    double getGenerationTime();
    unsigned long getSenderPseudo();
    unsigned long getReportedPseudo();
    std::string getMbType();
    std::string getAttackType();

    veins::Coord getSenderGps();
    veins::Coord getReportedGps();

    inet::MACAddress getSenderRealId();
    inet::MACAddress getReportedRealId();

    std::string getBaseReportXml();
    std::string getBaseReportJson(std::string reportType);
    bool writeStrToFile(const std::string strFileCnst, const std::string serial,
            const std::string version, const std::string outStr,const std::string curDate);
    bool writeStrToFileList(const std::string strFileCnst, const std::string serial,
            const std::string version, const std::string outStr,const std::string curDate);

};

#endif
