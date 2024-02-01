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

#ifndef __VEINS_BsmPrintable_H_
#define __VEINS_BsmPrintable_H_

#include <omnetpp.h>
#include "../mdEnumTypes/AttackTypes.h"
#include "F2MD_msgs/BSM_m.h"
#include "../mdReport/MDReport.h"
#include "../mdReport/ReportPrintable.h"
#include "../mdSupport/XmlWriter.h"
#include "../mdSupport/JsonWriter.h"
#include "../mdEnumTypes/MbTypes.h"

using namespace omnetpp;
using namespace veins;

class BsmPrintable {

private:
    BsmCheck bsmCheck;
    BSM bsm;
    inet::MACAddress receiverId;
    unsigned long receiverPseudo;

    std::string getBsmPrintHead();

    std::string getSelfBsmPrintHead(std::string myVType);

public:

    BsmPrintable();
    void setBsmCheck(BsmCheck check);
    void setBsm(BSM bsm);
    void setReceiverId(inet::MACAddress receiverId);
    void setReceiverPseudo(unsigned long receiverPseudo);


    std::string getBsmPrintableXml();
    std::string getBsmPrintableJson();

    bool writeStrToFile(const std::string strFileCnst, const std::string serial,
            const std::string version, const std::string outStr,
            const std::string curDate);


    std::string getSelfBsmPrintableJson(std::string myVType);

    bool writeSelfStrToFile(const std::string strFileCnst,
            const std::string serial, const std::string outStr,
            const std::string curDate);

    bool writeSelfStrToFileList(const std::string strFileCnst,
            const std::string serial, const std::string outStr,
            const std::string curDate);

    bool writeStrToFileList(const std::string strFileCnst,
            const std::string serial, const std::string version,
            const std::string outStr, const std::string curDate);

};

#endif
