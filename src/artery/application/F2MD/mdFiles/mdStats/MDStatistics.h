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

#ifndef __VEINS_MDAuthority_H_
#define __VEINS_MDAuthority_H_

#include "../mdStats/MDSBase.h"
#include "../mdReport/MDReport.h"
#include "../mdEnumTypes/MbTypes.h"

using namespace omnetpp;

#define ABaseNbr 2

class MDStatistics {
private:

    MDSBase baseList[ABaseNbr];

    int baseListNum = 0;
    void treatReport(MDSBase* base, int index, MDReport report);


public:
    MDStatistics();

    void registerNewBase(char* baseName);

    void addNewNode(unsigned long pseudo, mbTypes::Mbs mbType, double time);
    void addReportedNode(unsigned long pseudo, mbTypes::Mbs mbType, double time);
    void getReport(const char* baseName, MDReport report);

    void saveLine(std::string path, std::string serial,double time, bool printOut);

    void resetAll();

};

#endif
