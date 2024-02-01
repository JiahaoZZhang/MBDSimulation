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

#ifndef __VEINS_PCPolicy_H_
#define __VEINS_PCPolicy_H_

#include <tuple>
#include <omnetpp.h>
#include "../mdEnumTypes/PseudoChangeTypes.h"
#include "../mdStats/MDStatistics.h"
#include "../mdSupport/GeneralLib.h"
#include "../mdSupport/MDMLib.h"
#include "../../F2MD_CAMParameters.h"
#include "../mdBase/NodeTable.h"
#include "../mdBase/InterTest.h"
#include "../mdBase/BsmCheck.h"
#include "../mdBase/InterTest.h"

#include "../mdEnumTypes/MbTypes.h"

using namespace veins;
using namespace omnetpp;


class PCPolicy {
protected:
    GeneralLib genLib = GeneralLib();
    MDMLib mdmLib = MDMLib();

    veins::Coord* curPosition;
    inet::MACAddress* myId;
    int* pseudoNum;
    unsigned long* myPseudonym;

    long realPseudoNum;
    unsigned long myPseudonymList[MAX_PSEUDO_LIST];

    mbTypes::Mbs mbType;
    MDStatistics* mdAuthority;

    F2MD_CAMParameters * params;

public:
    unsigned long getNextPseudonym();

public:
    PCPolicy();
    PCPolicy(veins::Coord curPos, F2MD_CAMParameters * params);
    void setCurPosition(veins::Coord* curPosition);
    void setMyId(inet::MACAddress* myId);
    void setMyPseudonym(unsigned long* myPseudonym);
    void setPseudoNum(int* pseudoNum);

    void checkPseudonymChange(pseudoChangeTypes::PseudoChange);

    double messageToleranceBuffer = 0;
    void disposablePCP();

    double lastChangeTime = 0;
    void periodicalPCP();

    double cumulativeDistance = 0;
    veins::Coord lastPos = veins::Coord(0, 0, 0);
    void distanceBasedPCP();

    bool firstChange = true;
    bool randDistanceSet = false;
    double randDistance = 800;
    bool randTimeSet = false;
    double randTime = 120;
    double changeTimerStart = 0;
    void car2carPCP();


    void randomPCP();

    void setMbType(mbTypes::Mbs mbType);
    void setMdAuthority(MDStatistics* mdAuthority);

};

#endif
