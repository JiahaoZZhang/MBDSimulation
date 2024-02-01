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

#ifndef __VEINS_MdAppTypes_H_
#define __VEINS_MdAppTypes_H_

#include <iostream>

namespace mdAppTypes {

enum App {
    ThresholdApp = 0,
    AggrigationApp,
    BehavioralApp,
    CooperativeApp,
    ExperiApp,
    MachineLearningApp,

    SIZE_OF_ENUM
};

static const char* AppNames[] = { "ThresholdApp", "AggrigationApp",
        "CooperativeApp", "BehavioralApp", "ExperiApp", "MachineLearningApp" };

static_assert(sizeof(mdAppTypes::AppNames)/sizeof(char*) == mdAppTypes::SIZE_OF_ENUM
        , "sizes dont match");

static const mdAppTypes::App intApp[] = { ThresholdApp, AggrigationApp,
        BehavioralApp, CooperativeApp, ExperiApp, MachineLearningApp };

static_assert(sizeof(mdAppTypes::intApp)/sizeof(mdAppTypes::App) == mdAppTypes::SIZE_OF_ENUM
        , "sizes dont match");

}

#endif
