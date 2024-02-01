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

#ifndef __VEINS_MdChecksTypes_H_
#define __VEINS_MdChecksTypes_H_

#include <iostream>

namespace mdChecksTypes {

enum Checks {
    ProximityPlausibility = 0,
    RangePlausibility,
    PositionPlausibility,
    SpeedPlausibility,
    PositionConsistancy,
    PositionSpeedConsistancy,
    PositionSpeedMaxConsistancy,
    SpeedConsistancy,
    BeaconFrequency,
    Intersection,
    SuddenAppearence,
    PositionHeadingConsistancy,
    kalmanPSCP,
    kalmanPSCS,
    kalmanPSCSP,
    kalmanPSCSS,
    kalmanPCC,
    kalmanPACS,
    kalmanSCC,
    SIZE_OF_ENUM
};

static const char* ChecksNames[] = {
    "ProximityPlausibility",
    "RangePlausibility",
    "PositionPlausibility",
    "SpeedPlausibility",
    "PositionConsistancy",
    "PositionSpeedConsistancy",
    "PositionSpeedMaxConsistancy",
    "SpeedConsistancy",
    "BeaconFrequency",
    "Intersection",
    "SuddenAppearence",
    "PositionHeadingConsistancy",
    "kalmanPSCP",
    "kalmanPSCS",
    "kalmanPSCSP",
    "kalmanPSCSS",
    "kalmanPCC",
    "kalmanPACS",
    "kalmanSCC",
};

static_assert(sizeof(mdChecksTypes::ChecksNames) / sizeof(char*) == mdChecksTypes::SIZE_OF_ENUM, "sizes dont match");

} // namespace mdChecksTypes

#endif
