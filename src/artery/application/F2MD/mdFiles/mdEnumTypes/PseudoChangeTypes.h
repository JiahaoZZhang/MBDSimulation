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

#ifndef __VEINS_PseudoChangeTypes_H_
#define __VEINS_PseudoChangeTypes_H_

#include <iostream>

namespace pseudoChangeTypes {

enum PseudoChange {
    NoChange = 0,
    Periodical,
    Disposable,
    DistanceBased,
    Random,
    Car2car,
    SIZE_OF_ENUM
};

static const char* PseudoChangeNames[] = { "NoChange", "Periodical",
        "Disposable", "DistanceBased", "Random", "Car2car" };

static_assert(sizeof(pseudoChangeTypes::PseudoChangeNames)/sizeof(char*) == pseudoChangeTypes::SIZE_OF_ENUM
        , "sizes dont match");

static const pseudoChangeTypes::PseudoChange intPseudoChange[] = { NoChange,
        Periodical, Disposable, DistanceBased, Random, Car2car };

static_assert(sizeof(pseudoChangeTypes::intPseudoChange)/sizeof(pseudoChangeTypes::PseudoChange) == pseudoChangeTypes::SIZE_OF_ENUM
        , "sizes dont match");

}

#endif
