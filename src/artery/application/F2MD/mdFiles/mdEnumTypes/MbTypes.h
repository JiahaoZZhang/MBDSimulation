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

#ifndef __VEINS_mbTypes_H_
#define __VEINS_mbTypes_H_

#include <iostream>

namespace mbTypes {

enum Mbs {
    Genuine = 0,
    LocalAttacker,
    GlobalAttacker,
    SIZE_OF_ENUM
};

static const char* mbNames[] = { "Genuine", "LocalAttacker","GlobalAttacker"};

static const Mbs intMbs[] = { Genuine, LocalAttacker,GlobalAttacker };

static_assert(sizeof(mbTypes::mbNames)/sizeof(char*) == mbTypes::SIZE_OF_ENUM
        , "sizes dont match");

static_assert(sizeof(mbTypes::intMbs)/sizeof(Mbs) == mbTypes::SIZE_OF_ENUM
        , "sizes dont match");


}

#endif
