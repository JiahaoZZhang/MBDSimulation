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

#ifndef __VEINS_CpAttackTypes_H_
#define __VEINS_CpAttackTypes_H_

#include <iostream>

namespace cpAttackTypes {

enum Attacks {
    Genuine = 0,
	DropObj,
    DropAllObj,
    AddObj,
	SingleRandomDist,
    SingleRandomSpeed,
    MultiConstDist,
    MultiConstSpeed,
    MultiRandomDist,
    MultiRandomSpeed,
    SingleConstDistOffset,
    SingleConstSpeedOffset,
    SingleRandomDistOffset,
    SingleRandomSpeedOffset,
    MultiConstDistOffset,
    MultiConstSpeedOffset,
    MultiRandomDistOffset,
    MultiRandomSpeedOffset,
    SingleConstDist,
    SingleConstSpeed,
    SIZE_OF_ENUM
};

static const char* AttackNames[] = { "Genuine", "DropObj", "DropAllObj",
        "AddObj", "SingleRandomDist", "SingleRandomSpeed", "MultiConstDist", 
		"MultiConstSpeed", "MultiRandomDist", "MultiRandomSpeed", "SingleConstDistOffset",
        "SingleConstSpeedOffset", "SingleRandomDistOffset", "SingleRandomSpeedOffset", 
		"MultiConstDistOffset", "MultiConstSpeedOffset", "MultiRandomDistOffset", 
		"MultiRandomSpeedOffset","SingleConstDist","SingleConstSpeed"};

static_assert(sizeof(cpAttackTypes::AttackNames)/sizeof(char*) == cpAttackTypes::SIZE_OF_ENUM
        , "sizes dont match");

static const cpAttackTypes::Attacks intAttacks[] = { Genuine, DropObj, 
		DropAllObj, AddObj, SingleRandomDist, SingleRandomSpeed, MultiConstDist,
        MultiConstSpeed, MultiRandomDist, MultiRandomSpeed, SingleConstDistOffset,
        SingleConstSpeedOffset, SingleRandomDistOffset, SingleRandomSpeedOffset, 
		MultiConstDistOffset, MultiConstSpeedOffset, MultiRandomDistOffset, 
		MultiRandomSpeedOffset, SingleConstDist, SingleConstSpeed};

static_assert(sizeof(cpAttackTypes::intAttacks)/sizeof(cpAttackTypes::Attacks) == cpAttackTypes::SIZE_OF_ENUM
        , "sizes dont match");

}

#endif
