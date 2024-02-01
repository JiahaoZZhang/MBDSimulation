/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    12/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/


#ifndef __ConflictEventTypes__
#define __ConflictEventTypes__

#include <iostream>

namespace ConflictEventTypes{
    enum ConflictType{
        NoConflict =0,
        OI_type, // omission and injection conflict type

        SIZE_OF_ENUM_1
    };

    static const char* ConflictTypeName[] = { "NoConflict","OI_type" };

    static_assert(sizeof(ConflictEventTypes::ConflictTypeName)/sizeof(char*) == ConflictEventTypes::ConflictType::SIZE_OF_ENUM_1
        , "sizes dont match");

    static const ConflictEventTypes::ConflictType intConflictType[] = {NoConflict, OI_type};

    static_assert(sizeof(ConflictEventTypes::intConflictType)/sizeof(ConflictEventTypes::ConflictType) == ConflictEventTypes::ConflictType::SIZE_OF_ENUM_1
        , "sizes dont match");


    enum ConflictEventStatus{
        InProgress =0,
        Stop,

        SIZE_OF_ENUM_2
    };

    static const char* ConflictEventStatusName[] = { "InProgress","Stop" };

    static_assert(sizeof( ConflictEventTypes::ConflictEventStatusName)/sizeof(char*) == ConflictEventTypes::ConflictEventStatus::SIZE_OF_ENUM_2
        , "sizes dont match");

    static const ConflictEventTypes::ConflictEventStatus intConflictEventStatus[] = {InProgress, Stop};

    static_assert(sizeof(ConflictEventTypes::intConflictEventStatus)/sizeof(ConflictEventTypes::ConflictEventStatus) == ConflictEventTypes::ConflictEventStatus::SIZE_OF_ENUM_2
        , "sizes dont match");
}

#endif