/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    04/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __NodeTypes__
#define __NodeTypes__

#include <iostream>

namespace NodeTypes{
    enum Node{
        Source = 0,
        Object,

        SIZE_OF_ENUM_1
    };

    static const char* NodeNames[] = { "Source","Object" };

    static_assert(sizeof(NodeTypes::NodeNames)/sizeof(char*) == NodeTypes::Node::SIZE_OF_ENUM_1
        , "sizes dont match");

    static const NodeTypes::Node intNode[] = {Source, Object };

    static_assert(sizeof(NodeTypes::intNode)/sizeof(NodeTypes::Node) == NodeTypes::Node::SIZE_OF_ENUM_1
        , "sizes dont match");

    enum Visibility{
        Invisible = 0,
        DirectVisible,
        IndirectVisible,

        SIZE_OF_ENUM_2
    };


    static const char* VisibilityTypes[] = { "Invisible","DirectVisible","IndirectVisible" };

    static_assert(sizeof(NodeTypes::VisibilityTypes)/sizeof(char*) == NodeTypes::Visibility::SIZE_OF_ENUM_2
        , "sizes dont match");

    static const NodeTypes::Visibility intVisibilityTypes[] = { Invisible, DirectVisible, IndirectVisible };

    static_assert(sizeof(NodeTypes::intVisibilityTypes)/sizeof(NodeTypes::Visibility) == NodeTypes::Visibility::SIZE_OF_ENUM_2
        , "sizes dont match");


    enum Validity{
        Invalid = 0,
        Valid,

        SIZE_OF_ENUM_3
    };

    static const char* ValidityType[] = { "Invalid","Valid" };

    static_assert(sizeof(NodeTypes::ValidityType)/sizeof(char*) == NodeTypes::Validity::SIZE_OF_ENUM_3
        , "sizes dont match");

    static const NodeTypes::Validity intValidityTypes[] = { Invalid, Valid };

    static_assert(sizeof(NodeTypes::intValidityTypes)/sizeof(NodeTypes::Validity) == NodeTypes::Validity::SIZE_OF_ENUM_3
        , "sizes dont match");  
}
#endif