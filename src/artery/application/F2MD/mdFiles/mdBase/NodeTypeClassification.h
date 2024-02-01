/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    04/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __NodeTypeClassification__
#define __NodeTypeClassification__

#include "../mdEnumTypes/NodeTypes.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/utility/Identity.h"
#include "NodeTableCpm.h"
#include "NodeHistoryCpm.h"
#include <vanetza/asn1/cpm.hpp>
#include <vector>
#include <tuple>
#include <iterator>

using namespace std;
using namespace NodeTypes;
using namespace veins;
using namespace omnetpp;
using namespace vanetza;
using namespace artery;


struct Nodes
{
    uint32_t NodeID;
    NodeTypes::Node NodeType;
    NodeTypes::Visibility NodeVisibility;
    NodeTypes::Validity NodeValidity;

    double NodeLastRecord;
    bool operator == (const Nodes& e){
        return (this->NodeID == e.NodeID) && (this->NodeLastRecord == e.NodeLastRecord) && (this->NodeType == e.NodeType) && (this->NodeValidity == e.NodeValidity) && (this->NodeVisibility == e.NodeVisibility);
    }
    Nodes& operator = (const Nodes& e){
        this->NodeID = e.NodeID;
        this->NodeLastRecord = e.NodeLastRecord;
        this->NodeType = e.NodeType;
        this->NodeValidity = e.NodeValidity;
        this->NodeVisibility = e.NodeVisibility;
        return *this;
    }
};


class NodeTypeClassification
{
    private:
        vector<Nodes> NodeList;
        vector<Nodes> SourceList;
        vector<Nodes> ObjectList;
        
    public:
        NodeTypeClassification(){NodeList.clear();SourceList.clear();ObjectList.clear();};
        void NodeTypeClassificationUpdate(const artery::VehicleDataProvider* mVehicleDataProvider, const vanetza::asn1::Cpm& cpm,
                const LocalEnvironmentModel& lem, NodeTableCpm& TableCpm);
        
        void NodeTypeClassificationUpdate(const Identity* mIdentity, const vanetza::asn1::Cpm& cpm,
                const LocalEnvironmentModel& lem, NodeTableCpm& TableCpm);

        vector<Nodes>& GetNodeList();
        vector<Nodes>& GetSourceList();
        vector<Nodes>& GetObjectList();
        Nodes GetNodeState(uint32_t stationID);

        ~NodeTypeClassification(){};
    
    protected:
        tuple<bool,int> include(uint32_t stationID, vector<Nodes> NodeList);
        bool InLocalPerception(Nodes newNode, const LocalEnvironmentModel& lem);
        tuple<bool, int> InOtherSourcePerception(Nodes newNode,NodeTableCpm& TableCpm);

        
};




#endif