/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    04/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "NodeTypeClassification.h"

void NodeTypeClassification::NodeTypeClassificationUpdate(const artery::VehicleDataProvider* mVehicleDataProvider, const vanetza::asn1::Cpm& cpm, const LocalEnvironmentModel& lem, NodeTableCpm& TableCpm)
{
    Nodes newSourceNode;
    Nodes newObjectNode;
    bool NSbool_cpm, NObool_cpm;
    int NSindex_cpm, NOindex_cpm;

    bool NSbool_lem, NObool_lem;
    int NSindex_lem, NOindex_lem;

    double temptime = simTime().dbl();

    bool IfInOtherSourcePerception;
    bool IfInLocalPerception;
    int CountRedundancy;

    uint32_t myID = mVehicleDataProvider->getStationId()%100;
    
    auto& PerceivedObjectList = cpm->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects;
    for(int i=0; i < PerceivedObjectList.list.count; i++){
        newObjectNode.NodeID = PerceivedObjectList.list.array[i]->objectID;
        if(newObjectNode.NodeID == myID){
            continue;
        }
        newObjectNode.NodeLastRecord = temptime;
        newObjectNode.NodeType = Object;
        tie(IfInOtherSourcePerception, CountRedundancy) = InOtherSourcePerception(newObjectNode,TableCpm);

        tie(NSbool_cpm,NSindex_cpm) = include(newObjectNode.NodeID, SourceList);
        tie(NObool_cpm,NOindex_cpm) = include(newObjectNode.NodeID, ObjectList);

        if(InLocalPerception(newObjectNode,lem)){
            newObjectNode.NodeVisibility = DirectVisible;
            newObjectNode.NodeValidity = Valid;
        }else if(IfInOtherSourcePerception && (CountRedundancy >= 2 || NSbool_cpm)){
            newObjectNode.NodeVisibility = IndirectVisible;
            newObjectNode.NodeValidity = Valid;
        }else{
            newObjectNode.NodeValidity = Invalid;
            newObjectNode.NodeVisibility = Invisible;
        }

        if(NObool_cpm){
            if(NSbool_cpm){
                auto Niter = ObjectList.begin()+NOindex_cpm;
                this->ObjectList.erase(Niter);
            }else{  
                this->ObjectList[NOindex_cpm] = newObjectNode; 
            }
        }else{
            if(NSbool_cpm){
                newObjectNode.NodeType = Source;
                newObjectNode.NodeLastRecord = temptime;
                this->SourceList[NSindex_cpm] = newObjectNode;
                continue;
            }
            this->ObjectList.push_back(newObjectNode);
        }
    }

    newSourceNode.NodeID = cpm->header.stationID;
    newSourceNode.NodeLastRecord = temptime;
    newSourceNode.NodeType = Source;
    
    tie(IfInOtherSourcePerception, CountRedundancy) = InOtherSourcePerception(newSourceNode,TableCpm);
    IfInLocalPerception = InLocalPerception(newSourceNode,lem);

    if (IfInLocalPerception){
        newSourceNode.NodeValidity = Valid;
        newSourceNode.NodeVisibility = DirectVisible;
    }else if(IfInOtherSourcePerception){
        newSourceNode.NodeValidity = Valid;
        newSourceNode.NodeVisibility = IndirectVisible;
    }else{
        newSourceNode.NodeValidity = Invalid;
        newSourceNode.NodeVisibility = Invisible;
    }

    tie(NSbool_lem,NSindex_lem) = include(newSourceNode.NodeID, SourceList);
    tie(NObool_lem,NOindex_lem) = include(newSourceNode.NodeID, ObjectList);

    if(NSbool_lem){
        this->SourceList[NSindex_lem] = newSourceNode;
    }else{
        if(NObool_lem){
            auto Niter = ObjectList.begin()+NOindex_lem;
            this->ObjectList.erase(Niter);
        }
        if(newObjectNode.NodeID != myID){
            this->SourceList.push_back(newSourceNode);
        }
    }

    NodeList.clear();
    NodeList.shrink_to_fit();
    NodeList.insert(NodeList.end(),SourceList.begin(),SourceList.end());
    NodeList.insert(NodeList.end(),ObjectList.begin(),ObjectList.end());
    
}


void NodeTypeClassification::NodeTypeClassificationUpdate(const Identity* mIdentity, const vanetza::asn1::Cpm& cpm, const LocalEnvironmentModel& lem, NodeTableCpm& TableCpm)
{
    Nodes newSourceNode;
    Nodes newObjectNode;
    bool NSbool_cpm, NObool_cpm;
    int NSindex_cpm, NOindex_cpm;

    bool NSbool_lem, NObool_lem;
    int NSindex_lem, NOindex_lem;

    double temptime = simTime().dbl();

    bool IfInOtherSourcePerception;
    bool IfInLocalPerception;
    int CountRedundancy;

    uint32_t myID = mIdentity->application%100;
    
    auto& PerceivedObjectList = cpm->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects;
    for(int i=0; i < PerceivedObjectList.list.count ; i++){
        newObjectNode.NodeID = PerceivedObjectList.list.array[i]->objectID;
        if(newObjectNode.NodeID == myID){
            continue;
        }
        newObjectNode.NodeLastRecord = temptime;
        newObjectNode.NodeType = Object;
        tie(IfInOtherSourcePerception, CountRedundancy) = InOtherSourcePerception(newObjectNode,TableCpm);

        tie(NSbool_cpm,NSindex_cpm) = include(newObjectNode.NodeID, SourceList);
        tie(NObool_cpm,NOindex_cpm) = include(newObjectNode.NodeID, ObjectList);

        if(InLocalPerception(newObjectNode,lem)){
            newObjectNode.NodeVisibility = DirectVisible;
            newObjectNode.NodeValidity = Valid;
        }else if(IfInOtherSourcePerception && (CountRedundancy >= 2 || NSbool_cpm)){
            newObjectNode.NodeVisibility = IndirectVisible;
            newObjectNode.NodeValidity = Valid;
        }else{
            newObjectNode.NodeValidity = Invalid;
            newObjectNode.NodeVisibility = Invisible;
        }

        if(NObool_cpm){
            if(NSbool_cpm){
                auto Niter = ObjectList.begin()+NOindex_cpm;
                this->ObjectList.erase(Niter);
            }else{  
                this->ObjectList[NOindex_cpm] = newObjectNode; 
            }
        }else{
            if(NSbool_cpm){
                newObjectNode.NodeType = Source;
                newObjectNode.NodeLastRecord = temptime;
                this->SourceList[NSindex_cpm] = newObjectNode;
                continue;
            }
            this->ObjectList.push_back(newObjectNode);
        }
    }

    newSourceNode.NodeID = cpm->header.stationID;
    newSourceNode.NodeLastRecord = temptime;
    newSourceNode.NodeType = Source;
    
    tie(IfInOtherSourcePerception, CountRedundancy) = InOtherSourcePerception(newSourceNode,TableCpm);
    IfInLocalPerception = InLocalPerception(newSourceNode,lem);

    if (IfInLocalPerception){
        newSourceNode.NodeValidity = Valid;
        newSourceNode.NodeVisibility = DirectVisible;
    }else if(IfInOtherSourcePerception){
        newSourceNode.NodeValidity = Valid;
        newSourceNode.NodeVisibility = IndirectVisible;
    }else{
        newSourceNode.NodeValidity = Invalid;
        newSourceNode.NodeVisibility = Invisible;
    }

    tie(NSbool_lem,NSindex_lem) = include(newSourceNode.NodeID, SourceList);
    tie(NObool_lem,NOindex_lem) = include(newSourceNode.NodeID, ObjectList);

    if(NSbool_lem){
        this->SourceList[NSindex_lem] = newSourceNode;
    }else{
        if(NObool_lem){
            auto Niter = ObjectList.begin()+NOindex_lem;
            this->ObjectList.erase(Niter);
        }
        if(newObjectNode.NodeID != myID){
            this->SourceList.push_back(newSourceNode);
        }
    }

    NodeList.clear();
    NodeList.shrink_to_fit();
    NodeList.insert(NodeList.end(),SourceList.begin(),SourceList.end());
    NodeList.insert(NodeList.end(),ObjectList.begin(),ObjectList.end());
    
}


/**
 * @brief get the state list of all nodes
 */
vector<Nodes>& NodeTypeClassification::GetNodeList(){
    return NodeList;
}

/**
 * @brief  get the state list of source nodes(communicated nodes)
 */
vector<Nodes>& NodeTypeClassification::GetSourceList(){
    return SourceList;
}

/**
 * @brief get the state list of object nodes(uncommunicated nodes)
 * 
 */
vector<Nodes>& NodeTypeClassification::GetObjectList(){
    return ObjectList;
}


/** 
 * @brief get the state of a specific node
 */
Nodes NodeTypeClassification::GetNodeState(uint32_t stationID){
    Nodes T;
    for(const auto& nod: NodeList){
        if(nod.NodeID == stationID){
            return nod;
        }
    }
    return T;
}


/**
 * @brief check whether the node is in the list
 * @result tuple<bool include, int Node_Index>
 */
tuple<bool, int> NodeTypeClassification::include(uint32_t stationID, vector<Nodes> NodeList){
    for(const auto& i:NodeList){
        if(i.NodeID == stationID){
            auto IndexList = std::find(NodeList.begin(),NodeList.end(),i);
            int distance = IndexList - NodeList.begin(); 
            return make_tuple(true, distance);
        }
    }
    return make_tuple(false,-1);
}


/**
 * @brief check whether the node is in the local perception
 */
bool NodeTypeClassification::InLocalPerception(Nodes newNode, const LocalEnvironmentModel& lem){
    const auto& localObj = lem.allObjects();
    const TrackedObjectsFilterRange& TrackedBbjs = filterBySensorCategory(localObj, "Radar");
    for(const auto& i: TrackedBbjs){
        std::weak_ptr<EnvironmentModelObject> obj_ptr = i.first;  
        const VehicleDataProvider& vd = obj_ptr.lock()->getVehicleData();   
        if (newNode.NodeID == vd.station_id()%100){
            return true;
        }
        // if (obj_ptr.expired()) continue;
    }
    return false;
}

/**
 * @brief check whether the node is in other sources' perception
 * @return tuple<bool InOtherSourcePerception, int CountRedundancy> 
 */
tuple<bool, int> NodeTypeClassification::InOtherSourcePerception(Nodes newNode,NodeTableCpm& TableCpm){
    bool InOtherPerception = false;
    int CountRedundancy = 0;

    NodeHistoryCpm* NodesHistoryList = TableCpm.getNodeHistoryList();
    int NodeNum = TableCpm.getNodesNum();
    for(int i=0; i < NodeNum; i++){
        vanetza::asn1::Cpm EachNodeHistory = *(NodesHistoryList+i)->getLatestCPMAddr();
        auto& NodePerceivedObjectList = EachNodeHistory->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects;
        for(int g = 0; g < NodePerceivedObjectList.list.count; g++){
            PerceivedObject_t* PerceivedObject = NodePerceivedObjectList.list.array[g];
            if(newNode.NodeID == PerceivedObject->objectID){
                InOtherPerception = true;
                CountRedundancy++;
                break;
            }
        }
    }
    return make_tuple(InOtherPerception,CountRedundancy);
}