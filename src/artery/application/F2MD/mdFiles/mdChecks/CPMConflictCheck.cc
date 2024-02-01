/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    30/05/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "CPMConflictCheck.h"

ConflictEventTable& CPMConflictCheck::getConflictEventTable(){
    return this->mConflictEventTable;
} 

tuple<bool,int> CPMConflictCheck::IncludeEvent(int ConflictObjectID){
    for(auto& i:this->mConflictEventTable.getConflictEventInfoTable()){
        if(i.EventInfo.getConflictObjectID() == ConflictObjectID){
            return make_tuple(true,i.ConflictEventID);
        }
    }
    return make_tuple(false,-1);
}

// void CPMConflictCheck::updateConflictCheck(NodeTableCpm& TableCpm){
//     NodeHistoryCpm* NodesHistoryList = TableCpm.getNodeHistoryList();
//     int NodesNum = TableCpm.getNodesNum();
//     for(int i=0; i < NodesNum; i++){
//         vanetza::asn1::Cpm EachLastestCpm = *(NodesHistoryList+i)->getLatestCPMAddr();
//         PerceivedObjectContainer_t* NodePerceivedObjectList = EachLastestCpm->cpm.cpmParameters.perceivedObjectContainer;
//         StationID_t NodesID = EachLastestCpm->header.stationID;
//         for(int g = 0; g < NodePerceivedObjectList->list.count; g++){
//             PerceivedObject_t* PerceivedObject = NodePerceivedObjectList->list.array[g];
//             int PerceivedObjectID = PerceivedObject->objectID;
//             if( PerceivedObjectID == 1){
//                 bool include;
//                 int ConflictEventID;
//                 tie(include,ConflictEventID) = IncludeEvent(1);
//                 if(include){
//                     mConflictEventTable.getConflictEventInfoTable(ConflictEventID).EventInfo.addConflictSourceAssociated(NodesID);
//                 }else{
//                     vector<uint32_t> ConflictSourceAssociated = {uint32_t(NodesID),26};
//                     ConflictEventInfo newConflict(1, OI_type, ConflictSourceAssociated);
//                     mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
//                 }
//             }
//         }
//     }
// }



/**
 * @brief specific setting of omission attack senario
 */
void CPMConflictCheck::updateConflictCheckO(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,const VehicleDataProvider* vdp, NodeTrustTable& NTT, NodeTableCpm& TableCpm, NodeTypeClassification& NTC){
    bool include;
    bool inNTT;
    int ConflictEventID;
    int NTTdis;
    uint32_t stationID = msg->header.stationID;
    uint32_t egoID = vdp->getStationId()%100;
    bool conflictInclude;


    uint32_t station1 = 1;
    uint32_t station39 = 39;
    uint32_t station26 = 26;
    uint32_t station63 = 63;
    uint32_t station69 = 69;

    auto NTTtable = NTT.getTrustTable();
    for(int i = 0 ; i < NTTtable.size(); i++){
        auto tempStationID = NTTtable[i].stationID;
        double tempValidity = NTC.GetNodeState(tempStationID).NodeValidity;
        NTT.getNodeTrust(tempStationID)->NodeType =  tempValidity;
    }


    if(TableCpm.includes(station26)){
        vanetza::asn1::Cpm& AttackerLastCPM = *(TableCpm.getNodeHistoryAddr(station26))->getLatestCPMAddr();
        PerceivedObjectContainer_t& addr_NodePerceivedObjectList = AttackerLastCPM->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer;
        PerceivedObjectContainer_t* NodePerceivedObjectList = &addr_NodePerceivedObjectList;
        for(int i=0; i< NodePerceivedObjectList->perceivedObjects.list.count; i++){
            cout << "Attacker Perceived Station ID: " << NodePerceivedObjectList->perceivedObjects.list.array[i]->objectID << endl;
            if(NodePerceivedObjectList->perceivedObjects.list.array[i]->objectID == station1){
                conflictInclude = false;
                break;
            }
        }
        if (NodePerceivedObjectList->perceivedObjects.list.count < 1){
            conflictInclude = true;
            tie(inNTT,NTTdis) = NTT.include(station26);
            if(inNTT){
                NTT.getNodeTrust(station26)->NodeType = 0;
            }
        }
    }

    if(msg->cpm.cpmParameters.perceptionData->list.count >0){
        PerceivedObjectContainer& addr_objPointer = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer;
        PerceivedObjectContainer* objPointer = &addr_objPointer;
        for(int i=0 ; i < objPointer->perceivedObjects.list.count; i++){
            PerceivedObject_t *po = objPointer->perceivedObjects.list.array[i];
            if((po->objectID == station1) && (conflictInclude == true)){
                tie(include,ConflictEventID) = IncludeEvent(station1);
                if(include){
                    if(!mConflictEventTable.cTable[ConflictEventID].EventInfo.include(stationID)){ 
                            mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(stationID);
                        if(stationID != station39){  
                            xt::xarray<double> temp = NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice() * 0.9;
                            double temp2 = 1 - xt::sum(temp)();
                            NTT.getNodeTrust(stationID)->TrustOpinion.SetAdvice(temp);
                            NTT.getNodeTrust(stationID)->TrustOpinion.SetU(temp2);
                        }
                    }
                }else{
                    if(stationID == station26){
                        continue;
                    }
                    vector<uint32_t> ConflictSourceAssociated = {stationID,station26};
                    ConflictEventInfo newConflict(station1, OI_type, ConflictSourceAssociated);
                    mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
                    xt::xarray<double> temp = NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice() * 0.9;
                    double temp2 = 1 - xt::sum(temp)();
                    NTT.getNodeTrust(stationID)->TrustOpinion.SetAdvice(temp);
                    NTT.getNodeTrust(stationID)->TrustOpinion.SetU(temp2);
                    xt::xarray<double> temp3 = NTT.getNodeTrust(station26)->TrustOpinion.GetAdvice() * 0.9;
                    double temp4 = 1 - xt::sum(temp3)();
                    NTT.getNodeTrust(station26)->TrustOpinion.SetAdvice(temp3);
                    NTT.getNodeTrust(station26)->TrustOpinion.SetU(temp4);
                }
                break;
            }
        }
    }

    tie(inNTT,NTTdis) = NTT.include(station26);
    if(inNTT){
        if(InLocalPerception(station26,lem) && (conflictInclude==true)){
            tie(include,ConflictEventID) = IncludeEvent(station1);
            if(include){
                mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(egoID);
                NTT.getNodeTrust(station26)->TrustOpinion.SetAdvice({0,1});
                NTT.getNodeTrust(station26)->TrustOpinion.SetU(0);
                NTT.getNodeTrust(station26)->NodeType = 0;
            }else{
                vector<uint32_t> ConflictSourceAssociated = {egoID,station26};
                ConflictEventInfo newConflict(station1, OI_type, ConflictSourceAssociated);
                mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
                NTT.getNodeTrust(station26)->NodeType = 0;
            }
        }
    }


    if(InLocalPerception(station63,lem) && (InPerceptionFlag==0)){
        tie(inNTT,NTTdis) = NTT.include(station63);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(station63)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(station63)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(station63)->TrustOpinion.SetU(1-a);
            InPerceptionFlag++;
            NTT.getNodeTrust(station63)->NodeType = 1;
            NTT.getNodeTrust(station63)->LocalBonus = 1;
        }
    }

    if(InLocalPerception(station69,lem) && (InPerceptionFlag==0)){
        tie(inNTT,NTTdis) = NTT.include(station69);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(station69)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(station69)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(station69)->TrustOpinion.SetU(1-a);
            InPerceptionFlag++;
            NTT.getNodeTrust(station69)->NodeType = 1;
            NTT.getNodeTrust(station69)->LocalBonus = 1;
        }
    }

    tie(include,ConflictEventID) = IncludeEvent(station1);   
    if(InLocalPerception(station26,lem) && (conflictInclude==false) && (InPerceptionFlag2==0)){
        tie(inNTT,NTTdis) = NTT.include(station26);
        if (inNTT && (include == false)){    
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(station26)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(station26)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(station26)->TrustOpinion.SetU(1-a);
            InPerceptionFlag2++;  
            NTT.getNodeTrust(station26)->NodeType = 1;
        }     
    }

    if(include){
        cout << "NTT Station < 0.5 :::::"<< NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice()[0] << endl;
        // double temp = NTT.getNodeTrust(stationID)->RevisedOpinion.GetAdvice()[0];

        tie(inNTT,NTTdis) = NTT.include(station63);
        if (inNTT){
            if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (NTT.getNodeTrust(station63)->LocalBonus == 1) && !(InLocalPerception(station63,lem)) && (stationID == station63)){
                double a = 0.04;
                double b = -0.08;
                double n = NTT.getNodeTrust(station63)->LostTime;
                NTT.getNodeTrust(station63)->LostTime++;
                double tempval1 = PenaltyFunction1(n,a,b,0,0.2);
                double tempval2 = PenaltyFunction1(n-1,a,b,0,0.2);
                xt::xarray<double> tempval = {tempval1 - tempval2,0};
                xt::xarray<double> temp = NTT.getNodeTrust(station63)->TrustOpinion.GetAdvice() - tempval;
                NTT.getNodeTrust(station63)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                NTT.getNodeTrust(station63)->TrustOpinion.SetU(1-u);

            }
        }

        if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (recoverFlag == 0)){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
              
                uint32_t OtherID = g[i];
                tie(inNTT,NTTdis) = NTT.include(OtherID);
                if (inNTT){
                    if(g[i]!=station26 && g[i]!=egoID && g[i]!=station39){
                        xt::xarray<double> temp = NTT.getNodeTrust(OtherID)->TrustOpinion.GetAdvice() / 0.9;
                        NTT.getNodeTrust(OtherID)->TrustOpinion.SetAdvice(temp);
                        double a = temp(0);
                        NTT.getNodeTrust(OtherID)->TrustOpinion.SetU(1-a);
                        recoverFlag++;
                    }
                }
            }

        }
        
        if(this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
                tie(inNTT,NTTdis) = NTT.include(g[i]);
                if (inNTT){
                    NTT.getNodeTrust(g[i])->NodeType = 0;
                }
            }
        }
       
        tie(inNTT,NTTdis) = NTT.include(stationID);
        if (inNTT){
            if(NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice()[0] < 0.1){
                cout << "mConflictEventStatus success"<< endl;
                this->mConflictEventTable.setConflictEventStatus(ConflictEventID,Stop);
                NTT.getNodeTrust(stationID)->NodeType = 0;
                cout << "mConflictEventStatus success get status:::"<< this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus << endl;
            }
        }
    }

    for(int i = 0 ; i < NTTtable.size(); i++){
        auto tempStationID = NTTtable[i].stationID;
        double tempType = NTT.getNodeTrust(tempStationID)->NodeType;
        if(tempStationID == stationID){
            if(tempType == 1){
                double a = 0.1;
                double b = -1;
                double y = 8;

                double nTime = NTT.getNodeTrust(tempStationID)->RCtime;
                NTT.getNodeTrust(tempStationID)->RCtime = nTime+1;
                double mRCtime = NTT.getNodeTrust(tempStationID)->RCtime;
                double tempval1 = round(AssessmentFunction(mRCtime,a,b,y)*1000)/1000;
                double tempval2 = round(AssessmentFunction(mRCtime-1,a,b,y)*1000)/1000;
                xt::xarray<double> tempval = {tempval1 - tempval2,0};

                xt::xarray<double> temp = NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice() + tempval;
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetU(1-u);
            }
        }
    }

}


/**
 * @brief specific setting of injection attack senario
 */
void CPMConflictCheck::updateConflictCheckI(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,const VehicleDataProvider* vdp, NodeTrustTable& NTT, 
NodeTableCpm& TableCpm, NodeTypeClassification& NTC, const traci::VehicleController* VehController){

    bool include;
    bool inNTT;
    int ConflictEventID;
    int NTTdis;
    uint32_t stationID = msg->header.stationID;
    uint32_t egoID = vdp->getStationId()%100;
    

    // injection object ID
    uint32_t station1 = 99;

    uint32_t station39 = 39;
    uint32_t station26 = 1;
    uint32_t station63 = 63;
    uint32_t station69 = 82;

    double EmitterLon = msg->cpm.cpmParameters.managementContainer.referencePosition.longitude;
    double EmitterLat = msg->cpm.cpmParameters.managementContainer.referencePosition.latitude;
    double EmitterHeading = msg->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
    auto transEmitterLongLat2XY = VehController->getTraCI()->convert2D({EmitterLon/10000000,EmitterLat/10000000});

    double SensorRangeX1 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading-200)*PI/1800);
    double SensorRangeY1 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading-200)*PI/1800);

    double SensorRangeX2 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading+200)*PI/1800);
    double SensorRangeY2 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading+200)*PI/1800);

    artery::geometry::Point Epos(transEmitterLongLat2XY.x,transEmitterLongLat2XY.y);
    artery::geometry::Point SensorPoint1(SensorRangeX1,SensorRangeY1);
    artery::geometry::Point SensorPoint2(SensorRangeX2,SensorRangeY2);
    artery::geometry::Polygon SensorPolygon{{Epos,SensorPoint1,
                SensorPoint2,Epos}};

    artery::geometry::Point ObjPos;

    auto EgoXY = VehController->getTraCI()->convert2D({vdp->longitude().value(),vdp->latitude().value()});
    double EgoHeading = 360 * vdp->heading().value() / (2*PI);
    double EgoSensorRangeX1 = EgoXY.x + 65 * sin((EgoHeading-20)*PI/180);
    double EgoSensorRangeY1 = EgoXY.y + 65 * cos((EgoHeading-20)*PI/180);

    double EgoSensorRangeX2 = EgoXY.x + 65 * sin((EgoHeading+20)*PI/180);
    double EgoSensorRangeY2 = EgoXY.y + 65 * cos((EgoHeading+20)*PI/180);
    artery::geometry::Point EgoPos(EgoXY.x,EgoXY.y);
    artery::geometry::Point EgoSensorPoint1(EgoSensorRangeX1,EgoSensorRangeY1);
    artery::geometry::Point EgoSensorPoint2(EgoSensorRangeX2,EgoSensorRangeY2);
    artery::geometry::Polygon EgoSensorPolygon{{EgoPos,EgoSensorPoint1,
                EgoSensorPoint2,EgoPos}};


    double InjectObjectXpos;
    double InjectObjectYpos;


    auto NTTtable = NTT.getTrustTable();
    for(int i = 0 ; i < NTTtable.size(); i++){
        auto tempStationID = NTTtable[i].stationID;
        double tempValidity = NTC.GetNodeState(tempStationID).NodeValidity;
        NTT.getNodeTrust(tempStationID)->NodeType =  tempValidity;
    }


    // if(TableCpm.includes(station26)){
    //     vanetza::asn1::Cpm& AttackerLastCPM = *(TableCpm.getNodeHistoryAddr(station26))->getLatestCPMAddr();
    //     PerceivedObjectContainer_t* NodePerceivedObjectList = AttackerLastCPM->cpm.cpmParameters.perceivedObjectContainer;
    //     for(int i=0; i< NodePerceivedObjectList->list.count; i++){
    //         cout << "Attacker Perceived Station ID: " << NodePerceivedObjectList->list.array[i]->objectID << endl;
    //         if(NodePerceivedObjectList->list.array[i]->objectID == station1){
    //             conflictInclude = true;
    //             break;
    //         }
    //     }
    // }
        
    if(TableCpm.includes(station26)){
        vanetza::asn1::Cpm& AttackerLastCPM = *(TableCpm.getNodeHistoryAddr(station26))->getLatestCPMAddr();
        PerceivedObjectContainer_t& addr_NodePerceivedObjectList = AttackerLastCPM->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer;
        PerceivedObjectContainer_t* NodePerceivedObjectList = &addr_NodePerceivedObjectList;
        double AttackerLat = AttackerLastCPM->cpm.cpmParameters.managementContainer.referencePosition.latitude;
        double AttackerLon = AttackerLastCPM->cpm.cpmParameters.managementContainer.referencePosition.longitude;
        double Heading = AttackerLastCPM->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
        auto transAttackerLongLat2XY = VehController->getTraCI()->convert2D({AttackerLon/1e7,AttackerLat/1e7});

        for(int i=0; i< NodePerceivedObjectList->perceivedObjects.list.count; i++){
            auto Obj = NodePerceivedObjectList->perceivedObjects.list.array[i];
            if(Obj->objectID == station1){

                auto InjectObjectXYpos = myTrustManagementFunc.InvXYConvert(Obj->xDistance.value/100,Obj->yDistance.value/100,
                transAttackerLongLat2XY.x,transAttackerLongLat2XY.y,Heading*PI/1800);
                
                InjectObjectXpos = InjectObjectXYpos(0);
                InjectObjectYpos = InjectObjectXYpos(1);
                ObjPos = {InjectObjectXpos,InjectObjectYpos};
                std::cout << "Injection detection success !!!" << InjectObjectXpos << ";"<< InjectObjectYpos << ";" << EgoXY.x << ";" << EgoXY.y 
                        << ";" << EgoSensorRangeX1 << ";" << EgoSensorRangeY1 << ";" << EgoSensorRangeX2 << ";" << EgoSensorRangeY2 << ";" << EgoHeading << std::endl;

                if((egoID == station63) || (egoID == station69)){
                    if(IsPointInPolygon(ObjPos,EgoSensorPolygon)){
                        conflictInclude = true;
                        std::cout << "conflictInclude True !!!" << std::endl;
                    }
                }

                if(stationID != station39){
                    if(IsPointInPolygon(ObjPos,SensorPolygon)){
                        std::cout << "conflictIncludeflag True !!!" << std::endl;
                        tie(include,ConflictEventID) = IncludeEvent(station1);
                        if(include){
                            if(!mConflictEventTable.cTable[ConflictEventID].EventInfo.include(stationID)){ 
                                    mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(stationID);
                                    xt::xarray<double> temp = NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp2 = 1 - xt::sum(temp)();
                                    NTT.getNodeTrust(stationID)->TrustOpinion.SetAdvice(temp);
                                    NTT.getNodeTrust(stationID)->TrustOpinion.SetU(temp2);
                            }
                        }else{
                            if(stationID == station26){
                                continue;
                            }
                            vector<uint32_t> ConflictSourceAssociated = {stationID,station26};
                            ConflictEventInfo newConflict(station1, OI_type, ConflictSourceAssociated);
                            mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
                            xt::xarray<double> temp = NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice() * 0.9;
                            double temp2 = 1 - xt::sum(temp)();
                            NTT.getNodeTrust(stationID)->TrustOpinion.SetAdvice(temp);
                            NTT.getNodeTrust(stationID)->TrustOpinion.SetU(temp2);
                            xt::xarray<double> temp3 = NTT.getNodeTrust(station26)->TrustOpinion.GetAdvice() * 0.9;
                            double temp4 = 1 - xt::sum(temp3)();
                            NTT.getNodeTrust(station26)->TrustOpinion.SetAdvice(temp3);
                            NTT.getNodeTrust(station26)->TrustOpinion.SetU(temp4);
                        }
                    }
                }

                break;
            }
        }
    }
    

    if(InLocalPerception(station26,lem) && (conflictInclude==true)){
        tie(include,ConflictEventID) = IncludeEvent(station1);
        if(include){
            mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(egoID);
            NTT.getNodeTrust(station26)->TrustOpinion.SetAdvice({0,1});
            NTT.getNodeTrust(station26)->TrustOpinion.SetU(0);
            NTT.getNodeTrust(station26)->NodeType = 0;
        }else{
            vector<uint32_t> ConflictSourceAssociated = {egoID,station26};
            ConflictEventInfo newConflict(station1, OI_type, ConflictSourceAssociated);
            mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
            NTT.getNodeTrust(station26)->NodeType = 0;
        }
    }


    if(InLocalPerception(station63,lem) && (InPerceptionFlag==0)){
        tie(inNTT,NTTdis) = NTT.include(station63);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(station63)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(station63)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(station63)->TrustOpinion.SetU(1-a);
            InPerceptionFlag++;
            NTT.getNodeTrust(station63)->NodeType = 1;
            NTT.getNodeTrust(station63)->LocalBonus = 1;
        }
    }


    if(InLocalPerception(station69,lem) && (InPerceptionFlag==0)){
        tie(inNTT,NTTdis) = NTT.include(station69);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(station69)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(station69)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(station69)->TrustOpinion.SetU(1-a);
            InPerceptionFlag++;
            NTT.getNodeTrust(station69)->NodeType = 1;
            NTT.getNodeTrust(station69)->LocalBonus = 1;
        }
    }

    tie(include,ConflictEventID) = IncludeEvent(station1);
    if(InLocalPerception(station26,lem) && (InPerceptionFlag2==0) && (!include)){
        tie(inNTT,NTTdis) = NTT.include(station26);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(station26)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(station26)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(station26)->TrustOpinion.SetU(1-a);
            InPerceptionFlag2++;
            NTT.getNodeTrust(station26)->NodeType = 1;
            NTT.getNodeTrust(station26)->LocalBonus = 1;
        }
    }

    tie(include,ConflictEventID) = IncludeEvent(station1);
    if(include){
        cout << "NTT Station < 0.5 :::::"<< NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice()[0] << endl;
        // double temp = NTT.getNodeTrust(stationID)->RevisedOpinion.GetAdvice()[0];

        tie(inNTT,NTTdis) = NTT.include(station63);
        if(inNTT){
            if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (NTT.getNodeTrust(station63)->LocalBonus == 1) && !(InLocalPerception(station63,lem)) && (stationID == station63)){
                double a = 0.04;
                double b = -0.08;
                double n = NTT.getNodeTrust(station63)->LostTime;
                NTT.getNodeTrust(station63)->LostTime++;
                double tempval1 = PenaltyFunction1(n,a,b,0,0.2);
                double tempval2 = PenaltyFunction1(n-1,a,b,0,0.2);
                xt::xarray<double> tempval = {tempval1 - tempval2,0};
                xt::xarray<double> temp = NTT.getNodeTrust(station63)->TrustOpinion.GetAdvice() - tempval;
                NTT.getNodeTrust(station63)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                NTT.getNodeTrust(station63)->TrustOpinion.SetU(1-u);

            }
        }


        if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (recoverFlag == 0)){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
              
                uint32_t OtherID = g[i];
                tie(inNTT,NTTdis) = NTT.include(OtherID);
                if(inNTT){
                    if(g[i]!=station26 && g[i]!=egoID && g[i]!=station39){
                        xt::xarray<double> temp = NTT.getNodeTrust(OtherID)->TrustOpinion.GetAdvice() / 0.9;
                        NTT.getNodeTrust(OtherID)->TrustOpinion.SetAdvice(temp);
                        double a = temp(0);
                        NTT.getNodeTrust(OtherID)->TrustOpinion.SetU(1-a);
                        recoverFlag++;
                    }
                }
            }

        }
        
        if(this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
                tie(inNTT,NTTdis) = NTT.include(g[i]);
                if(inNTT){
                    NTT.getNodeTrust(g[i])->NodeType = 0;
                }
            }
        }
       

        if(NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice()[0] < 0.1){
            cout << "mConflictEventStatus success"<< endl;
            this->mConflictEventTable.setConflictEventStatus(ConflictEventID,Stop);
            NTT.getNodeTrust(stationID)->NodeType = 0;
            cout << "mConflictEventStatus success get status:::"<< this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus << endl;
        }
    }


    for(int i = 0 ; i < NTTtable.size(); i++){
        auto tempStationID = NTTtable[i].stationID;
        double tempType = NTT.getNodeTrust(tempStationID)->NodeType;
        if(tempStationID == stationID){
            if(tempType == 1){
                double a = 0.1;
                double b = -1;
                double y = 8;

                double nTime = NTT.getNodeTrust(tempStationID)->RCtime;
                NTT.getNodeTrust(tempStationID)->RCtime = nTime+1;
                double mRCtime = NTT.getNodeTrust(tempStationID)->RCtime;
                double tempval1 = round(AssessmentFunction(mRCtime,a,b,y)*1000)/1000;
                double tempval2 = round(AssessmentFunction(mRCtime-1,a,b,y)*1000)/1000;
                xt::xarray<double> tempval = {tempval1 - tempval2,0};

                xt::xarray<double> temp = NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice() + tempval;
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetU(1-u);
            }
        }
    }




}



/**
 * @brief specific setting of injection attack senario with Rsu (for car)
 */
void CPMConflictCheck::updateConflictCheckRsuIC(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,const VehicleDataProvider* vdp, NodeTrustTable& NTT, 
NodeTableCpm& TableCpm, NodeTypeClassification& NTC, const traci::VehicleController* VehController){

    bool include;
    bool inNTT;
    int ConflictEventID;
    int NTTdis;
    double EmitterHeading;
    libsumo::TraCIPosition transEmitterLongLat2XY;
    double SensorRangeX1,SensorRangeY1,SensorRangeX2,SensorRangeY2;

    uint32_t stationID = msg->header.stationID;
    uint32_t egoID = vdp->getStationId()%100;
    

    // injection object ID
    uint32_t stationGhost = 99;

    uint32_t stationEgo = 27;
    uint32_t stationAttacker = 78;
    uint32_t station1 = 73;
    uint32_t stationRsu = 60;

    double EmitterLon = msg->cpm.cpmParameters.managementContainer.referencePosition.longitude;
    double EmitterLat = msg->cpm.cpmParameters.managementContainer.referencePosition.latitude;
    auto CPMStationType = msg->cpm.cpmParameters.managementContainer.stationType;
    if (CPMStationType == StationType_passengerCar){
        EmitterHeading = msg->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
        transEmitterLongLat2XY = VehController->getTraCI()->convert2D({EmitterLon/1e7,EmitterLat/1e7});
        SensorRangeX1 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading-200)*PI/1800);
        SensorRangeY1 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading-200)*PI/1800);

        SensorRangeX2 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading+200)*PI/1800);
        SensorRangeY2 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading+200)*PI/1800);

    }else if(CPMStationType == StationType_roadSideUnit){
        EmitterHeading = 900;
        transEmitterLongLat2XY = VehController->getTraCI()->convert2D({EmitterLon/1e7,EmitterLat/1e7});
        SensorRangeX1 = transEmitterLongLat2XY.x + 200 * sin((EmitterHeading-100)*PI/1800);
        SensorRangeY1 = transEmitterLongLat2XY.y + 200 * cos((EmitterHeading-100)*PI/1800);

        SensorRangeX2 = transEmitterLongLat2XY.x + 200 * sin((EmitterHeading+100)*PI/1800);
        SensorRangeY2 = transEmitterLongLat2XY.y + 200 * cos((EmitterHeading+100)*PI/1800);        
    }
    artery::geometry::Point Emitpos(transEmitterLongLat2XY.x,transEmitterLongLat2XY.y);
    artery::geometry::Point SensorPoint1(SensorRangeX1,SensorRangeY1);
    artery::geometry::Point SensorPoint2(SensorRangeX2,SensorRangeY2);
    artery::geometry::Polygon EmitSensorPolygon{{Emitpos,SensorPoint1,
                SensorPoint2,Emitpos}};

    artery::geometry::Point ObjPos;
    auto EgoXY = VehController->getTraCI()->convert2D({vdp->longitude().value(),vdp->latitude().value()});
    double EgoHeading = 360 * vdp->heading().value() / (2*PI);
    double EgoSensorRangeX1 = EgoXY.x + 65 * sin((EgoHeading-20)*PI/180);
    double EgoSensorRangeY1 = EgoXY.y + 65 * cos((EgoHeading-20)*PI/180);

    double EgoSensorRangeX2 = EgoXY.x + 65 * sin((EgoHeading+20)*PI/180);
    double EgoSensorRangeY2 = EgoXY.y + 65 * cos((EgoHeading+20)*PI/180);
    artery::geometry::Point EgoPos(EgoXY.x,EgoXY.y);
    artery::geometry::Point EgoSensorPoint1(EgoSensorRangeX1,EgoSensorRangeY1);
    artery::geometry::Point EgoSensorPoint2(EgoSensorRangeX2,EgoSensorRangeY2);
    artery::geometry::Polygon EgoSensorPolygon{{EgoPos,EgoSensorPoint1,
                EgoSensorPoint2,EgoPos}};


    double InjectObjectXpos;
    double InjectObjectYpos;


    auto NTTtable = NTT.getTrustTable();
    for(int i = 0 ; i < NTTtable.size(); i++){
        auto tempStationID = NTTtable[i].stationID;
        double tempValidity = NTC.GetNodeState(tempStationID).NodeValidity;
        NTT.getNodeTrust(tempStationID)->NodeType =  tempValidity;
        if(NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice()[0] < 0.1){
            NTT.getNodeTrust(tempStationID)->NodeType = 0;
        }
    }

        
    if(TableCpm.includes(stationAttacker)){
        vanetza::asn1::Cpm& AttackerLastCPM = *(TableCpm.getNodeHistoryAddr(stationAttacker))->getLatestCPMAddr();
        PerceivedObjectContainer_t& addr_NodePerceivedObjectList = AttackerLastCPM->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer;
        PerceivedObjectContainer_t* NodePerceivedObjectList = &addr_NodePerceivedObjectList;
        double AttackerLat = AttackerLastCPM->cpm.cpmParameters.managementContainer.referencePosition.latitude;
        double AttackerLon = AttackerLastCPM->cpm.cpmParameters.managementContainer.referencePosition.longitude;
        auto transAttackerLongLat2XY = VehController->getTraCI()->convert2D({AttackerLon/1e7,AttackerLat/1e7});

        for(int i=0; i< NodePerceivedObjectList->perceivedObjects.list.count; i++){
            auto Obj = NodePerceivedObjectList->perceivedObjects.list.array[i];
            if(Obj->objectID == stationGhost){
                InjectObjectXpos = transAttackerLongLat2XY.x + Obj->xDistance.value/100;
                InjectObjectYpos = transAttackerLongLat2XY.y + Obj->yDistance.value/100;
                ObjPos = {InjectObjectXpos,InjectObjectYpos};
                std::cout << "Injection detection success !!!" << InjectObjectXpos << ";"<< InjectObjectYpos << ";" << EgoXY.x << ";" << EgoXY.y 
                        << ";" << EgoSensorRangeX1 << ";" << EgoSensorRangeY1 << ";" << EgoSensorRangeX2 << ";" << EgoSensorRangeY2 << ";" << EgoHeading << std::endl;

                if((egoID == station1) || (egoID == stationRsu)){
                    if(IsPointInPolygon(ObjPos,EgoSensorPolygon)){
                        conflictInclude = true;
                        std::cout << "conflictInclude True !!!" << std::endl;
                    }
                }

                if(stationID != stationEgo){
                    if(IsPointInPolygon(ObjPos,EmitSensorPolygon)){
                        std::cout << "conflictIncludeflag True !!!" << std::endl;
                        tie(include,ConflictEventID) = IncludeEvent(stationGhost);
                        if(include){
                            if(!mConflictEventTable.cTable[ConflictEventID].EventInfo.include(stationID)){ 
                                    mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(stationID);
                                    xt::xarray<double> temp = NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp2 = 1 - xt::sum(temp)();
                                    NTT.getNodeTrust(stationID)->TrustOpinion.SetAdvice(temp);
                                    NTT.getNodeTrust(stationID)->TrustOpinion.SetU(temp2);
                                    std::cout << "include okkkk!!!" << std::endl;
                                    NTT.getNodeTrust(stationID)->InConflict = 1;
                            }
                        }else{
                            std::cout << "include else okkkk!!! 1" << std::endl;
                            if(stationID == stationAttacker){
                                continue;
                            }
                            vector<uint32_t> ConflictSourceAssociated = {stationID,stationAttacker};
                            ConflictEventInfo newConflict(stationGhost, OI_type, ConflictSourceAssociated);
                            mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
                            std::cout << "include else okkkk!!! 2 stationID::" << stationID << std::endl;
                            auto iiinclude = std::get<0>(NTT.include(stationID));
                            std::cout << "include else okkkk!!! 3:::" << iiinclude << std::endl;
                            xt::xarray<double> temp = NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice() * 0.9;
                            
                            double temp2 = 1 - xt::sum(temp)();
                            NTT.getNodeTrust(stationID)->TrustOpinion.SetAdvice(temp);
                            NTT.getNodeTrust(stationID)->TrustOpinion.SetU(temp2);
                            NTT.getNodeTrust(stationID)->InConflict = 1;
                            
                            xt::xarray<double> temp3 = NTT.getNodeTrust(stationAttacker)->TrustOpinion.GetAdvice() * 0.9;
                            double temp4 = 1 - xt::sum(temp3)();
                            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetAdvice(temp3);
                            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetU(temp4);
                            std::cout << "include else okkkk!!! 4" << std::endl;
                            NTT.getNodeTrust(stationAttacker)->InConflict = 1;
                            
                        }
                    }
                }

                break;
            }
        }
    }
    

    if(InLocalPerception(stationAttacker,lem) && (conflictInclude==true)){
        // tie(include,ConflictEventID) = IncludeEvent(stationGhost);
        // if(include){
            // mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(egoID);
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetAdvice({0,1});
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetU(0);
            NTT.getNodeTrust(stationAttacker)->NodeType = 0;
        // }else{
            // vector<uint32_t> ConflictSourceAssociated = {egoID,stationAttacker};
            // ConflictEventInfo newConflict(stationGhost, OI_type, ConflictSourceAssociated);
            // mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
            // NTT.getNodeTrust(stationAttacker)->NodeType = 0;
        // }
    }

    if(InLocalPerception(station1,lem) && (InPerceptionFlag==0)){
        tie(inNTT,NTTdis) = NTT.include(station1);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(station1)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(station1)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(station1)->TrustOpinion.SetU(1-a);
            InPerceptionFlag++;
            NTT.getNodeTrust(station1)->NodeType = 1;
            NTT.getNodeTrust(station1)->LocalBonus = 1;
        }
    }

    if(InLocalPerception(stationAttacker,lem) && (InPerceptionFlag==0)){
        tie(inNTT,NTTdis) = NTT.include(stationAttacker);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(stationAttacker)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetU(1-a);
            InPerceptionFlag++;
            NTT.getNodeTrust(stationAttacker)->NodeType = 1;
            NTT.getNodeTrust(stationAttacker)->LocalBonus = 1;
        }
    }

    tie(include,ConflictEventID) = IncludeEvent(stationGhost);
    if(include){
        cout << "NTT Station < 0.5 :::::"<< NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice()[0] << endl;  
        // auto TmConflictEventStatus = this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus;


        if((NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice()[0] < 0.1) 
        &&( this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress)){
            cout << "mConflictEventStatus success"<< endl;
            this->mConflictEventTable.setConflictEventStatus(ConflictEventID,Stop);
            NTT.getNodeTrust(stationID)->NodeType = 0;
            cout << "mConflictEventStatus success get status:::"<< this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus << endl;
        }
        
        if(this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            int gszie = g.size();
            for(int i=0; i < gszie; i++){
                NTT.getNodeTrust(g[i])->NodeType = 0;
            }
            
        }   
        

        // if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (NTT.getNodeTrust(station1)->LocalBonus == 1) && !(InLocalPerception(station1,lem)) && (stationID == station1)){
        //     double a = 0.04;
        //     double b = -0.08;
        //     double n = NTT.getNodeTrust(station1)->LostTime;
        //     NTT.getNodeTrust(station1)->LostTime++;
        //     double tempval1 = PenaltyFunction1(n,a,b,0,0.2);
        //     double tempval2 = PenaltyFunction1(n-1,a,b,0,0.2);
        //     xt::xarray<double> tempval = {tempval1 - tempval2,0};
        //     xt::xarray<double> temp = NTT.getNodeTrust(station1)->TrustOpinion.GetAdvice() - tempval;
        //     NTT.getNodeTrust(station1)->TrustOpinion.SetAdvice(temp);
        //     double u = temp(0);
        //     NTT.getNodeTrust(station1)->TrustOpinion.SetU(1-u);
        // }
       

        if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (recoverFlag == 0)){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
              
                uint32_t OtherID = g[i];
                if(g[i]!=stationAttacker && g[i]!=egoID && g[i]!=stationEgo){
                    xt::xarray<double> temp = NTT.getNodeTrust(OtherID)->TrustOpinion.GetAdvice() / 0.9;
                    NTT.getNodeTrust(OtherID)->TrustOpinion.SetAdvice(temp);
                    double a = temp(0);
                    NTT.getNodeTrust(OtherID)->TrustOpinion.SetU(1-a);
                    recoverFlag++;
                }
            }

        }
        
        
    }


    for(int i = 0 ; i < NTTtable.size(); i++){
        if(NTTtable[i].StationType == StationTypeTrustInit_RoadSideUnit){
            continue;
        }
        auto tempStationID = NTTtable[i].stationID;
        double tempType = NTT.getNodeTrust(tempStationID)->NodeType;
        if(tempStationID == stationID){
            if(tempType == 1){
                double a = 0.1;
                double b = -1;
                double y = 8;

                double nTime = NTT.getNodeTrust(tempStationID)->RCtime;
                NTT.getNodeTrust(tempStationID)->RCtime = nTime+1;
                double mRCtime = NTT.getNodeTrust(tempStationID)->RCtime;
                double tempval1 = round(AssessmentFunction(mRCtime,a,b,y)*1000)/1000;
                double tempval2 = round(AssessmentFunction(mRCtime-1,a,b,y)*1000)/1000;
                xt::xarray<double> tempval = {tempval1 - tempval2,0};

                xt::xarray<double> temp = NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice() + tempval;
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetU(1-u);
            }
        }
    }




}



/**
 * @brief specific setting of injection attack senario with Rsu (for Rsu)
 */
void CPMConflictCheck::updateConflictCheckRsuIR(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,NodeTrustTable& NTT, 
NodeTableCpm& TableCpm, NodeTypeClassification& NTC,std::shared_ptr<traci::API>* TraciLiteAPI, const Identity* mIdentity,const GeoPosition* mGeoPosition){

    bool include;
    bool inNTT;
    int ConflictEventID;
    int NTTdis;
    double EmitterHeading;
    libsumo::TraCIPosition transEmitterLongLat2XY;
    double SensorRangeX1,SensorRangeY1,SensorRangeX2,SensorRangeY2;

    uint32_t stationID = msg->header.stationID;
    uint32_t egoID = mIdentity->application%100;
    

    // injection object ID
    uint32_t stationGhost = 99;

    uint32_t stationEgo = 27;
    uint32_t stationAttacker = 78;
    uint32_t station1 = 73;
    uint32_t stationRsu = 60;

    double EmitterLon = msg->cpm.cpmParameters.managementContainer.referencePosition.longitude;
    double EmitterLat = msg->cpm.cpmParameters.managementContainer.referencePosition.latitude;
    auto CPMStationType = msg->cpm.cpmParameters.managementContainer.stationType;
    if (CPMStationType == StationType_passengerCar){
        EmitterHeading = msg->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
        transEmitterLongLat2XY = (*TraciLiteAPI)->convert2D({EmitterLon/1e7,EmitterLat/1e7});
        SensorRangeX1 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading-200)*PI/1800);
        SensorRangeY1 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading-200)*PI/1800);

        SensorRangeX2 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading+200)*PI/1800);
        SensorRangeY2 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading+200)*PI/1800);

    }else if(CPMStationType == StationType_roadSideUnit){
        EmitterHeading = 900;
        transEmitterLongLat2XY = (*TraciLiteAPI)->convert2D({EmitterLon/1e7,EmitterLat/1e7});
        SensorRangeX1 = transEmitterLongLat2XY.x + 200 * sin((EmitterHeading-100)*PI/1800);
        SensorRangeY1 = transEmitterLongLat2XY.y + 200 * cos((EmitterHeading-100)*PI/1800);

        SensorRangeX2 = transEmitterLongLat2XY.x + 200 * sin((EmitterHeading+100)*PI/1800);
        SensorRangeY2 = transEmitterLongLat2XY.y + 200 * cos((EmitterHeading+100)*PI/1800);        
    }
    artery::geometry::Point Emitpos(transEmitterLongLat2XY.x,transEmitterLongLat2XY.y);
    artery::geometry::Point SensorPoint1(SensorRangeX1,SensorRangeY1);
    artery::geometry::Point SensorPoint2(SensorRangeX2,SensorRangeY2);
    artery::geometry::Polygon EmitSensorPolygon{{Emitpos,SensorPoint1,
                SensorPoint2,Emitpos}};

    artery::geometry::Point ObjPos;
    auto EgoXY = (*TraciLiteAPI)->convert2D({mGeoPosition->longitude.value(),mGeoPosition->latitude.value()});
    double EgoHeading = 90;
    double EgoSensorRangeX1 = EgoXY.x + 200 * sin((EgoHeading-10)*PI/180);
    double EgoSensorRangeY1 = EgoXY.y + 200 * cos((EgoHeading-10)*PI/180);

    double EgoSensorRangeX2 = EgoXY.x + 200 * sin((EgoHeading+10)*PI/180);
    double EgoSensorRangeY2 = EgoXY.y + 200 * cos((EgoHeading+10)*PI/180);
    artery::geometry::Point EgoPos(EgoXY.x,EgoXY.y);
    artery::geometry::Point EgoSensorPoint1(EgoSensorRangeX1,EgoSensorRangeY1);
    artery::geometry::Point EgoSensorPoint2(EgoSensorRangeX2,EgoSensorRangeY2);
    artery::geometry::Polygon EgoSensorPolygon{{EgoPos,EgoSensorPoint1,
                EgoSensorPoint2,EgoPos}};


    double InjectObjectXpos;
    double InjectObjectYpos;


    auto NTTtable = NTT.getTrustTable();
    for(int i = 0 ; i < NTTtable.size(); i++){
        auto tempStationID = NTTtable[i].stationID;
        double tempValidity = NTC.GetNodeState(tempStationID).NodeValidity;
        NTT.getNodeTrust(tempStationID)->NodeType =  tempValidity;
        if(NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice()[0] < 0.1){
            NTT.getNodeTrust(tempStationID)->NodeType = 0;
        }
    }

        
    if(TableCpm.includes(stationAttacker)){
        vanetza::asn1::Cpm& AttackerLastCPM = *(TableCpm.getNodeHistoryAddr(stationAttacker))->getLatestCPMAddr();
        PerceivedObjectContainer_t& addr_NodePerceivedObjectList = AttackerLastCPM->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer;
        PerceivedObjectContainer_t* NodePerceivedObjectList = &addr_NodePerceivedObjectList;
        double AttackerLat = AttackerLastCPM->cpm.cpmParameters.managementContainer.referencePosition.latitude;
        double AttackerLon = AttackerLastCPM->cpm.cpmParameters.managementContainer.referencePosition.longitude;
        auto transAttackerLongLat2XY = (*TraciLiteAPI)->convert2D({AttackerLon/1e7,AttackerLat/1e7});

        for(int i=0; i< NodePerceivedObjectList->perceivedObjects.list.count; i++){
            auto Obj = NodePerceivedObjectList->perceivedObjects.list.array[i];
            if(Obj->objectID == stationGhost){
                InjectObjectXpos = transAttackerLongLat2XY.x + Obj->xDistance.value/100;
                InjectObjectYpos = transAttackerLongLat2XY.y + Obj->yDistance.value/100;
                ObjPos = {InjectObjectXpos,InjectObjectYpos};
                std::cout << "Injection detection success !!!" << InjectObjectXpos << ";"<< InjectObjectYpos << ";" << EgoXY.x << ";" << EgoXY.y 
                        << ";" << EgoSensorRangeX1 << ";" << EgoSensorRangeY1 << ";" << EgoSensorRangeX2 << ";" << EgoSensorRangeY2 << ";" << EgoHeading << std::endl;

                if(egoID == stationRsu){
                    if(IsPointInPolygon(ObjPos,EgoSensorPolygon)){
                        conflictInclude = true;
                        std::cout << "RSU conflictInclude True !!!" << std::endl;
                    }
                }

                if((stationID != stationEgo) && (conflictInclude != true)){
                    if(IsPointInPolygon(ObjPos,EmitSensorPolygon)){
                        std::cout << "conflictIncludeflag True !!!" << std::endl;
                        tie(include,ConflictEventID) = IncludeEvent(stationGhost);
                        if(include){
                            if(!mConflictEventTable.cTable[ConflictEventID].EventInfo.include(stationID)){ 
                                    mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(stationID);
                                    xt::xarray<double> temp = NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp2 = 1 - xt::sum(temp)();
                                    NTT.getNodeTrust(stationID)->TrustOpinion.SetAdvice(temp);
                                    NTT.getNodeTrust(stationID)->TrustOpinion.SetU(temp2);
                            }
                        }else{
                            if(stationID == stationAttacker){
                                continue;
                            }
                            vector<uint32_t> ConflictSourceAssociated = {stationID,stationAttacker};
                            ConflictEventInfo newConflict(stationGhost, OI_type, ConflictSourceAssociated);
                            mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
                            xt::xarray<double> temp = NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice() * 0.9;
                            double temp2 = 1 - xt::sum(temp)();
                            NTT.getNodeTrust(stationID)->TrustOpinion.SetAdvice(temp);
                            NTT.getNodeTrust(stationID)->TrustOpinion.SetU(temp2);
                            xt::xarray<double> temp3 = NTT.getNodeTrust(stationAttacker)->TrustOpinion.GetAdvice() * 0.9;
                            double temp4 = 1 - xt::sum(temp3)();
                            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetAdvice(temp3);
                            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetU(temp4);
                        }
                    }
                }

                break;
            }
        }
    }
    

    if(InLocalPerception(stationAttacker,lem) && (conflictInclude==true)){
        // tie(include,ConflictEventID) = IncludeEvent(stationGhost);
        // if(include){
            // mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(egoID);
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetAdvice({0,1});
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetU(0);
            NTT.getNodeTrust(stationAttacker)->NodeType = 0;
            std::cout << "RSU attacker setting correct !!!" << std::endl;
        // }else{
            // vector<uint32_t> ConflictSourceAssociated = {egoID,stationAttacker};
            // ConflictEventInfo newConflict(stationGhost, OI_type, ConflictSourceAssociated);
            // mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
            // NTT.getNodeTrust(stationAttacker)->NodeType = 0;
        // }
    }

    if(InLocalPerception(station1,lem) && (InPerceptionFlag2==0)){
        tie(inNTT,NTTdis) = NTT.include(station1);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(station1)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(station1)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(station1)->TrustOpinion.SetU(1-a);
            InPerceptionFlag2++;
            NTT.getNodeTrust(station1)->NodeType = 1;
            NTT.getNodeTrust(station1)->LocalBonus = 1;
        }
    }

    if(InLocalPerception(stationAttacker,lem) && (InPerceptionFlag==0)){
        tie(inNTT,NTTdis) = NTT.include(stationAttacker);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(stationAttacker)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetU(1-a);
            InPerceptionFlag++;
            NTT.getNodeTrust(stationAttacker)->NodeType = 1;
            NTT.getNodeTrust(stationAttacker)->LocalBonus = 1;
        }
    }

    tie(include,ConflictEventID) = IncludeEvent(stationGhost);
    if(include){
        cout << "NTT Station < 0.5 :::::"<< NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice()[0] << endl;
        // double temp = NTT.getNodeTrust(stationID)->RevisedOpinion.GetAdvice()[0];

        if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (NTT.getNodeTrust(station1)->LocalBonus == 1) && !(InLocalPerception(station1,lem)) && (stationID == station1)){
            double a = 0.04;
            double b = -0.08;
            double n = NTT.getNodeTrust(station1)->LostTime;
            NTT.getNodeTrust(station1)->LostTime++;
            double tempval1 = PenaltyFunction1(n,a,b,0,0.2);
            double tempval2 = PenaltyFunction1(n-1,a,b,0,0.2);
            xt::xarray<double> tempval = {tempval1 - tempval2,0};
            xt::xarray<double> temp = NTT.getNodeTrust(station1)->TrustOpinion.GetAdvice() - tempval;
            NTT.getNodeTrust(station1)->TrustOpinion.SetAdvice(temp);
            double u = temp(0);
            NTT.getNodeTrust(station1)->TrustOpinion.SetU(1-u);

        }


        if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (recoverFlag == 0)){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
              
                uint32_t OtherID = g[i];
                if(g[i]!=stationAttacker && g[i]!=egoID && g[i]!=stationEgo){
                    xt::xarray<double> temp = NTT.getNodeTrust(OtherID)->TrustOpinion.GetAdvice() / 0.9;
                    NTT.getNodeTrust(OtherID)->TrustOpinion.SetAdvice(temp);
                    double a = temp(0);
                    NTT.getNodeTrust(OtherID)->TrustOpinion.SetU(1-a);
                    recoverFlag++;
                }
            }

        }
        
        if(this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
                NTT.getNodeTrust(g[i])->NodeType = 0;
            }
        }
       

        if(NTT.getNodeTrust(stationID)->TrustOpinion.GetAdvice()[0] < 0.1){
            cout << "mConflictEventStatus success"<< endl;
            this->mConflictEventTable.setConflictEventStatus(ConflictEventID,Stop);
            NTT.getNodeTrust(stationID)->NodeType = 0;
            cout << "mConflictEventStatus success get status:::"<< this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus << endl;
        }
    }


    for(int i = 0 ; i < NTTtable.size(); i++){
        if(NTTtable[i].StationType == StationTypeTrustInit_RoadSideUnit){
            continue;
        }
        auto tempStationID = NTTtable[i].stationID;
        double tempType = NTT.getNodeTrust(tempStationID)->NodeType;
        if(tempStationID == stationID){
            if(tempType == 1){
                double a = 0.1;
                double b = -1;
                double y = 8;

                double nTime = NTT.getNodeTrust(tempStationID)->RCtime;
                NTT.getNodeTrust(tempStationID)->RCtime = nTime+1;
                double mRCtime = NTT.getNodeTrust(tempStationID)->RCtime;
                double tempval1 = round(AssessmentFunction(mRCtime,a,b,y)*1000)/1000;
                double tempval2 = round(AssessmentFunction(mRCtime-1,a,b,y)*1000)/1000;
                xt::xarray<double> tempval = {tempval1 - tempval2,0};

                xt::xarray<double> temp = NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice() + tempval;
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetU(1-u);
            }
        }
    }

}



/**
 * @brief specific setting of Omission attack senario with Rsu (for car)
 */
void CPMConflictCheck::updateConflictCheckRsuOC(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem,const VehicleDataProvider* vdp, NodeTrustTable& NTT, 
NodeTableCpm& TableCpm, NodeTypeClassification& NTC, const traci::VehicleController* VehController){

    bool include;
    bool inNTT;
    int ConflictEventID;
    int NTTdis;
    double EmitterHeading;
    libsumo::TraCIPosition transEmitterLongLat2XY;
    double SensorRangeX1,SensorRangeY1,SensorRangeX2,SensorRangeY2;

    uint32_t EmitterID = msg->header.stationID;
    uint32_t egoID = vdp->getStationId()%100;
    auto PerceivedSize = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
    
    

    // injection object ID
    uint32_t stationOmit = 78;

    uint32_t stationEgo = 27;
    uint32_t stationAttacker = 24;
    uint32_t station1 = 80;
    uint32_t stationRsu = 60;

    double EmitterLon = msg->cpm.cpmParameters.managementContainer.referencePosition.longitude;
    double EmitterLat = msg->cpm.cpmParameters.managementContainer.referencePosition.latitude;
    auto CPMStationType = msg->cpm.cpmParameters.managementContainer.stationType;
    if (CPMStationType == StationType_passengerCar){
        EmitterHeading = msg->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
        transEmitterLongLat2XY = VehController->getTraCI()->convert2D({EmitterLon/1e7,EmitterLat/1e7});
        SensorRangeX1 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading-200)*PI/1800);
        SensorRangeY1 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading-200)*PI/1800);

        SensorRangeX2 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading+200)*PI/1800);
        SensorRangeY2 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading+200)*PI/1800);

    }else if(CPMStationType == StationType_roadSideUnit){
        EmitterHeading = 900;
        transEmitterLongLat2XY = VehController->getTraCI()->convert2D({EmitterLon/1e7,EmitterLat/1e7});
        SensorRangeX1 = transEmitterLongLat2XY.x + 200 * sin((EmitterHeading-100)*PI/1800);
        SensorRangeY1 = transEmitterLongLat2XY.y + 200 * cos((EmitterHeading-100)*PI/1800);

        SensorRangeX2 = transEmitterLongLat2XY.x + 200 * sin((EmitterHeading+100)*PI/1800);
        SensorRangeY2 = transEmitterLongLat2XY.y + 200 * cos((EmitterHeading+100)*PI/1800);        
    }
    artery::geometry::Point Emitpos(transEmitterLongLat2XY.x,transEmitterLongLat2XY.y);
    artery::geometry::Point SensorPoint1(SensorRangeX1,SensorRangeY1);
    artery::geometry::Point SensorPoint2(SensorRangeX2,SensorRangeY2);
    artery::geometry::Polygon EmitSensorPolygon{{Emitpos,SensorPoint1,
                SensorPoint2,Emitpos}};

    artery::geometry::Point ObjPos;
    auto EgoXY = VehController->getTraCI()->convert2D({vdp->longitude().value(),vdp->latitude().value()});
    double EgoHeading = 360 * vdp->heading().value() / (2*PI);
    double EgoSensorRangeX1 = EgoXY.x + 65 * sin((EgoHeading-20)*PI/180);
    double EgoSensorRangeY1 = EgoXY.y + 65 * cos((EgoHeading-20)*PI/180);

    double EgoSensorRangeX2 = EgoXY.x + 65 * sin((EgoHeading+20)*PI/180);
    double EgoSensorRangeY2 = EgoXY.y + 65 * cos((EgoHeading+20)*PI/180);
    artery::geometry::Point EgoPos(EgoXY.x,EgoXY.y);
    artery::geometry::Point EgoSensorPoint1(EgoSensorRangeX1,EgoSensorRangeY1);
    artery::geometry::Point EgoSensorPoint2(EgoSensorRangeX2,EgoSensorRangeY2);
    artery::geometry::Polygon EgoSensorPolygon{{EgoPos,EgoSensorPoint1,
                EgoSensorPoint2,EgoPos}};


    double LocalObjectXpos;
    double LocalObjectYpos;
    int EmitterLocalBonus;

    auto NTTtable = NTT.getTrustTable();
    for(int i = 0 ; i < NTTtable.size(); i++){
        auto tempStationID = NTTtable[i].stationID;
        double tempValidity = NTC.GetNodeState(tempStationID).NodeValidity;
        NTT.getNodeTrust(tempStationID)->NodeType =  tempValidity;
        if(NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice()[0] < 0.1){
            NTT.getNodeTrust(tempStationID)->NodeType = 0;
        }
    }

    if(std::get<0>(NTT.include(EmitterID))){
        EmitterLocalBonus = NTT.getNodeTrust(EmitterID)->LocalBonus;
    }
    // Source Reliability Verification Module
    const auto& localObj = lem.allObjects();
    const TrackedObjectsFilterRange& TrackedBbjs = filterBySensorCategory(localObj, "Radar");
    for(const auto& i: TrackedBbjs){
        std::weak_ptr<EnvironmentModelObject> obj_ptr = i.first;  
        if (obj_ptr.expired()) continue;
        const VehicleDataProvider& vd = obj_ptr.lock()->getVehicleData(); 
        auto LocalPerceivedObjID =  vd.getStationId()%100; 
        if (LocalPerceivedObjID == EmitterID){

        }else{
            auto LocalObjectXYpos = VehController->getTraCI()->convert2D({vd.longitude().value(),vd.latitude().value()});
            LocalObjectXpos = LocalObjectXYpos.x;
            LocalObjectYpos = LocalObjectXYpos.y;
            artery::geometry::Point LocalObjPos(LocalObjectXpos,LocalObjectYpos);

            bool LocalObjTest = IsPointInPolygon(LocalObjPos,EmitSensorPolygon);
            if(LocalObjTest){
                if(PerceivedSize == 0){
                    std::cout << "detected the omission within local perception !!!!  " << EmitterID << "; " << PerceivedSize << std::endl;
                    conflictInclude = true;
                    goto detected;
                }else{
                    for(int indxPerceivedObj = 0; indxPerceivedObj < PerceivedSize ; indxPerceivedObj++){
                        auto ReportedNodeID = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[indxPerceivedObj]->objectID;
                        if(LocalPerceivedObjID == ReportedNodeID){
                            conflictInclude = false;
                            break;
                        }
                        conflictInclude = true;    
                    }
                    if(conflictInclude){
                        goto detected;
                    }
                }
            }
        }
    }


    // Perception Data reliability Verification Module
    for(int indxPerceivedObj = 0; indxPerceivedObj < PerceivedSize ; indxPerceivedObj++){
        auto Obj = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[indxPerceivedObj];
        auto ReportedNodeID = Obj->objectID;
        double ObjPosX = transEmitterLongLat2XY.x + (Obj->xDistance.value)/100;
        double ObjPosY = transEmitterLongLat2XY.y + (Obj->yDistance.value)/100;
        double ObjSpeedX = Obj->xSpeed.value/100;
        double ObjSpeedY = Obj->ySpeed.value/100;
         std::cout << "EO dis:::::"<< Obj->xDistance.value << ","<< Obj->yDistance.value <<std::endl;

        artery::geometry::Point ObjPos(ObjPosX,ObjPosY);

        bool IsInLocalPerception = InLocalPerception(ReportedNodeID,lem);

        if(IsInLocalPerception){
            // Kinematic data checks
        
        }else{



            // Reported object, is in other sources perception range ?
            int SourceNum = TableCpm.getNodesNum();
            
            for(int SourceIndx = 0; SourceIndx < SourceNum; SourceIndx++){
                uint32_t SourceID = TableCpm.getNodeID(SourceIndx);


                if((SourceID != EmitterID) && (SourceID != ReportedNodeID)){
                   
                    vanetza::asn1::Cpm* LastCPM = TableCpm.getNodeHistoryAddr(SourceID)->getLatestCPMAddr();
                    int PerceivedSize = (*LastCPM)->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
                    double LastNodeLat = (*LastCPM)->cpm.cpmParameters.managementContainer.referencePosition.latitude;
                    double LastNodeLog = (*LastCPM)->cpm.cpmParameters.managementContainer.referencePosition.longitude;
                    auto SourceStationType = (*LastCPM)->cpm.cpmParameters.managementContainer.stationType;
                    double SourceHeading;
                    double SourceSensorRange,SourceSensorAngle;

                    // sensor Info
                    if(SourceStationType == StationType_passengerCar){
                        SourceHeading  = (*LastCPM)->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
                        SourceSensorRange = 65;
                        SourceSensorAngle = 400;
                    }else if(SourceStationType ==  StationType_roadSideUnit){
                        SourceHeading  = 900;
                        SourceSensorRange = 200;
                        SourceSensorAngle = 200;
                    }
                    
                    auto transLongLatXY = VehController->getTraCI()->convert2D({LastNodeLog/1e7,LastNodeLat/1e7});
                    
                    double SensorRangeX1 = transLongLatXY.x + SourceSensorRange * sin((SourceHeading-SourceSensorAngle/2)*PI/1800);
                    double SensorRangeY1 = transLongLatXY.y + SourceSensorRange * cos((SourceHeading-SourceSensorAngle/2)*PI/1800);

                    double SensorRangeX2 = transLongLatXY.x + SourceSensorRange * sin((SourceHeading+SourceSensorAngle/2)*PI/1800);
                    double SensorRangeY2 = transLongLatXY.y + SourceSensorRange * cos((SourceHeading+SourceSensorAngle/2)*PI/1800);

                    artery::geometry::Point otherVPos(transLongLatXY.x,transLongLatXY.y);
                    artery::geometry::Point SensorRangePos1(SensorRangeX1,SensorRangeY1);
                    artery::geometry::Point SensorRangePos2(SensorRangeX2,SensorRangeY2);
                    std::cout << "Emitter Pos:::::"<< bg::get<0>(Emitpos) << ","<< bg::get<1>(Emitpos) <<std::endl;
                    std::cout << "ObjPos:::::"<< bg::get<0>(ObjPos) << ","<< bg::get<1>(ObjPos) <<std::endl;
                    std::cout << "otherVPos:::::"<< bg::get<0>(otherVPos) << ","<< bg::get<1>(otherVPos) <<std::endl;
                    std::cout << "SensorRangePos1:::::"<< bg::get<0>(SensorRangePos1) << ","<< bg::get<1>(SensorRangePos1) <<std::endl;
                    std::cout << "SensorRangePos2:::::"<< bg::get<0>(SensorRangePos2) << ","<< bg::get<1>(SensorRangePos2) <<std::endl;

                    // Sensor Polygon
                    artery::geometry::Polygon SensorPolygon{{otherVPos,SensorRangePos1,
                                SensorRangePos2,otherVPos}};
                    // artery::geometry::Polygon SensorPolygon{{{0,0},{100,0},{100,100},{0,100},{0,0}}};

                
                    // check if point in the other source's sensor range
                    bool IsInSensorRange = IsPointInPolygon(ObjPos,SensorPolygon);

                    if(SourceID == stationEgo){
                        IsInSensorRange = false;
                    }
                    
                    std:cout << "______SenderID:" << EmitterID << "; SourceID:" << SourceID<< "; objectID:" << ReportedNodeID << ";InSR?"<<IsInSensorRange << std::endl;
                    if(IsInSensorRange){

                        std::cout << "22222222222222222222222222222222222222"  <<std::endl;
                        if(PerceivedSize == 0){
                            CheckConflict = 1;
                        }
                        for(int nodePerceivedObjIndx = 0; nodePerceivedObjIndx < PerceivedSize; nodePerceivedObjIndx++){
                            if((*LastCPM)->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[nodePerceivedObjIndx]->objectID == ReportedNodeID){
                                // other source detects this object
                                CheckConflict = 0;
                                break;
                            }
                            // other source does not detect this object
                            CheckConflict = 1;
                        }
                        
                        std::cout << "+++______SenderID:" << EmitterID << "; SourceID:" << SourceID << "; objectID:" << ReportedNodeID << "; CheckConflict:" << CheckConflict << std::endl;


                        // add conflict event in table
                        if(CheckConflict){
                            double include,ConflictEventID;
                            tie(include,ConflictEventID) = mConflictEventTable.IncludeEvent(ReportedNodeID);
                            if(include){
                                if(!mConflictEventTable.cTable[ConflictEventID].EventInfo.include(SourceID)){
                                    mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(SourceID);
                                    xt::xarray<double> temp = NTT.getNodeTrust(SourceID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp2 = 1 - xt::sum(temp)();
                                    NTT.getNodeTrust(SourceID)->TrustOpinion.SetAdvice(temp);
                                    NTT.getNodeTrust(SourceID)->TrustOpinion.SetU(temp2);
                                    NTT.getNodeTrust(SourceID)->NodeType = 0;
                                    NTT.getNodeTrust(SourceID)->InConflict += 1;
                                }

                                if(!mConflictEventTable.cTable[ConflictEventID].EventInfo.include(EmitterID)){
                                    mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(EmitterID);
                                    xt::xarray<double> temp = NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp2 = 1 - xt::sum(temp)();
                                    NTT.getNodeTrust(EmitterID)->TrustOpinion.SetAdvice(temp);
                                    NTT.getNodeTrust(EmitterID)->TrustOpinion.SetU(temp2);
                                    NTT.getNodeTrust(EmitterID)->NodeType = 0;
                                    NTT.getNodeTrust(EmitterID)->InConflict += 1;
                                }

                            }else{
                                vector<uint32_t> ConflictSourceAssociated = {EmitterID,SourceID};
                                ConflictEventInfo newConflict(ReportedNodeID, OI_type, ConflictSourceAssociated);
                                mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
                                xt::xarray<double> temp = NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice() * 0.9;
                                double temp2 = 1 - xt::sum(temp)();
                                NTT.getNodeTrust(EmitterID)->TrustOpinion.SetAdvice(temp);
                                NTT.getNodeTrust(EmitterID)->TrustOpinion.SetU(temp2);
                                NTT.getNodeTrust(EmitterID)->NodeType = 0;
                                NTT.getNodeTrust(EmitterID)->InConflict += 1;

                                xt::xarray<double> temp3 = NTT.getNodeTrust(SourceID)->TrustOpinion.GetAdvice() * 0.9;
                                double temp4 = 1 - xt::sum(temp3)();
                                NTT.getNodeTrust(SourceID)->TrustOpinion.SetAdvice(temp3);
                                NTT.getNodeTrust(SourceID)->TrustOpinion.SetU(temp4);
                                NTT.getNodeTrust(SourceID)->NodeType = 0;
                                NTT.getNodeTrust(SourceID)->InConflict += 1;
                            }
                        }
                    }
                }
            }
        }

    }

    detected:
    if(InLocalPerception(EmitterID,lem) && (( EmitterLocalBonus & 0b01)== 0)){
        tie(inNTT,NTTdis) = NTT.include(EmitterID);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(EmitterID)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(EmitterID)->TrustOpinion.SetU(1-a);
            NTT.getNodeTrust(EmitterID)->NodeType = 1;
            NTT.getNodeTrust(EmitterID)->LocalBonus |= 0b01;
        }
    }

     
    if(InLocalPerception(stationAttacker,lem) && (conflictInclude==true)){
        tie(inNTT,NTTdis) = NTT.include(stationAttacker);
        if(inNTT){           
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetAdvice({0,1});
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetU(0);
            NTT.getNodeTrust(stationAttacker)->NodeType = 0;
        }
    }

    tie(include,ConflictEventID) = IncludeEvent(stationOmit);
    if(include){
        cout << "NTT Station < 0.5 :::::"<< NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice()[0] << endl;  
        // auto TmConflictEventStatus = this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus;


        if((NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice()[0] < 0.1) 
        &&( this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress)){
            cout << "mConflictEventStatus success"<< endl;
            this->mConflictEventTable.setConflictEventStatus(ConflictEventID,Stop);
            NTT.getNodeTrust(EmitterID)->NodeType = 0;
            cout << "mConflictEventStatus success get status:::"<< this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus << endl;
        }
        
        if(this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            int gszie = g.size();
            for(int i=0; i < gszie; i++){
                NTT.getNodeTrust(g[i])->NodeType = 0;
            }
            
        }   

        if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (recoverFlag == 0)){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
              
                uint32_t OtherID = g[i];
                if(g[i]!=stationAttacker && g[i]!=egoID && g[i]!=stationEgo){
                    xt::xarray<double> temp = NTT.getNodeTrust(OtherID)->TrustOpinion.GetAdvice() / 0.9;
                    NTT.getNodeTrust(OtherID)->TrustOpinion.SetAdvice(temp);
                    double a = temp(0);
                    NTT.getNodeTrust(OtherID)->TrustOpinion.SetU(1-a);
                    recoverFlag++;
                }
            }

        }
        
        
    }


    for(int i = 0 ; i < NTTtable.size(); i++){
        if(NTTtable[i].StationType == StationTypeTrustInit_RoadSideUnit){
            continue;
        }
        auto tempStationID = NTTtable[i].stationID;
        double tempType = NTT.getNodeTrust(tempStationID)->NodeType;
        if(tempStationID == EmitterID){
            if(tempType == 1){
                double a = 0.1;
                double b = -1;
                double y = 8;

                double nTime = NTT.getNodeTrust(tempStationID)->RCtime;
                NTT.getNodeTrust(tempStationID)->RCtime = nTime+1;
                double mRCtime = NTT.getNodeTrust(tempStationID)->RCtime;
                double tempval1 = round(AssessmentFunction(mRCtime,a,b,y)*1000)/1000;
                double tempval2 = round(AssessmentFunction(mRCtime-1,a,b,y)*1000)/1000;
                xt::xarray<double> tempval = {tempval1 - tempval2,0};

                xt::xarray<double> temp = NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice() + tempval;
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetU(1-u);
            }
        }
    }

}


/**
 * @brief specific setting of Omission attack senario with Rsu (for RSU)
 */

void CPMConflictCheck::updateConflictCheckRsuOR(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem, NodeTrustTable& NTT, NodeTableCpm& TableCpm, 
NodeTypeClassification& NTC,std::shared_ptr<traci::API>* TraciLiteAPI, const Identity* mIdentity,const GeoPosition* mGeoPosition){

    bool include;
    bool inNTT;
    int ConflictEventID;
    int NTTdis;
    double EmitterHeading;
    libsumo::TraCIPosition transEmitterLongLat2XY;
    double SensorRangeX1,SensorRangeY1,SensorRangeX2,SensorRangeY2;

    uint32_t EmitterID = msg->header.stationID;
    uint32_t egoID = mIdentity->application%100;
    auto PerceivedSize = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
    

    // injection object ID
    uint32_t stationOmit = 78;

    uint32_t stationEgo = 27;
    uint32_t stationAttacker = 24;
    uint32_t station1 = 80;
    uint32_t stationRsu = 60;

    double EmitterLon = msg->cpm.cpmParameters.managementContainer.referencePosition.longitude;
    double EmitterLat = msg->cpm.cpmParameters.managementContainer.referencePosition.latitude;
    auto CPMStationType = msg->cpm.cpmParameters.managementContainer.stationType;
    if (CPMStationType == StationType_passengerCar){
        EmitterHeading = msg->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
        transEmitterLongLat2XY = (*TraciLiteAPI)->convert2D({EmitterLon/1e7,EmitterLat/1e7});
        SensorRangeX1 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading-200)*PI/1800);
        SensorRangeY1 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading-200)*PI/1800);

        SensorRangeX2 = transEmitterLongLat2XY.x + 65 * sin((EmitterHeading+200)*PI/1800);
        SensorRangeY2 = transEmitterLongLat2XY.y + 65 * cos((EmitterHeading+200)*PI/1800);

    }else if(CPMStationType == StationType_roadSideUnit){
        EmitterHeading = 900;
        transEmitterLongLat2XY = (*TraciLiteAPI)->convert2D({EmitterLon/1e7,EmitterLat/1e7});
        SensorRangeX1 = transEmitterLongLat2XY.x + 200 * sin((EmitterHeading-100)*PI/1800);
        SensorRangeY1 = transEmitterLongLat2XY.y + 200 * cos((EmitterHeading-100)*PI/1800);

        SensorRangeX2 = transEmitterLongLat2XY.x + 200 * sin((EmitterHeading+100)*PI/1800);
        SensorRangeY2 = transEmitterLongLat2XY.y + 200 * cos((EmitterHeading+100)*PI/1800);        
    }
    artery::geometry::Point Emitpos(transEmitterLongLat2XY.x,transEmitterLongLat2XY.y);
    artery::geometry::Point SensorPoint1(SensorRangeX1,SensorRangeY1);
    artery::geometry::Point SensorPoint2(SensorRangeX2,SensorRangeY2);
    artery::geometry::Polygon EmitSensorPolygon{{Emitpos,SensorPoint1,
                SensorPoint2,Emitpos}};

    artery::geometry::Point ObjPos;
    auto EgoXY = (*TraciLiteAPI)->convert2D({mGeoPosition->longitude.value(),mGeoPosition->latitude.value()});
    double EgoHeading = 90;
    double EgoSensorRangeX1 = EgoXY.x + 65 * sin((EgoHeading-20)*PI/180);
    double EgoSensorRangeY1 = EgoXY.y + 65 * cos((EgoHeading-20)*PI/180);

    double EgoSensorRangeX2 = EgoXY.x + 65 * sin((EgoHeading+20)*PI/180);
    double EgoSensorRangeY2 = EgoXY.y + 65 * cos((EgoHeading+20)*PI/180);
    artery::geometry::Point EgoPos(EgoXY.x,EgoXY.y);
    artery::geometry::Point EgoSensorPoint1(EgoSensorRangeX1,EgoSensorRangeY1);
    artery::geometry::Point EgoSensorPoint2(EgoSensorRangeX2,EgoSensorRangeY2);
    artery::geometry::Polygon EgoSensorPolygon{{EgoPos,EgoSensorPoint1,
                EgoSensorPoint2,EgoPos}};


    double LocalObjectXpos;
    double LocalObjectYpos;
    int EmitterLocalBonus;


    auto NTTtable = NTT.getTrustTable();
    for(int i = 0 ; i < NTTtable.size(); i++){
        auto tempStationID = NTTtable[i].stationID;
        double tempValidity = NTC.GetNodeState(tempStationID).NodeValidity;
        NTT.getNodeTrust(tempStationID)->NodeType =  tempValidity;
        if(NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice()[0] < 0.1){
            NTT.getNodeTrust(tempStationID)->NodeType = 0;
        }
    }

    if(std::get<0>(NTT.include(EmitterID))){
        EmitterLocalBonus = NTT.getNodeTrust(EmitterID)->LocalBonus;
    }
    // Source Reliability Verification Module
    const auto& localObj = lem.allObjects();
    const TrackedObjectsFilterRange& TrackedBbjs = filterBySensorCategory(localObj, "Radar");


    for(const auto& i: TrackedBbjs){
        std::weak_ptr<EnvironmentModelObject> obj_ptr = i.first;  
        const VehicleDataProvider& vd = obj_ptr.lock()->getVehicleData();
        auto LocalPerceivedObjID = vd.getStationId()%100;
        if (LocalPerceivedObjID == EmitterID){
            continue;
        }else{
            auto LocalObjectXYpos = (*TraciLiteAPI)->convert2D({vd.longitude().value(),vd.latitude().value()});
            LocalObjectXpos = LocalObjectXYpos.x;
            LocalObjectYpos = LocalObjectXYpos.y;
            artery::geometry::Point LocalObjPos(LocalObjectXpos,LocalObjectYpos);

            bool LocalObjTest = IsPointInPolygon(LocalObjPos,EmitSensorPolygon);
            if(LocalObjTest){
                if(PerceivedSize == 0){
                    conflictInclude = true;
                    goto detected;
                }else{
                    for(int indxPerceivedObj = 0; indxPerceivedObj < PerceivedSize ; indxPerceivedObj++){
                        auto ReportedNodeID = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[indxPerceivedObj]->objectID;
                        if(LocalPerceivedObjID == ReportedNodeID){
                            conflictInclude = false;
                            break;
                        }
                        conflictInclude = true;
                    }

                    if(conflictInclude){
                        goto detected;
                    }
                }
            }
        }
    }


    // Perception Data reliability Verification Module
    for(int indxPerceivedObj = 0; indxPerceivedObj < PerceivedSize ; indxPerceivedObj++){
        auto Obj = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[indxPerceivedObj];
        auto ReportedNodeID = Obj->objectID;
        double ObjPosX = transEmitterLongLat2XY.x + (Obj->xDistance.value)/100;
        double ObjPosY = transEmitterLongLat2XY.y + (Obj->yDistance.value)/100;
        double ObjSpeedX = Obj->xSpeed.value/100;
        double ObjSpeedY = Obj->ySpeed.value/100;

        artery::geometry::Point ObjPos(ObjPosX + ObjSpeedX*1,ObjPosY + ObjSpeedY*1);

        bool IsInLocalPerception = InLocalPerception(ReportedNodeID,lem);

        if(IsInLocalPerception){
            // Kinematic data checks
        
        }else{



            // Reported object, is in other sources perception range ?
            int SourceNum = TableCpm.getNodesNum();
            
            for(int SourceIndx = 0; SourceIndx < SourceNum; SourceIndx++){
                uint32_t SourceID = TableCpm.getNodeID(SourceIndx);
                if((SourceID != EmitterID) && (SourceID != ReportedNodeID)){
                   
                    vanetza::asn1::Cpm* LastCPM = TableCpm.getNodeHistoryAddr(SourceID)->getLatestCPMAddr();
                    int PerceivedSize = (*LastCPM)->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
                    double LastNodeLat = (*LastCPM)->cpm.cpmParameters.managementContainer.referencePosition.latitude;
                    double LastNodeLog = (*LastCPM)->cpm.cpmParameters.managementContainer.referencePosition.longitude;
                    auto SourceStationType = (*LastCPM)->cpm.cpmParameters.managementContainer.stationType;
                    double SourceHeading;
                    double SourceSensorRange,SourceSensorAngle;

                    // sensor Info
                    if(SourceStationType == StationType_passengerCar){
                        SourceHeading  = (*LastCPM)->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
                        SourceSensorRange = 65;
                        SourceSensorAngle = 200;
                    }else if(SourceStationType ==  StationType_roadSideUnit){
                        SourceHeading  = 900;
                        SourceSensorRange = 200;
                        SourceSensorAngle = 100;
                    }
                    
                    auto transLongLatXY = (*TraciLiteAPI)->convert2D({LastNodeLog/1e7,LastNodeLat/1e7});
                    
                    double SensorRangeX1 = transLongLatXY.x + SourceSensorRange * sin((SourceHeading-SourceSensorAngle)*PI/1800);
                    double SensorRangeY1 = transLongLatXY.y + SourceSensorRange * cos((SourceHeading-SourceSensorAngle)*PI/1800);

                    double SensorRangeX2 = transLongLatXY.x + SourceSensorRange * sin((SourceHeading+SourceSensorAngle)*PI/1800);
                    double SensorRangeY2 = transLongLatXY.y + SourceSensorRange * cos((SourceHeading+SourceSensorAngle)*PI/1800);

                    artery::geometry::Point otherVPos(transLongLatXY.x,transLongLatXY.y);
                    artery::geometry::Point SensorRangePos1(SensorRangeX1,SensorRangeY1);
                    artery::geometry::Point SensorRangePos2(SensorRangeX2,SensorRangeY2);
                    std::cout << "otherVPos:::::"<< bg::get<0>(otherVPos) << ","<< bg::get<1>(otherVPos) <<std::endl;
                    std::cout << "SensorRangePos1:::::"<< bg::get<0>(SensorRangePos1) << ","<< bg::get<1>(SensorRangePos1) <<std::endl;
                    std::cout << "SensorRangePos2:::::"<< bg::get<0>(SensorRangePos2) << ","<< bg::get<1>(SensorRangePos2) <<std::endl;

                    // Sensor Polygon
                    artery::geometry::Polygon SensorPolygon{{otherVPos,SensorRangePos1,
                                SensorRangePos2,otherVPos}};
                    // artery::geometry::Polygon SensorPolygon{{{0,0},{100,0},{100,100},{0,100},{0,0}}};

                
                    // check if point in the other source's sensor range
                    bool IsInSensorRange = IsPointInPolygon(ObjPos,SensorPolygon);
                    if(SourceID == stationEgo){
                        IsInSensorRange = false;
                    }



                    std:cout << "______SenderID:" << EmitterID << "; SourceID:" << SourceID<< "; objectID:" << ReportedNodeID << ";InSR?"<<IsInSensorRange << std::endl;
                    if(IsInSensorRange){
                        if(PerceivedSize == 0){
                            this->CheckConflict = 1;
                        }
                        for(int nodePerceivedObjIndx = 0; nodePerceivedObjIndx < PerceivedSize; nodePerceivedObjIndx++){
                            if((*LastCPM)->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[nodePerceivedObjIndx]->objectID == ReportedNodeID){
                                // other source detects this object
                                this->CheckConflict = 0;
                                break;
                            }
                            // other source does not detect this object
                            this->CheckConflict = 1;
                        }
                        
                        std::cout << "+++______SenderID:" << EmitterID << "; SourceID:" << SourceID << "; objectID:" << ReportedNodeID << "; CheckConflict:" << CheckConflict << std::endl;


                        // add conflict event in table
                        if(this->CheckConflict){
                            double include,ConflictEventID;
                            tie(include,ConflictEventID) = mConflictEventTable.IncludeEvent(ReportedNodeID);
                            if(include){
                                if(!mConflictEventTable.cTable[ConflictEventID].EventInfo.include(SourceID)){
                                    mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(SourceID);
                                    xt::xarray<double> temp = NTT.getNodeTrust(SourceID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp2 = 1 - xt::sum(temp)();
                                    NTT.getNodeTrust(SourceID)->TrustOpinion.SetAdvice(temp);
                                    NTT.getNodeTrust(SourceID)->TrustOpinion.SetU(temp2);
                                    NTT.getNodeTrust(SourceID)->NodeType = 0;
                                    NTT.getNodeTrust(SourceID)->InConflict += 1;
                                }

                                if(!mConflictEventTable.cTable[ConflictEventID].EventInfo.include(EmitterID)){
                                    mConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(EmitterID);
                                    xt::xarray<double> temp = NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp2 = 1 - xt::sum(temp)();
                                    NTT.getNodeTrust(EmitterID)->TrustOpinion.SetAdvice(temp);
                                    NTT.getNodeTrust(EmitterID)->TrustOpinion.SetU(temp2);
                                    NTT.getNodeTrust(EmitterID)->NodeType = 0;
                                    NTT.getNodeTrust(EmitterID)->InConflict += 1;
                                }

                            }else{
                                vector<uint32_t> ConflictSourceAssociated = {EmitterID,SourceID};
                                ConflictEventInfo newConflict(ReportedNodeID, OI_type, ConflictSourceAssociated);
                                mConflictEventTable.addElementToConflictEventInfoTable(newConflict);
                                xt::xarray<double> temp = NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice() * 0.9;
                                double temp2 = 1 - xt::sum(temp)();
                                NTT.getNodeTrust(EmitterID)->TrustOpinion.SetAdvice(temp);
                                NTT.getNodeTrust(EmitterID)->TrustOpinion.SetU(temp2);
                                NTT.getNodeTrust(EmitterID)->NodeType = 0;
                                NTT.getNodeTrust(EmitterID)->InConflict += 1;

                                xt::xarray<double> temp3 = NTT.getNodeTrust(SourceID)->TrustOpinion.GetAdvice() * 0.9;
                                double temp4 = 1 - xt::sum(temp3)();
                                NTT.getNodeTrust(SourceID)->TrustOpinion.SetAdvice(temp3);
                                NTT.getNodeTrust(SourceID)->TrustOpinion.SetU(temp4);
                                NTT.getNodeTrust(SourceID)->NodeType = 0;
                                NTT.getNodeTrust(SourceID)->InConflict += 1;
                            }
                        }
                    }
                }
            }
        }

    }
    
    detected:   
    if(InLocalPerception(EmitterID,lem) && (( EmitterLocalBonus & 0b01)== 0)){
        tie(inNTT,NTTdis) = NTT.include(EmitterID);
        if(inNTT){
            xt::xarray<double> temp = {0.2,0};
            xt::xarray<double> temp2 = NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice() + temp;
            NTT.getNodeTrust(EmitterID)->TrustOpinion.SetAdvice(temp2);
            double a = temp2(0);
            NTT.getNodeTrust(EmitterID)->TrustOpinion.SetU(1-a);
            NTT.getNodeTrust(EmitterID)->NodeType = 1;
            NTT.getNodeTrust(EmitterID)->LocalBonus |= 0b01;
        }
    }

 
    if(InLocalPerception(stationAttacker,lem) && (conflictInclude==true)){
        tie(inNTT,NTTdis) = NTT.include(stationAttacker);
        if(inNTT){           
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetAdvice({0,1});
            NTT.getNodeTrust(stationAttacker)->TrustOpinion.SetU(0);
            NTT.getNodeTrust(stationAttacker)->NodeType = 0;
        }
    }



    tie(include,ConflictEventID) = IncludeEvent(stationOmit);
    if(include){
        cout << "NTT Station < 0.5 :::::"<< NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice()[0] << endl;  
        // auto TmConflictEventStatus = this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus;


        if((NTT.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice()[0] < 0.1) 
        &&( this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress)){
            cout << "mConflictEventStatus success"<< endl;
            this->mConflictEventTable.setConflictEventStatus(ConflictEventID,Stop);
            NTT.getNodeTrust(EmitterID)->NodeType = 0;
            cout << "mConflictEventStatus success get status:::"<< this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus << endl;
        }
        
        if(this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == InProgress){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            int gszie = g.size();
            for(int i=0; i < gszie; i++){
                NTT.getNodeTrust(g[i])->NodeType = 0;
            }
            
        }   

        if((this->mConflictEventTable.cTable[ConflictEventID].mConflictEventStatus == Stop) && (recoverFlag == 0)){
            auto g = this->mConflictEventTable.cTable[ConflictEventID].EventInfo.getConflictSourceAssociated();
            for(int i=0; i < g.size(); i++){
              
                uint32_t OtherID = g[i];
                if(g[i]!=stationAttacker && g[i]!=egoID && g[i]!=stationEgo){
                    xt::xarray<double> temp = NTT.getNodeTrust(OtherID)->TrustOpinion.GetAdvice() / 0.9;
                    NTT.getNodeTrust(OtherID)->TrustOpinion.SetAdvice(temp);
                    double a = temp(0);
                    NTT.getNodeTrust(OtherID)->TrustOpinion.SetU(1-a);
                    this->recoverFlag++;
                }
            }

        }
        
        
    }


    for(int i = 0 ; i < NTTtable.size(); i++){
        if(NTTtable[i].StationType == StationTypeTrustInit_RoadSideUnit){
            continue;
        }
        auto tempStationID = NTTtable[i].stationID;
        double tempType = NTT.getNodeTrust(tempStationID)->NodeType;
        if(tempStationID == EmitterID){
            if(tempType == 1){
                double a = 0.1;
                double b = -1;
                double y = 8;

                double nTime = NTT.getNodeTrust(tempStationID)->RCtime;
                NTT.getNodeTrust(tempStationID)->RCtime = nTime+1;
                double mRCtime = NTT.getNodeTrust(tempStationID)->RCtime;
                double tempval1 = round(AssessmentFunction(mRCtime,a,b,y)*1000)/1000;
                double tempval2 = round(AssessmentFunction(mRCtime-1,a,b,y)*1000)/1000;
                xt::xarray<double> tempval = {tempval1 - tempval2,0};

                xt::xarray<double> temp = NTT.getNodeTrust(tempStationID)->TrustOpinion.GetAdvice() + tempval;
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                NTT.getNodeTrust(tempStationID)->TrustOpinion.SetU(1-u);
            }
        }
    }

}


















// ******************************************************************************************************************
// ***********************  Trust Management Framework using Subjective Logic ***************************************
// ******************************************************************************************************************


// --------------------------------------- Trust Revision L1 --------------------------------------------------------
// -----------------------------     Environment Factors Assessment       -------------------------------------------

void CPMConflictCheck::TrustRevisionL1(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem, const VehicleDataProvider* mVehicleDataProvider,
NodeTrustTable& myNodeTrustTable,NodeTableCpm& detectedNodes, CPMConflictCheck& myConflictCheck, NodeTypeClassification& myNodeTypeClassification,F2MD_CPParameters* params){

    uint32_t senderID = msg->header.stationID;
    StationTypeTrustInit mStationTypeTrustInit;
    bool checkN = false;
    bool newNodeType;

    if(msg->cpm.cpmParameters.managementContainer.stationType == StationType_passengerCar){
        mStationTypeTrustInit = StationTypeTrustInit_PassengerCar;
    }else if(msg->cpm.cpmParameters.managementContainer.stationType == StationType_roadSideUnit){
        mStationTypeTrustInit = StationTypeTrustInit_RoadSideUnit;
    }


    // if N > 1 ?
    if (!detectedNodes.includes(senderID)) {
        NodeHistoryCpm newNode(senderID);
        newNode.addCPM(msg);
        detectedNodes.put(senderID, newNode, params->MAX_CPM_HISTORY_TIME);
        myNodeTrustTable.addNode(senderID,mStationTypeTrustInit);
    }
    else {
        NodeHistoryCpm* existingNode = detectedNodes.getNodeHistoryAddr(
            senderID);
        existingNode->addCPM(msg);
        checkN = true;
    }
    // if NodeType == Valid
    auto checkV = myNodeTrustTable.getNodeTrust(senderID)->NodeType;
    // if Node in conflict?
    auto checkC = myNodeTrustTable.getNodeTrust(senderID)->InConflict;

    // check N && V
    if(checkN && checkV){

        if(myConflictCheck.InLocalPerception(senderID,lem)){
            if((myNodeTrustTable.getNodeTrust(senderID)->LocalBonus & 0b01)== 0){
                xt::xarray<double> temp = {0.2,0};
                xt::xarray<double> temp2 = myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.GetAdvice() + temp;
                myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetAdvice(temp2);
                double a = temp2(0);
                myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetU(1-a);
                myNodeTrustTable.getNodeTrust(senderID)->LocalBonus |= 0b01;
            }
        }

        // Penalty & Assessment Function Parameters
        double a = 0.1;
        double b = -1;
        double y = 8;

        double nTime = myNodeTrustTable.getNodeTrust(senderID)->RCtime;
        myNodeTrustTable.getNodeTrust(senderID)->RCtime = nTime+1;
        double mRCtime = myNodeTrustTable.getNodeTrust(senderID)->RCtime;
        double tempval1 = round(myTrustManagementFunc.AssessmentFunction(mRCtime,a,b,y)*1000)/1000;
        double tempval2 = round(myTrustManagementFunc.AssessmentFunction(mRCtime-1,a,b,y)*1000)/1000;
        xt::xarray<double> tempval = {tempval1 - tempval2,0};

        xt::xarray<double> temp = myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.GetAdvice() + tempval;
        myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetAdvice(temp);
        double u = temp(0);
        myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetU(1-u);
        myNodeTrustTable.getNodeTrust(senderID)->LocalBonus |= 0b10;
        myNodeTrustTable.getNodeTrust(senderID)->TrustEnvState = tempval1;

        int mLocalBonus = myNodeTrustTable.getNodeTrust(senderID)->LocalBonus;
        if((mLocalBonus & 0b01) == 0b01){
            if(!(myConflictCheck.InLocalPerception(senderID,lem))){
                double a = 0.04;
                double b = -0.08;
                double n = myNodeTrustTable.getNodeTrust(senderID)->LostTime;
                myNodeTrustTable.getNodeTrust(senderID)->LostTime++;
                double tempval1 = myTrustManagementFunc.PenaltyFunction1(n,a,b,0,0.2);
                double tempval2 = myTrustManagementFunc.PenaltyFunction1(n-1,a,b,0,0.2);
                xt::xarray<double> tempval = {tempval1 - tempval2,0};
                xt::xarray<double> temp = myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.GetAdvice() - tempval;
                myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetAdvice(temp);
                double u = temp(0);
                myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetU(1-u);
                if(tempval1 >= 0.2){
                    // Complete lost the target, reset
                    myNodeTrustTable.getNodeTrust(senderID)->LostTime = 0;
                    myNodeTrustTable.getNodeTrust(senderID)->LocalBonus &= ~(0b01);
                }
            }else{
                if(myNodeTrustTable.getNodeTrust(senderID)->LostTime > 0){
                    myNodeTrustTable.getNodeTrust(senderID)->LostTime--;
                    double a = 0.04;
                    double b = -0.08;
                    double n = myNodeTrustTable.getNodeTrust(senderID)->LostTime;
                    double tempval1 = myTrustManagementFunc.PenaltyFunction1(n,a,b,0,0.2);
                    double tempval2 = myTrustManagementFunc.PenaltyFunction1(n-1,a,b,0,0.2);
                    xt::xarray<double> tempval = {tempval1 - tempval2,0};
                    xt::xarray<double> temp = myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.GetAdvice() + tempval;
                    myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetAdvice(temp);
                    double u = temp(0);
                    myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetU(1-u);
                }
            }
        }

    }else{
        if(checkC == 0){
            int mLocalBonus = myNodeTrustTable.getNodeTrust(senderID)->LocalBonus;
            if((mLocalBonus & 0b01) == 0b01){
                if(!(myConflictCheck.InLocalPerception(senderID,lem))){
                    double a = 0.04;
                    double b = -0.08;
                    double n = myNodeTrustTable.getNodeTrust(senderID)->LostTime;
                    myNodeTrustTable.getNodeTrust(senderID)->LostTime++;
                    double tempval1 = myTrustManagementFunc.PenaltyFunction1(n,a,b,0,0.2);
                    double tempval2 = myTrustManagementFunc.PenaltyFunction1(n-1,a,b,0,0.2);
                    xt::xarray<double> tempval = {tempval1 - tempval2,0};
                    xt::xarray<double> temp = myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.GetAdvice() - tempval;
                    myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetAdvice(temp);
                    double u = temp(0);
                    myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetU(1-u);
                    if(tempval1 >= 0.2){
                        // Complete lost the target, reset
                        myNodeTrustTable.getNodeTrust(senderID)->LostTime = 0;
                        myNodeTrustTable.getNodeTrust(senderID)->LocalBonus &= ~(0b01);
                    }
                }
            }
            if((mLocalBonus & 0b10) == 0b10){
                if(checkV==false){
                    double a = 0.1;
                    double b = -1;
                    double y = 8;
                    double nTime = myNodeTrustTable.getNodeTrust(senderID)->InvalidTime;
                    double AFval = myNodeTrustTable.getNodeTrust(senderID)->TrustEnvState;
                    myNodeTrustTable.getNodeTrust(senderID)->InvalidTime = nTime+1;

                    double mInvalidTime = myNodeTrustTable.getNodeTrust(senderID)->InvalidTime;
                    double tempval1 = round(myTrustManagementFunc.PenaltyFunction2(mInvalidTime,a,b,y,AFval)*1000)/1000;
                    double tempval2 = round(myTrustManagementFunc.PenaltyFunction2(mInvalidTime-1,a,b,y,AFval)*1000)/1000;
                    xt::xarray<double> tempval = {tempval1 - tempval2,0};

                    xt::xarray<double> temp = myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.GetAdvice() + tempval;
                    myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetAdvice(temp);
                    double u = temp(0);
                    myNodeTrustTable.getNodeTrust(senderID)->TrustOpinion.SetU(1-u);
                    if(tempval1 <= (-AFval)){
                            // Complete lost the target, reset
                            myNodeTrustTable.getNodeTrust(senderID)->InvalidTime = 0;
                            myNodeTrustTable.getNodeTrust(senderID)->RCtime = 0;
                            myNodeTrustTable.getNodeTrust(senderID)->LocalBonus &= ~(0b10);
                    }
                }
            }
        }
    }


    myNodeTypeClassification.NodeTypeClassificationUpdate(mVehicleDataProvider,msg,lem, detectedNodes);
    newNodeType = myNodeTypeClassification.GetNodeState(senderID).NodeValidity;
    myNodeTrustTable.setNodeTrust(senderID, 10, (bool)newNodeType);
}




// ********************************************************************************************************************



// --------------------------------------- Trust Revision L2 --------------------------------------------------------
// -------------------------------     Misbehavior Detection Module       -------------------------------------------
void CPMConflictCheck::TrustRevisionL2(const vanetza::asn1::Cpm& msg, const LocalEnvironmentModel& lem, const traci::VehicleController* VehController,
const VehicleDataProvider* mVehicleDataProvider,NodeTrustTable& myNodeTrustTable, NodeTableCpm& detectedNodes, CPMConflictCheck& myConflictCheck
,ConflictEventTable& myConflictEventTable, CpmChecks mCpmChecks, GlobalEnvironmentModel& mGlobalEnvironmentModel){

    uint32_t EmitterID = msg->header.stationID;
    auto PerceivedSize = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
    double EmitterLat = msg->cpm.cpmParameters.managementContainer.referencePosition.latitude;
    double EmitterLog = msg->cpm.cpmParameters.managementContainer.referencePosition.longitude; 
    double EmitterHeading;
    if(msg->cpm.cpmParameters.managementContainer.stationType == StationType_passengerCar){
        EmitterHeading = msg->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
    }else if(msg->cpm.cpmParameters.managementContainer.stationType == StationType_roadSideUnit){
        EmitterHeading = 900;
    }
     
    std::cout << "EmitterID:::::"<< EmitterID <<std::endl;
    std::cout << "EmitterPosLatLog:::::"<< EmitterLat << ","<< EmitterLog <<std::endl;   
    auto transEmitterLongLat2XY = VehController->getTraCI()->convert2D({EmitterLog/1e7,EmitterLat/1e7});
    std::cout << "EmitterPos:::::"<< transEmitterLongLat2XY.x << ","<< transEmitterLongLat2XY.y <<std::endl;
    std::cout << "EmitterHeading:::::"<< EmitterHeading <<std::endl;

    uint32_t EgoID = mVehicleDataProvider->station_id()%100;
    auto EgoXY = VehController->getTraCI()->convert2D({mVehicleDataProvider->longitude().value(),mVehicleDataProvider->latitude().value()});
    double EgoHeading = 180 * mVehicleDataProvider->heading().value() / PI;
    double EgoSensorRangeX1 = EgoXY.x + 65 * sin((EgoHeading-20)*PI/180);
    double EgoSensorRangeY1 = EgoXY.y + 65 * cos((EgoHeading-20)*PI/180);

    double EgoSensorRangeX2 = EgoXY.x + 65 * sin((EgoHeading+20)*PI/180);
    double EgoSensorRangeY2 = EgoXY.y + 65 * cos((EgoHeading+20)*PI/180);
    artery::Position EgoPosition(EgoXY.x,EgoXY.y);
    artery::geometry::Point EgoPos(EgoXY.x,EgoXY.y);
    artery::geometry::Point EgoSensorPoint1(EgoSensorRangeX1,EgoSensorRangeY1);
    artery::geometry::Point EgoSensorPoint2(EgoSensorRangeX2,EgoSensorRangeY2);
    artery::geometry::Polygon EgoSensorPolygon{{EgoPos,EgoSensorPoint1,
                EgoSensorPoint2,EgoPos}};

    std::cout << "EgoID:::::"<< EgoID <<std::endl;

    bool checkfaild = 0;
    // CpmChecks CpmChecks(params, MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL, MAX_PLAUSIBLE_DECEL);
    // Source Reliability Verification Module
    if(checkfaild){
        goto DetectFaild;
    }


    // Perception Data reliability Verification Module
    for(int i = 0 ; i < PerceivedSize; i++){
        auto Obj = msg->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[i];
        auto ReportedNodeID = Obj->objectID;

        double ReportedNodeHeading = Obj->yawAngle->value * PI/1800;
        auto ObjPosXY = myTrustManagementFunc.InvXYConvert( (Obj->xDistance.value)/100, (Obj->yDistance.value)/100, 
        transEmitterLongLat2XY.x, transEmitterLongLat2XY.y, EmitterHeading*PI/1800);
        double ObjPosX = ObjPosXY(0);
        double ObjPosY = ObjPosXY(1);
        double ObjSpeedX = Obj->xSpeed.value/100;
        double ObjSpeedY = Obj->ySpeed.value/100;
        // artery::geometry::Point ObjPos(ObjPosX,ObjPosY);
        // point position 
        artery::geometry::Point ObjPos(ObjPosX - ObjSpeedX*0,ObjPosY - ObjSpeedY*0);
        std::cout << "ObjID:::::"<< ReportedNodeID <<std::endl;
        std::cout << "ObjPos:::::"<< bg::get<0>(ObjPos) << ","<< bg::get<1>(ObjPos) <<std::endl;

        // For ego, should the reported object in local perception ?
        bool ShouldInLocalPerception = IsPointInPolygon(ObjPos,EgoSensorPolygon);
        // For ego, is the reported object in local perception ?
        bool IsInLocalPerception = myConflictCheck.InLocalPerception(ReportedNodeID,lem);
        bool CheckHidden = false;

        
        auto ReportedObjectOutline = myTrustManagementFunc.PosOfFourCorners(ObjPosX,ObjPosY,ReportedNodeHeading);
        // masking check for the object
        const auto& objects = lem.allObjects();
        for (const auto& obj : objects) { 
            std::weak_ptr<EnvironmentModelObject> obj_ptr = obj.first;  
            if (obj_ptr.expired()) continue; /*< objects remain in tracking briefly after leaving simulation */

            auto object = vanetza::asn1::allocate<PerceivedObject_t>();
            const VehicleDataProvider& vd = obj_ptr.lock()->getVehicleData();
            double objPx = vd.position().x.value();
            double objPy = -vd.position().y.value();
            double objheading = vd.heading().value() * PI /180;
            auto objOutline = myTrustManagementFunc.PosOfFourCorners(objPx,objPy,objheading);
            CheckHidden = CheckHidden | myTrustManagementFunc.CheckTargetHideByObject(ReportedObjectOutline, objOutline, EgoPosition);
        }
        // masking check for the obstacle
        auto ListObstacleSelected = myTrustManagementFunc.ObstacleSelectedByArea(mGlobalEnvironmentModel,200,EgoXY.x,EgoXY.y);
        for(int ListIndx=0; ListIndx <  ListObstacleSelected.size(); ListIndx++){
            auto ListPoints = ListObstacleSelected[ListIndx].get()->getOutline();
            for(auto& EachPoint:ListPoints){
                EachPoint.y = -EachPoint.y;
            }
            CheckHidden = CheckHidden | myTrustManagementFunc.CheckTargetHideByObject(ReportedObjectOutline, ListPoints, EgoPosition);
        }

        if(ShouldInLocalPerception){
            // if the reported object should in local perception but ego can not perceive it. check faild.
            if(!IsInLocalPerception && (ReportedNodeID!=EgoID) && (!CheckHidden)){
                checkfaild = 1;
            }
        }
        std::cout << "CheckHidden:::::"<< CheckHidden <<";; checkfaild:::::"<< checkfaild <<std::endl;
        if(checkfaild){
            goto DetectFaild;
        }
        
        
        if(IsInLocalPerception){
            // Kinematic data checks


        }else{
            std::cout << "stape TR L2 success !" << std::endl;
            // For emitter, Kinematic data checks



            // Reported object, is in other sources perception range ?
            int SourceNum = detectedNodes.getNodesNum();
            std::cout << "stape TR L2 SourceNum:::" << SourceNum << std::endl;

            for(int SourceIndx = 0; SourceIndx < SourceNum; SourceIndx++){
                uint32_t SourceID = detectedNodes.getNodeID(SourceIndx);
                if((SourceID != EmitterID) && (SourceID != ReportedNodeID)){
                   
                    vanetza::asn1::Cpm* LastCPM = detectedNodes.getNodeHistoryAddr(SourceID)->getLatestCPMAddr();
                    int PerceivedSize = (*LastCPM)->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count;
                    double LastNodeLat = (*LastCPM)->cpm.cpmParameters.managementContainer.referencePosition.latitude;
                    double LastNodeLog = (*LastCPM)->cpm.cpmParameters.managementContainer.referencePosition.longitude;
                    double VehicleHeading = (*LastCPM)->cpm.cpmParameters.stationDataContainer->containerData.choice.OriginatingVehicleContainer.heading.headingValue;
                    // double VSpeed = (*LastCPM)->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedValue;
                    // double SpeedX = VSpeed * sin(VehicleHeading * PI / 1800);
                    // double SpeedY = VSpeed * cos(VehicleHeading * PI / 1800);

                    // sensor Info
                    // range 200, angle 20
                    auto transLongLatXY = VehController->getTraCI()->convert2D({LastNodeLog/1e7,LastNodeLat/1e7});
                    // auto temptransLongLatXY2x = transLongLatXY.x + SpeedX * 1;
                    // auto temptransLongLatXY2y = transLongLatXY.y + SpeedY * 1;
                    
                    double SensorRangeX1 = transLongLatXY.x + 65 * sin((VehicleHeading-200)*PI/1800);
                    double SensorRangeY1 = transLongLatXY.y + 65 * cos((VehicleHeading-200)*PI/1800);

                    double SensorRangeX2 = transLongLatXY.x + 65 * sin((VehicleHeading+200)*PI/1800);
                    double SensorRangeY2 = transLongLatXY.y + 65 * cos((VehicleHeading+200)*PI/1800);

                    artery::Position otherVEgoPos(transLongLatXY.x,transLongLatXY.y);
                    artery::geometry::Point otherVPos(transLongLatXY.x,transLongLatXY.y);
                    artery::geometry::Point SensorRangePos1(SensorRangeX1,SensorRangeY1);
                    artery::geometry::Point SensorRangePos2(SensorRangeX2,SensorRangeY2);
                    std::cout << "otherVPos:::::"<< bg::get<0>(otherVPos) << ","<< bg::get<1>(otherVPos) <<std::endl;
                    std::cout << "SensorRangePos1:::::"<< bg::get<0>(SensorRangePos1) << ","<< bg::get<1>(SensorRangePos1) <<std::endl;
                    std::cout << "SensorRangePos2:::::"<< bg::get<0>(SensorRangePos2) << ","<< bg::get<1>(SensorRangePos2) <<std::endl;

                    // Sensor Polygon
                    artery::geometry::Polygon SensorPolygon{{otherVPos,SensorRangePos1,
                                SensorRangePos2,otherVPos}};
                    // artery::geometry::Polygon SensorPolygon{{{0,0},{100,0},{100,100},{0,100},{0,0}}};

                
                    // check if point in the other source's sensor range
                    bool IsInSensorRange = IsPointInPolygon(ObjPos,SensorPolygon);
                    std::cout << "______SenderID:" << EmitterID << "; SourceID:" << SourceID<< "; objectID:" << ReportedNodeID << ";InSR?"<<IsInSensorRange << std::endl;
                    if(IsInSensorRange){
                        bool CheckHidden = false;

                        // check if this reported object is hidden by other object or the obstacle
                        // get the reported object outline
                        auto ReportedOutline = myTrustManagementFunc.PosOfFourCorners(ObjPosX,ObjPosY,ReportedNodeHeading);


                        //check for the object
                        for(int indxPerceivedSize = 0; indxPerceivedSize < PerceivedSize; indxPerceivedSize++){
                            auto PerceivedObj =  (*LastCPM)->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[indxPerceivedSize];
                            double PerceivedObjHeading = (*LastCPM)->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[indxPerceivedSize]->yawAngle->value * PI/1800;
                            double XDis = PerceivedObj->xDistance.value;
                            double YDis = PerceivedObj->yDistance.value;
                            double RadianHeading = VehicleHeading*PI/1800;
                            auto RelatedPos = myTrustManagementFunc.InvXYConvert( XDis, YDis, transLongLatXY.x, transLongLatXY.y, RadianHeading);
                            auto OtherPerceivedObjectOutline = myTrustManagementFunc.PosOfFourCorners(RelatedPos(0),RelatedPos(1),PerceivedObjHeading);
                            CheckHidden = CheckHidden | myTrustManagementFunc.CheckTargetHideByObject(ReportedOutline, OtherPerceivedObjectOutline, otherVEgoPos);
                        }
                        
                        std::cout << "222222222222222222222222222" << std::endl;

                        // check for the obstacle
                        auto ListObstacleSelected = myTrustManagementFunc.ObstacleSelectedByArea(mGlobalEnvironmentModel,200,transLongLatXY.x,-transLongLatXY.y);
                        for(int ListIndx=0; ListIndx <  ListObstacleSelected.size(); ListIndx++){
                            std::cout << "333333333333333333333333333333 ;;;;" << ListObstacleSelected.size() << std::endl;
                            auto ListPoints = ListObstacleSelected[ListIndx].get()->getOutline();
                            std::cout << "444444444444444444444444444444" << std::endl;
                            for(auto& EachPoint:ListPoints){
                                EachPoint.y = -EachPoint.y;
                            }
                            CheckHidden = CheckHidden | myTrustManagementFunc.CheckTargetHideByObject(ReportedOutline, ListPoints, otherVEgoPos);
                        }

                        std::cout << "55555555555555555555555555555555 ;;;;;; "  << CheckHidden << std::endl;

                        if(!CheckHidden){
                            if(PerceivedSize == 0){
                                CheckConflict = 1;
                            }
                            for(int nodePerceivedObjIndx = 0; nodePerceivedObjIndx < PerceivedSize; nodePerceivedObjIndx++){
                                if((*LastCPM)->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[nodePerceivedObjIndx]->objectID == ReportedNodeID){
                                    // other source detects this object
                                    CheckConflict = 0;
                                    break;
                                }
                                // other source does not detect this object
                                CheckConflict = 1;
                            }
                            
                            std::cout << "+++______SenderID:" << EmitterID << "; SourceID:" << SourceID << "; objectID:" << ReportedNodeID << "; CheckConflict:" << CheckConflict << std::endl;


                            // add conflict event in table
                            if(CheckConflict){
                                double include,ConflictEventID;
                                tie(include,ConflictEventID) = myConflictEventTable.IncludeEvent(ReportedNodeID);
                                if(include){
                                    if(!myConflictEventTable.cTable[ConflictEventID].EventInfo.include(SourceID)){
                                        myConflictEventTable.cTable[ConflictEventID].EventInfo.addConflictSourceAssociated(SourceID);
                                        xt::xarray<double> temp = myNodeTrustTable.getNodeTrust(SourceID)->TrustOpinion.GetAdvice() * 0.9;
                                        double temp2 = 1 - xt::sum(temp)();
                                        myNodeTrustTable.getNodeTrust(SourceID)->TrustOpinion.SetAdvice(temp);
                                        myNodeTrustTable.getNodeTrust(SourceID)->TrustOpinion.SetU(temp2);
                                        myNodeTrustTable.getNodeTrust(SourceID)->NodeType = 0;
                                        myNodeTrustTable.getNodeTrust(SourceID)->InConflict += 1;
                                    }
                                }else{
                                    vector<uint32_t> ConflictSourceAssociated = {EmitterID,SourceID};
                                    ConflictEventInfo newConflict(ReportedNodeID, OI_type, ConflictSourceAssociated);
                                    myConflictEventTable.addElementToConflictEventInfoTable(newConflict);
                                    xt::xarray<double> temp = myNodeTrustTable.getNodeTrust(EmitterID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp2 = 1 - xt::sum(temp)();
                                    myNodeTrustTable.getNodeTrust(EmitterID)->TrustOpinion.SetAdvice(temp);
                                    myNodeTrustTable.getNodeTrust(EmitterID)->TrustOpinion.SetU(temp2);
                                    myNodeTrustTable.getNodeTrust(EmitterID)->NodeType = 0;
                                    myNodeTrustTable.getNodeTrust(EmitterID)->InConflict += 1;

                                    xt::xarray<double> temp3 = myNodeTrustTable.getNodeTrust(SourceID)->TrustOpinion.GetAdvice() * 0.9;
                                    double temp4 = 1 - xt::sum(temp3)();
                                    myNodeTrustTable.getNodeTrust(SourceID)->TrustOpinion.SetAdvice(temp3);
                                    myNodeTrustTable.getNodeTrust(SourceID)->TrustOpinion.SetU(temp4);
                                    myNodeTrustTable.getNodeTrust(SourceID)->NodeType = 0;
                                    myNodeTrustTable.getNodeTrust(SourceID)->InConflict += 1;
                                }
                            }
                        }
                    }
                }
            }
        }

    }



    DetectFaild:
        if(checkfaild){
            myNodeTrustTable.getNodeTrust(EmitterID)->TrustOpinion.SetAdvice({0,1});
            myNodeTrustTable.getNodeTrust(EmitterID)->TrustOpinion.SetU(0);
            myNodeTrustTable.getNodeTrust(EmitterID)->NodeType = 0;
            myNodeTrustTable.getNodeTrust(EmitterID)->InConflict += 1;
        }

}


// ********************************************************************************************************************


// --------------------------------------- Trust Revision L3 --------------------------------------------------------
// ----------------------------------     Opinion Fusion Module       ----------------------------------------------
void CPMConflictCheck::TrustRevisionL3(ConflictEventTable& myConflictEventTable, NodeTableCpm& detectedNodes,const VehicleDataProvider* mVehicleDataProvider,
NodeTrustTable& myNodeTrustTable,OpinionConstruction& myOpinionConstruction, ConflictEventTrustRevision myConflictEventTrustRevision){

    myOpinionConstruction.UpdateConflictEventOpinionTable(myConflictEventTable,detectedNodes,mVehicleDataProvider);
    auto myConflictEventOpinionsTable = myOpinionConstruction.getConflictEventOpinionsTable();
    if(myConflictEventOpinionsTable.size() > 0 && myConflictEventOpinionsTable[0].EventStationOpinionsTable.size() > 2){
        myConflictEventTrustRevision.update(&myNodeTrustTable,myOpinionConstruction, myConflictEventTable);
    }

    for(int i =0; i < myConflictEventTable.cTable.size(); i++){
        if(myConflictEventTable.cTable[i].mConflictEventStatus == InProgress){
            bool flagStop = 0;
            auto CSA = myConflictEventTable.cTable[i].EventInfo.getConflictSourceAssociated();
            auto ConflictEventID = myConflictEventTable.cTable[i].ConflictEventID;
            for(int p = 0; p < CSA.size(); p++){
                if(myNodeTrustTable.getNodeTrust(CSA[p])->TrustOpinion.GetAdvice()[0] < 0.1){
                    cout << "mConflictEventStatus success"<< endl;
                    myConflictEventTable.setConflictEventStatus(ConflictEventID,Stop);
                    flagStop = 1;
                    myNodeTrustTable.getNodeTrust(CSA[p])->NodeType = 0;
                    cout << "mConflictEventStatus success get status:::"<< myConflictEventTable.cTable[ConflictEventID].mConflictEventStatus << endl;
                }
            }

            if(flagStop){
                for(int p = 0; p < CSA.size(); p++){
                    if(myNodeTrustTable.getNodeTrust(CSA[p])->TrustOpinion.GetAdvice()[0] > 0.1){
                        myNodeTrustTable.getNodeTrust(CSA[p])->InConflict -= 1;
                    }
                    if(myNodeTrustTable.getNodeTrust(CSA[p])->InConflict == 0){
                        myNodeTrustTable.getNodeTrust(CSA[p])->NodeType = 1;
                    }
                }
            }
        }  
    }


}


// ********************************************************************************************************************












/**
 * @brief Function to check if the node is in the local perception list
 */
bool CPMConflictCheck::InLocalPerception(uint32_t newNodeID, const LocalEnvironmentModel& lem){
    const auto& localObj = lem.allObjects();
    const TrackedObjectsFilterRange& TrackedBbjs = filterBySensorCategory(localObj, "Radar");
    for(const auto& i: TrackedBbjs){
        std::weak_ptr<EnvironmentModelObject> obj_ptr = i.first;  
        const VehicleDataProvider& vd = obj_ptr.lock()->getVehicleData();   
        if (vd.getStationId()%100 == newNodeID){
            return true;
        }
    }
    return false;
}


double CPMConflictCheck::AssessmentFunction(double n, double a, double b, double y){
    double val = a / (1 + exp(b*n + y));
    return val;
}

double CPMConflictCheck::PenaltyFunction1(double n, double a, double b, double min , double max){

    double val = a*n + b;

    if(val <= min){
        return min;
    }else if(val >= max){
        return max;
    }else{
        return val;
    }
}

bool CPMConflictCheck::IsPointInPolygon(artery::geometry::Point pt, artery::geometry::Polygon polygon){
    // Count polygon size
    return bg::within(pt,polygon);
}

