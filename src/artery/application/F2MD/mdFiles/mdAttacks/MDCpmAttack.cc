/*******************************************************************************
 * @author  Jiahao ZHANG
 * @email   jiahao.zhang96@gmail.com
 * @date    09/11/2022
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <vanetza/btp/ports.hpp>
#include "MDCpmAttack.h"


// const std::__cxx11::string objID = "Obj";
// const std::__cxx11::string ghostID = "ghost";
// int flag_creat_ghost = 0;

MDCpmAttack::MDCpmAttack() {
}


/**
   *  @brief initialize the attack that will further be added to the CPM (Calculate constant offset, positions and vairious constant attacks parameters)
   *  @param F2MD_CAMParameters* CAM Parameters
*/
void MDCpmAttack::init(F2MD_CPParameters *params, const traci::VehicleController* VehController) {
    
    this->mVehicleController = VehController;
    
    this->params = params;
    this->MaxRadarRange = params->MaxRadarRange;
    this->MaxMapBoundary = params->MaxMapBoundary;

    ConstDistX = genLib.RandomDouble(0, MaxRadarRange) * 100;
    ConstDistY = genLib.RandomDouble(0, MaxRadarRange) * 100;

    // ConstSpeedX = genLib.RandomDouble(0, params->RandomSpeed) * 100;
    // ConstSpeedY = genLib.RandomDouble(0, params->RandomSpeed) * 100;
    ConstSpeed = genLib.RandomDouble(0, params->RandomSpeed) * 100;


    ConstDistOffsetX = genLib.RandomDouble((1-params->cpParVar) * params->RandomDistOffsetX * 100,
            (1+params->cpParVar) * params->RandomDistOffsetX) * 100;
    ConstDistOffsetY = genLib.RandomDouble((1-params->cpParVar) * params->RandomDistOffsetY * 100,
            (1+params->cpParVar) * params->RandomDistOffsetY) * 100;

    // ConstSpeedOffsetX = genLib.RandomDouble((1-params->cpParVar) * params->RandomSpeedOffset,
    //         (1+params->cpParVar) * params->RandomSpeedOffset) * 100;
    // ConstSpeedOffsetY = genLib.RandomDouble((1-params->cpParVar) * params->RandomSpeedOffset,
    //         (1+params->cpParVar) * params->RandomSpeedOffset) * 100;
    ConstSpeedOffset = genLib.RandomDouble((1-params->cpParVar) * params->RandomSpeedOffset,
            (1+params->cpParVar) * params->RandomSpeedOffset) * 100;

    maxTimeOfMeasurement = 1500;     //ASN regulation


}

/**
 * @brief change the transmission frequency, preserve for the DoS attack
 * @param setBeaconInterval 
 */
void MDCpmAttack::setBeaconInterval(simtime_t* beaconInterval) {
    this->beaconInterval = beaconInterval;
}

/**
   *  @brief  launch the attack specified in the parameters. The attack is directly added to the CPM given as an input
   *  @param  AttackTypes AttackType.
   *  @param  CpmPayload_t* CPM on which the attack must be done.
*/
void MDCpmAttack::launchAttack(cpAttackTypes::Attacks myAttackType, CpmPayload_t* cpm) {

    // check each CPM data container
    for(int cpmContainerIndx = 0;  cpmContainerIndx < cpm->cpmContainers.list.count; cpmContainerIndx++){
        ConstraintWrappedCpmContainers_t* pd_DF = &cpm->cpmContainers;
            // Inject attacks in the Sensor Information Container
            // Conserve this part for the future work
            if(pd_DF->list.array[cpmContainerIndx]->containerId == CpmContainerId_sensorInformationContainer){

            }

            // Inject the attacks in the Perception Region Container
            // Conserve this part for the future work
            if(pd_DF->list.array[cpmContainerIndx]->containerId == CpmContainerId_perceptionRegionContainer){

            }

            // Inject attacks in the Originating Vehicle Container
            // Conserve this part for the future work
            if(pd_DF->list.array[cpmContainerIndx]->containerId == CpmContainerId_originatingVehicleContainer){

            }

            // Inject attacks in the Originating Rsu Container
            // Conserve this part for the future work
            if(pd_DF->list.array[cpmContainerIndx]->containerId == CpmContainerId_originatingRsuContainer){

            }

            // Inject attacks in the Perceived Object Container
            if(pd_DF->list.array[cpmContainerIndx]->containerId == CpmContainerId_perceivedObjectContainer){
                PerceivedObjectContainer& Addr_poc = pd_DF->list.array[cpmContainerIndx]->containerData.choice.PerceivedObjectContainer;
                auto* poc = &Addr_poc;
                //EV_INFO << "MyAttackType : " << myAttackType << endl;
            
                if(Addr_poc.numberOfPerceivedObjects > 0){
                    switch (myAttackType) {
                        
                        // Constant XY distance modification for the perceived objects
                        // ENU reference system employed (Ego_coordinates(0,0))
                        case cpAttackTypes::MultiConstDist: {
                            //EV_INFO << "Launched Attack : ConstDist" << endl;
                            for (int i =0; i < poc->perceivedObjects.list.count; i++) {
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                po->position.xCoordinate.value = ConstDistX;
                                po->position.yCoordinate.value = ConstDistY;
                            }
                        } break;

                        // Constant Velocity modification for all the perceived objects
                        // We only consider the attack in polarVelocity format according to the ASN1
                        case cpAttackTypes::MultiConstSpeed: {
                            //EV_INFO << "Launched Attack : ConstSpeed" << endl;
                            for (int i =0; i < poc->perceivedObjects.list.count; i++) {
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = ConstSpeed;
                            }
                        } break;
                        
                        // Random XY distance modification for all perceived objects
                        // ENU reference system employed (Ego_coordinates(0,0))
                        case cpAttackTypes::MultiRandomDist: {
                            //EV_INFO << "Launched Attack : randomDist" << endl;
                            for (int i =0; i < poc->perceivedObjects.list.count; i++) {
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                po->position.xCoordinate.value = genLib.RandomDouble(-MaxRadarRange * 100, MaxRadarRange * 100);
                                po->position.yCoordinate.value = genLib.RandomDouble(-MaxRadarRange * 100, MaxRadarRange * 100);
                            }
                        } break;

                        // Random Velocity modification for all perceived objects
                        // We only consider the attack in polarVelocity format according to the ASN1
                        case cpAttackTypes::MultiRandomSpeed: {
                            //EV_INFO << "Launched Attack : RandomSpeed" << endl;
                            for (int i =0; i < poc->perceivedObjects.list.count; i++) {
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = genLib.RandomDouble(0, params->RandomSpeed * 100);
                            }
                        } break;
                        

                        // Random XY distance modification for a perceived object chosen randomly
                        // Attack on a random perceived object (different ID) for each CPM
                        // The random distance interval is [ -MaxMapBoundary, +MaxMapBoundary], 
                        // Unit: 0.01 m in ASN1 (*100 because we need to change the unit to 1 m)
                        case cpAttackTypes::SingleRandomDist: {
                            // std::cout << "*****Launched Attack : SingleRandomDist" << std::endl;
                            if (params->KeepSameID){
                                bool flagSD = false;
                                for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                        PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                        int RandomDistMax = MaxMapBoundary * 100;
                                        auto xCoordinateValue = genLib.RandomInt(-RandomDistMax, RandomDistMax);
                                        auto yCoordinateValue = genLib.RandomInt(-RandomDistMax, RandomDistMax);
                                        if(xCoordinateValue > -131072 && xCoordinateValue < 131072){
                                            po->position.xCoordinate.value = xCoordinateValue;
                                        }else if(xCoordinateValue <= -131072){
                                            po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                        }else if(xCoordinateValue >= 131071){
                                            po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange; 
                                        }

                                        if(yCoordinateValue > -131072 && yCoordinateValue < 131072){
                                            po->position.yCoordinate.value = yCoordinateValue;
                                        }else if(yCoordinateValue <= -131072){
                                            po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                        }else if(yCoordinateValue >= 131071){
                                            po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                        }

                                        flagSD = true;
                                        break; 
                                    }
                                }
                                if(!flagSD){
                                    InitSelectedStationID(poc);
                                    for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                        if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                            PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                            int RandomDistMax = MaxMapBoundary * 100;
                                            auto xCoordinateValue = genLib.RandomInt(-RandomDistMax, RandomDistMax);
                                            auto yCoordinateValue = genLib.RandomInt(-RandomDistMax, RandomDistMax);
                                            if(xCoordinateValue > -131072 && xCoordinateValue < 131072){
                                                po->position.xCoordinate.value = xCoordinateValue;
                                            }else if(xCoordinateValue <= -131072){
                                                po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                            }else if(xCoordinateValue >= 131071){
                                                po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange; 
                                            }

                                            if(yCoordinateValue > -131072 && yCoordinateValue < 131072){
                                                po->position.yCoordinate.value = yCoordinateValue;
                                            }else if(yCoordinateValue <= -131072){
                                                po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                            }else if(yCoordinateValue >= 131071){
                                                po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                            }
                                            break; 
                                        }
                                    }
                                }
                            }else{
                                int index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[index];
                                int RandomDistMax = MaxMapBoundary * 100;
                                auto xCoordinateValue = genLib.RandomInt(-RandomDistMax, RandomDistMax);
                                auto yCoordinateValue = genLib.RandomInt(-RandomDistMax, RandomDistMax);
                                if(xCoordinateValue > -131072 && xCoordinateValue < 131072){
                                    po->position.xCoordinate.value = xCoordinateValue;
                                }else if(xCoordinateValue <= -131072){
                                    po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                }else if(xCoordinateValue >= 131071){
                                    po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                }

                                if(yCoordinateValue > -131072 && yCoordinateValue < 131072){
                                    po->position.yCoordinate.value = yCoordinateValue;
                                }else if(yCoordinateValue <= -131072){
                                    po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                }else if(yCoordinateValue >= 131071){
                                    po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                } 
                            }
                             

                        } break;
                        
                        // Random Velocity modification for a perceived object chosen randomly
                        // Attack on a random perceived object (different ID) for each CPM
                        // The random speed interval is [ -100km/h, 100km/h ] ~= [ -28m/s, +28m/s] by default
                        // Unit: 0.01 m/s in ASN1 (*100 because we need to change the unit to 1 m/s)
                        case cpAttackTypes::SingleRandomSpeed: {
                            //EV_INFO << "Launched Attack : SingleRandomSpeed" << endl;
                            // std::cout << "******Launched Attack : SingleRandomSpeed" << endl;
                            if (params->KeepSameID){
                                bool flagSD = false;
                                for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                        PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                        int RandomSpeedMax = params->RandomSpeed * 100;
                                        int genRandomSpeed = genLib.RandomInt(0, RandomSpeedMax);  // random speed magnitude [0, 28m/s]
                                        if (genRandomSpeed > 16381){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                        }else if(genRandomSpeed > 0 && genRandomSpeed < 16382){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = genRandomSpeed;
                                        }else if(genRandomSpeed == 0){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                        }else{
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                        }
                                        flagSD = true;
                                        break;
                                    }
                                }
                                if(!flagSD){
                                    InitSelectedStationID(poc);
                                    for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                        PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                        int RandomSpeedMax = params->RandomSpeed * 100;
                                        int genRandomSpeed = genLib.RandomInt(0, RandomSpeedMax);  // random speed magnitude [0, 28m/s]
                                        if (genRandomSpeed > 16381){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                        }else if(genRandomSpeed > 0 && genRandomSpeed < 16382){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = genRandomSpeed;
                                        }else if(genRandomSpeed == 0){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                        }else{
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                        }
                                    }
                                    break;
                                  }
                                }
                            }else{
                                int index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[index];
                                int RandomSpeedMax = params->RandomSpeed * 100;
                                int genRandomSpeed = genLib.RandomInt(0, RandomSpeedMax);  // random speed magnitude [0, 28m/s]
                                if (genRandomSpeed > 16381){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                }else if(genRandomSpeed > 0 && genRandomSpeed < 16382){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = genRandomSpeed;
                                }else if(genRandomSpeed == 0){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                }else{
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                }
                            }
                            
                            

                            // auto& vehicle_api = mVehicleController->getTraCI()->vehicle;

                            // if (flag_creat_ghost > 0){
                            //     for(const auto& id:vehicle_api.getIDList()){
                            //         if (id == ghostID){
                            //             vehicle_api.setColor(ghostID,libsumo::TraCIColor(255,204,0,255));
                            //             vehicle_api.setSpeed(ghostID,double(genRandomSpeed/100));
                            //         }
                            //     }
                            // }else{
                                
                            //     auto RouteID = vehicle_api.getRouteID(objID);
                            //     auto vType = vehicle_api.getTypeID(objID);
                            //     string depart = "-1";
                            //     auto departlane = to_string(vehicle_api.getLaneIndex(objID));
                            //     auto LanePosition = to_string(vehicle_api.getLanePosition(objID));
                            //     vehicle_api.add(ghostID,RouteID,vType,depart,departlane,LanePosition);
                            // }

                            // flag_creat_ghost++;
                        } break;

                        // Const XY distance modification for a random perceived object
                        case cpAttackTypes::SingleConstDist: {
                            // std::cout << "Launched Attack : SingleConstDist" << std::endl;
                            if (params->KeepSameID){
                                bool flagSD = false;
                                for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                        PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                        po->position.xCoordinate.value = ConstDistX;
                                        po->position.yCoordinate.value = ConstDistY;
                                        flagSD = true;
                                        break;
                                    }
                                }
                                if(!flagSD){
                                    InitSelectedStationID(poc);
                                    for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                        if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                            PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                            po->position.xCoordinate.value = ConstDistX;
                                            po->position.yCoordinate.value = ConstDistY;
                                            break;
                                        }
                                    }
                                }
                            }else{
                                int index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[index];
                                po->position.xCoordinate.value = ConstDistX;
                                po->position.yCoordinate.value = ConstDistY;
                            }

                        } break;
                        
                        // Const Velocity modification for a random perceived object
                        // We only consider the attack in polarVelocity format according to the ASN1
                        case cpAttackTypes::SingleConstSpeed: {
                            // EV_INFO << "Launched Attack : SingleRandomSpeed" << endl;
                            if(params->KeepSameID){
                                bool flagSD = false;
                                for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                        PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                        if (ConstSpeed > 16381){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                        }else if(ConstSpeed > 0 && ConstSpeed < 16382){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = ConstSpeed;
                                        }else if(ConstSpeed == 0){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                        }else{
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                        }
                                        flagSD = true;
                                        break;
                                    }
                                }
                                if(!flagSD){
                                    InitSelectedStationID(poc);
                                    for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                        if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                            PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                            if (ConstSpeed > 16381){
                                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                            }else if(ConstSpeed > 0 && ConstSpeed < 16382){
                                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = ConstSpeed;
                                            }else if(ConstSpeed == 0){
                                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                            }else{
                                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                            }
                                            break;
                                        }
                                    }
                                }
                            }else{
                                int index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[index];
                                if (ConstSpeed > 16381){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                }else if(ConstSpeed > 0 && ConstSpeed < 16382){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = ConstSpeed;
                                }else if(ConstSpeed == 0){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                }else{
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                }
                            }
                            
                        } break;


                        // Each attacker injects a constant distance offset for a random perceived object in each CPM
                        case cpAttackTypes::SingleConstDistOffset: {
                            //EV_INFO << "Launched Attack : ConstDistOffset" << endl;
                            if(params->KeepSameID){
                                bool flagSD = false;
                                for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                        PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                        auto SumConstDistOffsetX = po->position.xCoordinate.value + ConstDistOffsetX;
                                        auto SumConstDistOffsetY = po->position.yCoordinate.value + ConstDistOffsetY;
                                        if(SumConstDistOffsetX > -131072 && SumConstDistOffsetX < 131072){
                                            po->position.xCoordinate.value = SumConstDistOffsetX;
                                        }else if(SumConstDistOffsetX <= -131072){
                                            po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                        }else if(SumConstDistOffsetX >= 131071){
                                            po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                        }

                                        if(SumConstDistOffsetY > -131072 && SumConstDistOffsetY < 131072){  
                                            po->position.yCoordinate.value = SumConstDistOffsetY;
                                        }else if(SumConstDistOffsetY <= -131072){
                                            po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                        }else if(SumConstDistOffsetY >= 131071){
                                            po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                        }
                                    }
                                    flagSD = true;
                                    break;
                                }
                                if(!flagSD){
                                    InitSelectedStationID(poc);
                                    for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                        if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                            PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                            auto SumConstDistOffsetX = po->position.xCoordinate.value + ConstDistOffsetX;
                                            auto SumConstDistOffsetY = po->position.yCoordinate.value + ConstDistOffsetY;
                                            if(SumConstDistOffsetX > -131072 && SumConstDistOffsetX < 131072){
                                                po->position.xCoordinate.value = SumConstDistOffsetX;
                                            }else if(SumConstDistOffsetX <= -131072){
                                                po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                            }else if(SumConstDistOffsetX >= 131071){
                                                po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                            }

                                            if(SumConstDistOffsetY > -131072 && SumConstDistOffsetY < 131072){  
                                                po->position.yCoordinate.value = SumConstDistOffsetY;
                                            }else if(SumConstDistOffsetY <= -131072){
                                                po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                            }else if(SumConstDistOffsetY >= 131071){
                                                po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                            }
                                            break;
                                        }                                        
                                    }
                                }
                            }else{                          
                                int index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[index];
                                auto SumConstDistOffsetX = po->position.xCoordinate.value + ConstDistOffsetX;
                                auto SumConstDistOffsetY = po->position.yCoordinate.value + ConstDistOffsetY;
                                if(SumConstDistOffsetX > -131072 && SumConstDistOffsetX < 131072){
                                    po->position.xCoordinate.value = SumConstDistOffsetX;
                                }else if(SumConstDistOffsetX <= -131072){
                                    po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                }else if(SumConstDistOffsetX >= 131071){
                                    po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                }

                                if(SumConstDistOffsetY > -131072 && SumConstDistOffsetY < 131072){  
                                    po->position.yCoordinate.value = SumConstDistOffsetY;
                                }else if(SumConstDistOffsetY <= -131072){
                                    po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                }else if(SumConstDistOffsetY >= 131071){
                                    po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                }
                            }
                        } break;

                        // Each attacker injects a constant speed offset for a random perceived object in each CPM
                        // We only consider the attack in polarVelocity format according to the ASN1
                        case cpAttackTypes::SingleConstSpeedOffset: {
                            //EV_INFO << "Launched Attack : ConstSpeedOffset" << endl;
                            int index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                            PerceivedObject_t *po = poc->perceivedObjects.list.array[index];
                            auto SumConstSpeedOffset = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue + ConstSpeedOffset; // ConstSpeedOffsetX need to correct 
                            if (SumConstSpeedOffset > 16381){
                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                            }else if(SumConstSpeedOffset > 0 && SumConstSpeedOffset < 16382){
                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SumConstSpeedOffset;
                            }else if(SumConstSpeedOffset == 0){
                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                            }else{
                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                            }
                        } break;


                        // Random distance offset modification for a random perceived object
                        // add noise to the distance, the noise adapts the Gaussian iid with mean = 0 and sigma = maxRadarRange / 10 
                        case cpAttackTypes::SingleRandomDistOffset: {
                            //EV_INFO << "Launched Attack : RandomDistOffset" << endl;
                            if(params->KeepSameID){
                                bool flagSD = false;
                                for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                        PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                        double  RandDistOffsetX = genLib.GaussianRandomDouble(0, MaxRadarRange/10) * 100;
                                        double  RandDistOffsetY = genLib.GaussianRandomDouble(0, MaxRadarRange/10) * 100;

                                        auto SumRandDistOffsetX = po->position.xCoordinate.value + int(RandDistOffsetX);
                                        auto SumRandDistOffsetY = po->position.yCoordinate.value + int(RandDistOffsetY);
                                        if(SumRandDistOffsetX > -131072 && SumRandDistOffsetX < 131072){
                                            po->position.xCoordinate.value = SumRandDistOffsetX;
                                        }else if(SumRandDistOffsetX <= -131072){
                                            po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                        }else if(SumRandDistOffsetX >= 131071){
                                            po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange; 
                                        }

                                        if(SumRandDistOffsetY > -131072 && SumRandDistOffsetY < 131072){  
                                            po->position.yCoordinate.value = SumRandDistOffsetY;
                                        }else if(SumRandDistOffsetY <= -131072){
                                            po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                        }else if(SumRandDistOffsetY >= 131071){
                                            po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                        }

                                        flagSD = true;
                                        break;
                                    }
                                }
                                if(!flagSD){
                                    InitSelectedStationID(poc);
                                    for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                        if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                            PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                            double  RandDistOffsetX = genLib.GaussianRandomDouble(0, MaxRadarRange/10) * 100;
                                            double  RandDistOffsetY = genLib.GaussianRandomDouble(0, MaxRadarRange/10) * 100;

                                            auto SumRandDistOffsetX = po->position.xCoordinate.value + int(RandDistOffsetX);
                                            auto SumRandDistOffsetY = po->position.yCoordinate.value + int(RandDistOffsetY);
                                            if(SumRandDistOffsetX > -131072 && SumRandDistOffsetX < 131072){
                                                po->position.xCoordinate.value = SumRandDistOffsetX;
                                            }else if(SumRandDistOffsetX <= -131072){
                                                po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                            }else if(SumRandDistOffsetX >= 131071){
                                                po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange; 
                                            }

                                            if(SumRandDistOffsetY > -131072 && SumRandDistOffsetY < 131072){  
                                                po->position.yCoordinate.value = SumRandDistOffsetY;
                                            }else if(SumRandDistOffsetY <= -131072){
                                                po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                            }else if(SumRandDistOffsetY >= 131071){
                                                po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                            }
                                            break;
                                        }
                                    }    
                                }
                            }else{
                                int index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[index];

                                double  RandDistOffsetX = genLib.GaussianRandomDouble(0, MaxRadarRange/10) * 100;
                                double  RandDistOffsetY = genLib.GaussianRandomDouble(0, MaxRadarRange/10) * 100;

                                auto SumRandDistOffsetX = po->position.xCoordinate.value + int(RandDistOffsetX);
                                auto SumRandDistOffsetY = po->position.yCoordinate.value + int(RandDistOffsetY);
                                if(SumRandDistOffsetX > -131072 && SumRandDistOffsetX < 131072){
                                    po->position.xCoordinate.value = SumRandDistOffsetX;
                                }else if(SumRandDistOffsetX <= -131072){
                                    po->position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                }else if(SumRandDistOffsetX >= 131071){
                                    po->position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                }

                                if(SumRandDistOffsetY > -131072 && SumRandDistOffsetY < 131072){  
                                    po->position.yCoordinate.value = SumRandDistOffsetY;
                                }else if(SumRandDistOffsetY <= -131072){
                                    po->position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
                                }else if(SumRandDistOffsetY >= 131071){
                                    po->position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
                                }
                            }
                        } break;
                        

                        //  Random speed offset modification for a random perceived object
                        //  add noise to the velocity, the noise adapts the Gaussian iid with mean = 0 and sigma = real_velocity / 10
                        // We only consider the attack in polarVelocity format according to the ASN1
                        case cpAttackTypes::SingleRandomSpeedOffset: {
                            if(params->KeepSameID){
                                bool flagSD = false;
                                for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                        PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                        double curSpeed = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue / 100;
                                        double RandSpeedOffset = fabs(genLib.GaussianRandomDouble(0, curSpeed) * 100);
                                        auto SumRandSpeedOffset = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue + int(RandSpeedOffset);
                                        if (SumRandSpeedOffset > 16381){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                        }else if(SumRandSpeedOffset > 0 && SumRandSpeedOffset < 16382){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SumRandSpeedOffset;
                                        }else if(SumRandSpeedOffset == 0){
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                        }else{
                                            po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                        }
                                        flagSD = true;
                                        break;
                                    }
                                }
                                if(!flagSD){
                                    InitSelectedStationID(poc);
                                    for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                        if(*poc->perceivedObjects.list.array[i]->objectId == SelectedStationID){
                                            PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                            double curSpeed = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue / 100;
                                            double RandSpeedOffset = fabs(genLib.GaussianRandomDouble(0, curSpeed) * 100);
                                            auto SumRandSpeedOffset = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue + int(RandSpeedOffset);
                                            if (SumRandSpeedOffset > 16381){
                                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                            }else if(SumRandSpeedOffset > 0 && SumRandSpeedOffset < 16382){
                                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SumRandSpeedOffset;
                                            }else if(SumRandSpeedOffset == 0){
                                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                            }else{
                                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                            }
                                            break;
                                        }
                                    } 
                                }

                            }else{
                                int index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[index];
                                double curSpeed = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue / 100;

                                double RandSpeedOffset = fabs(genLib.GaussianRandomDouble(0, curSpeed) * 100);
                                auto SumRandSpeedOffset = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue + int(RandSpeedOffset);
                                if (SumRandSpeedOffset > 16381){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_outOfRange;
                                }else if(SumRandSpeedOffset > 0 && SumRandSpeedOffset < 16382){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SumRandSpeedOffset;
                                }else if(SumRandSpeedOffset == 0){
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_standstill;
                                }else{
                                    po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = SpeedValue_unavailable;
                                }
                            }
                        } break;

                        // Constant distance offset modification for all perceived objects
                        case cpAttackTypes::MultiConstDistOffset: {
                            //EV_INFO << "Launched Attack : MultiConstDistOffset" << endl;
                            for (int i =0; i < poc->perceivedObjects.list.count; i++) {
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                po->position.xCoordinate.value = po->position.xCoordinate.value + ConstDistOffsetX;
                                po->position.yCoordinate.value = po->position.yCoordinate.value + ConstDistOffsetY;
                            }
                        } break;


                        // Constant speed offset modification for all perceived objects
                        // We only consider the attack in polarVelocity format according to the ASN1
                        case cpAttackTypes::MultiConstSpeedOffset: {
                            //EV_INFO << "Launched Attack : MultiConstSpeedOffset" << endl;
                            for (int i =0; i < poc->perceivedObjects.list.count; i++) {
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[i];
                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue + ConstSpeedOffset; // ConstSpeedOffsetX need to correct
                            }
                        } break;


                        // Random distance offset modification for multiple perceived objects
                        case cpAttackTypes::MultiRandomDistOffset: {
                            //EV_INFO << "Launched Attack : MultiRandomDistOffset" << endl;
                            for (int i =0; i < poc->perceivedObjects.list.count; i++) {
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[i];

                                double RandDistOffsetX = genLib.RandomDouble((1-params->cpParVar) * params->RandomDistOffsetX * 100,
                                (1+params->cpParVar) * params->RandomDistOffsetX * 100);
                                double RandDistOffsetY = genLib.RandomDouble((1-params->cpParVar) * params->RandomDistOffsetY * 100,
                                (1+params->cpParVar) * params->RandomDistOffsetY * 100);

                                po->position.xCoordinate.value = po->position.xCoordinate.value + RandDistOffsetX;
                                po->position.yCoordinate.value = po->position.yCoordinate.value + RandDistOffsetY;
                            }
                        } break;



                        // Random speed offset modification for all perceived objects
                        case cpAttackTypes::MultiRandomSpeedOffset: {
                            //EV_INFO << "Launched Attack : MultiRandomSpeedOffset" << endl;
                            for (int i =0; i < poc->perceivedObjects.list.count; i++) {
                                PerceivedObject_t *po = poc->perceivedObjects.list.array[i];

                                double randSpeedOffset = genLib.RandomDouble((1-params->cpParVar) * params->RandomSpeedOffset * 100,
                                (1+params->cpParVar) * params->RandomSpeedOffset * 100);
                                
                                po->velocity->choice.polarVelocity.velocityMagnitude.speedValue = po->velocity->choice.polarVelocity.velocityMagnitude.speedValue + randSpeedOffset;
                            }
                        } break;


                        // Drop single object: Drop an object randomly chosen from the Perceived Dynamic Object list
                        // KeepSameID:: dropping the same object as long as it stays in the list  
                        case cpAttackTypes::DropObj: {
                            //EV_INFO << "Launched Attack : DropObj" << endl;
                            if(params->KeepSameID){
                                // std::cout << "####Launched Attack : DropObj_KeepSameID" << endl;
                                bool flagSD = false;
                                for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                    const int ObjStationID = *poc->perceivedObjects.list.array[i]->objectId;
                                    if(ObjStationID == SelectedStationID){
                                        if(poc->perceivedObjects.list.count > 1){
                                            asn_sequence_del(&poc->perceivedObjects, i, 0);
                                            poc->numberOfPerceivedObjects--;
                                        }else{
                                            asn_sequence_empty(&poc->perceivedObjects);
                                            poc->numberOfPerceivedObjects--;
                                        }
                                        flagSD = true;
                                        break;
                                    }
                                }
                                if(!flagSD){
                                    InitSelectedStationID(poc);
                                    for(int i=0;i < poc->perceivedObjects.list.count; i++){
                                        const int ObjStationID = *poc->perceivedObjects.list.array[i]->objectId;
                                        if(ObjStationID == SelectedStationID){
                                            if(poc->perceivedObjects.list.count > 1){
                                                asn_sequence_del(&poc->perceivedObjects, i, 0);
                                                poc->numberOfPerceivedObjects--;
                                            }else{
                                                asn_sequence_empty(&poc->perceivedObjects);
                                                poc->numberOfPerceivedObjects--;
                                            }
                                            break;
                                        }
                                    }
                                }
                            }else{
                                // std::cout << "####Launched Attack : DropObj" << endl;
                                if(poc->numberOfPerceivedObjects > 1){
                                    int DropObj_index = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
                                    poc->numberOfPerceivedObjects--;
                                    asn_sequence_del(&poc->perceivedObjects,DropObj_index,0);
                                }else if(poc->numberOfPerceivedObjects == 1){       
                                    // if the list existe only one perceiived object, the attacker should delete the whole perception data container
                                    // to adapt the CPM standard
                                    asn_sequence_empty(&poc->perceivedObjects);
                                    poc->numberOfPerceivedObjects--;
                                }
                            }
                        } break;

                        // Drop all perceived objects
                        case cpAttackTypes::DropAllObj: {
                            //EV_INFO << "####Launched Attack : DropAllObj" << endl;
                            // std::cout << "####Launched Attack : DropAllObj" << endl;

                            asn_sequence_empty(&poc->perceivedObjects);
                            poc->numberOfPerceivedObjects = 0;
                        } break;


                        // Add a dynamic ghost with a fixed ID and offset XY
                        // Inject a reasonable ghost vehicle
                        case cpAttackTypes::AddObj: {
                            // std::cout << "####Launched Attack : addObj" << endl;
                            auto object = vanetza::asn1::allocate<PerceivedObject_t>();
                            object->objectId = vanetza::asn1::allocate<Identifier2B_t>();
                            int GhostID = 99;
                            *(object->objectId) = GhostID;
                            object->measurementDeltaTime = genLib.RandomInt(0,maxTimeOfMeasurement);

                            object->position.xCoordinate.value = 3000 + genLib.RandomInt(-200, 200);
                            object->position.xCoordinate.confidence = CoordinateConfidence_unavailable;
                            
                            object->position.yCoordinate.value = 0;
                            object->position.yCoordinate.confidence = CoordinateConfidence_unavailable;

                            object->velocity = vanetza::asn1::allocate<Velocity3dWithConfidence_t>();
                            object->velocity->present = Velocity3dWithConfidence_PR_polarVelocity;
                            object->velocity->choice.polarVelocity.velocityMagnitude.speedValue = CartesianAngleValue_unavailable; 
                            object->velocity->choice.polarVelocity.velocityMagnitude.speedConfidence = SpeedConfidence_unavailable;

                            object->velocity->choice.polarVelocity.velocityDirection.value  = CartesianAngleValue_unavailable;
                            object->velocity->choice.polarVelocity.velocityDirection.confidence = AngleConfidence_unavailable;

                            object->acceleration = vanetza::asn1::allocate<Acceleration3dWithConfidence_t>();
                            object->acceleration->present = Acceleration3dWithConfidence_PR_polarAcceleration;
                            object->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue = AccelerationMagnitudeValue_unavailable;
                            object->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationConfidence = AccelerationConfidence_unavailable;
                            object->acceleration->choice.polarAcceleration.accelerationDirection.value = CartesianAngleValue_unavailable;
                            object->acceleration->choice.polarAcceleration.accelerationDirection.confidence = AngleConfidence_unavailable;

                            object->zAngularVelocity = vanetza::asn1::allocate<CartesianAngularVelocityComponent_t>();
                            object->zAngularVelocity->value = CartesianAngularVelocityComponentValue_unavailable;
                            object->zAngularVelocity->confidence = AngularSpeedConfidence_unavailable;

                            object->angles = vanetza::asn1::allocate<EulerAnglesWithConfidence_t>();
                            object->angles->zAngle.value = CartesianAngleValue_unavailable; // degree from north, clockwise, right hand rule
                            object->angles->zAngle.confidence = AngleConfidence_unavailable;
                            ASN_SEQUENCE_ADD(&poc->perceivedObjects,object);
                            poc->numberOfPerceivedObjects++;
                        } break;
                    }
                }else{
                    switch (myAttackType) {
                        case cpAttackTypes::AddObj: {
                            auto object = vanetza::asn1::allocate<PerceivedObject_t>();
                            object->objectId = vanetza::asn1::allocate<Identifier2B_t>();
                            int GhostID = 99;
                            *(object->objectId) = GhostID;
                            object->measurementDeltaTime = genLib.RandomInt(0,maxTimeOfMeasurement);

                            object->position.xCoordinate.value = 3000 + genLib.RandomInt(-200, 200);
                            object->position.xCoordinate.confidence = CoordinateConfidence_unavailable;
                            
                            object->position.yCoordinate.value = 0;
                            object->position.yCoordinate.confidence = CoordinateConfidence_unavailable;

                            object->velocity = vanetza::asn1::allocate<Velocity3dWithConfidence_t>();
                            object->velocity->present = Velocity3dWithConfidence_PR_polarVelocity;
                            object->velocity->choice.polarVelocity.velocityMagnitude.speedValue = CartesianAngleValue_unavailable; 
                            object->velocity->choice.polarVelocity.velocityMagnitude.speedConfidence = SpeedConfidence_unavailable;

                            object->velocity->choice.polarVelocity.velocityDirection.value  = CartesianAngleValue_unavailable;
                            object->velocity->choice.polarVelocity.velocityDirection.confidence = AngleConfidence_unavailable;

                            object->acceleration = vanetza::asn1::allocate<Acceleration3dWithConfidence_t>();
                            object->acceleration->present = Acceleration3dWithConfidence_PR_polarAcceleration;
                            object->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue = AccelerationMagnitudeValue_unavailable;
                            object->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationConfidence = AccelerationConfidence_unavailable;
                            object->acceleration->choice.polarAcceleration.accelerationDirection.value = CartesianAngleValue_unavailable;
                            object->acceleration->choice.polarAcceleration.accelerationDirection.confidence = AngleConfidence_unavailable;

                            object->zAngularVelocity = vanetza::asn1::allocate<CartesianAngularVelocityComponent_t>();
                            object->zAngularVelocity->value = CartesianAngularVelocityComponentValue_unavailable;
                            object->zAngularVelocity->confidence = AngularSpeedConfidence_unavailable;

                            object->angles = vanetza::asn1::allocate<EulerAnglesWithConfidence_t>();
                            object->angles->zAngle.value = CartesianAngleValue_unavailable; // degree from north, clockwise, right hand rule
                            object->angles->zAngle.confidence = AngleConfidence_unavailable;

                            ASN_SEQUENCE_ADD(&poc->perceivedObjects,object);
                            poc->numberOfPerceivedObjects++;       
                        }break;
                    }

                }        
        }
    }
}


/**
 * @brief initialize the attacked target ID 
 * @param poc PerceivedObjectContainer
 */
void MDCpmAttack::InitSelectedStationID(PerceivedObjectContainer *poc){
    const int RandomIndx = genLib.RandomInt(0, poc->perceivedObjects.list.count-1);
    cout << "RandomIndx:::" << RandomIndx << endl;
    SelectedStationID = *poc->perceivedObjects.list.array[RandomIndx]->objectId;
    cout << "SelectedStationID:::" << SelectedStationID << endl;
}