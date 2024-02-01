
#include "CpmCheckList.h"

CpmCheckList::CpmCheckList(){
    
    this->distancePlausibility = 0;
    this->speedPlausibility = 0;
    this->speedConsistency = 0;
    this->positionSpeedConsistency = 0;
    
    this->kalmanPositionConsistency = 0;
    this->kalmanSpeedConsistency = 0;
    this->kalmanPositionSpeedConsistancyP = 0;
    this->kalmanPositionSpeedConsistancyS = 0;

    this->report = 0;

    this->attackedObject.clear();
    this->attackedObject.shrink_to_fit();
}

void CpmCheckList::setdistancePlausibility(double val){
    this->distancePlausibility = val;
}

double CpmCheckList::getdistancePlausibility(){
    return this->distancePlausibility;
}

void CpmCheckList::setspeedConsistency(double val){
    this->speedConsistency = val;
}

double CpmCheckList::getspeedConsistency(){
    return this->speedConsistency;
}

void CpmCheckList::setspeedPlausibility(double val){
    this->speedPlausibility = val;
}

double CpmCheckList::getspeedPlausibility(){
    return this->speedPlausibility;
}

void CpmCheckList::setpositionSpeedConsistency(double val){
    this->positionSpeedConsistency = val;
}

double CpmCheckList::getpositionSpeedConsistency(){
    return this->positionSpeedConsistency;
}


/* kalman check list  */
void CpmCheckList::setkalmanPositionConsistency(double val){
    this->kalmanPositionConsistency = val;
}

double CpmCheckList::getkalmanPositionConsistency(){
    return this->kalmanPositionConsistency;
}

void CpmCheckList::setkalmanSpeedConsistency(double val){
    this->kalmanSpeedConsistency = val;
}

double CpmCheckList::getkalmanSpeedConsistency(){
    return this->kalmanSpeedConsistency;
}

void CpmCheckList::setkalmanPositionSpeedConsistancyP(double val){
    this->kalmanPositionSpeedConsistancyP = val;
}

double CpmCheckList::getkalmanPositionSpeedConsistancyP(){
    return this->kalmanPositionSpeedConsistancyP;
}

void CpmCheckList::setkalmanPositionSpeedConsistancyS(double val){
    this->kalmanPositionSpeedConsistancyS = val;
}

double CpmCheckList::getkalmanPositionSpeedConsistancyS(){
    return this->kalmanPositionSpeedConsistancyS;
}


/* report */
void CpmCheckList::setreport(bool val){
    this->report = val;
}

bool CpmCheckList::getreport(){
    return this->report;
}


void CpmCheckList::setAttackedObject(AttackedObject objectID){
    this->attackedObject.push_back(objectID);
}

vector<AttackedObject> CpmCheckList::getAttackedObject(){
    return this->attackedObject;
}


void CpmCheckList::resetAll(){

    this->speedConsistency = 0;
    this->speedPlausibility = 0;
    this->distancePlausibility = 0;
    this->positionSpeedConsistency = 0;
    
    this->kalmanPositionConsistency = 0;
    this->kalmanSpeedConsistency = 0;
    this->kalmanPositionSpeedConsistancyP = 0;
    this->kalmanPositionSpeedConsistancyS = 0;

    this->report = 0;

    this->attackedObject.clear();
    this->attackedObject.shrink_to_fit();
}