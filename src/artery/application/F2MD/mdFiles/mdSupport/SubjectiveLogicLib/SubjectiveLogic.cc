/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    14/02/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "SubjectiveLogic.h"

/******************************************************************************
 * @brief Trust revision uing Subjective Logic
 * 
 * @param Ego Ego's trust opinion
 * @param Source Source's advice
 * *****************************************************************************/
SubjectiveLogicFusion::SubjectiveLogicFusion(vector<ObjectOpinion> Ego,vector<ObjectOpinion> Source){
    try{
        if (Ego.size() != Source.size()){
            throw "*Arg should have same size!";
        }

        this->EgoTrust = Ego;
        this->SourceAdvice = Source;

    }catch(const char* e){
        std::cout << e << std::endl;
    }
}

/*********************************************************************************
 * @brief Trust Discounting
 * *******************************************************************************/
vector<ObjectOpinion> SubjectiveLogicFusion::TrustDiscount(vector<ObjectOpinion> EgoTrust, vector<ObjectOpinion> SourceAdvice){
    vector<ObjectOpinion> Opinion2X;
    for (int i=0; i < EgoTrust.size(); i++){
        xt::xarray<double> tempAdvice = EgoTrust[i].GetProjProba()[0] * SourceAdvice[i].GetAdvice();
        double tempU = 1 - EgoTrust[i].GetProjProba()[0] * xt::sum(SourceAdvice[i].GetAdvice())();
        xt::xarray<double> tempBaseRate = SourceAdvice[i].GetBaseRate();
        ObjectOpinion NewDiscountedOpinion(tempAdvice,tempU,tempBaseRate);
        Opinion2X.push_back(NewDiscountedOpinion);
    }
    return Opinion2X;
}


/*********************************************************************************
 * @brief Average Trust Fusion
 * *******************************************************************************/
ObjectOpinion SubjectiveLogicFusion::AvgTrustFusion(vector<ObjectOpinion> EgoTrust, vector<ObjectOpinion> SourceAdvice){

    xt::xarray<double> AvgTrustFusion_advice;
    double AvgTrustFusion_u;
    xt::xarray<double> AvgTrustFusion_a;

    vector<ObjectOpinion> SLtd = this->TrustDiscount(EgoTrust, SourceAdvice);
    int _case = 0;
    for(const auto& i: SLtd){
        if(i.GetU() != 0){
            break;
        }else{
            _case = 1;
        }
    }

    switch (_case)
    {

    // Existing an uncertainty value which doesn't equal 0. 
    case 0:{
        vector<xt::xarray<double>> TempListProdresU;
        vector<xt::xarray<double>> TempListAdvice;

        double TempProdU = 1;
        for(const auto& i: SLtd){

            // deepcopy SLtd
            vector<ObjectOpinion> resSLtd;
            resSLtd.assign(SLtd.begin(),SLtd.end());
            auto find_i = std::find(resSLtd.begin(),resSLtd.end(),i);
            resSLtd.erase(find_i);

            double TempProdresU = 1;
            for(const auto& g : resSLtd){
                TempProdresU *= g.GetU();
            }
            xt::xarray<double> _TempProdresU = {TempProdresU};
            TempListProdresU.push_back(_TempProdresU);
            auto TempAdvice = i.GetAdvice() * TempProdresU;
            TempListAdvice.push_back(TempAdvice);
            TempProdU *= i.GetU();
        }
        vector<std::size_t> shape_TempListAdvice = {1,TempListAdvice.size()};
        vector<std::size_t> shape_TempListProdresU = {1,TempListProdresU.size()};
        auto new_TempListAdvice = xt::xadapt(TempListAdvice,shape_TempListAdvice);
        auto new_TempListProdresU = xt::xadapt(TempListProdresU,shape_TempListProdresU);
        auto _sumTempListAdvice = xt::sum(new_TempListAdvice,{1})();
        auto _sumTempListProdresU = xt::sum(new_TempListProdresU)();
        AvgTrustFusion_advice = _sumTempListAdvice / _sumTempListProdresU;
        AvgTrustFusion_u = SLtd.size() * TempProdU / (_sumTempListProdresU)();
        AvgTrustFusion_a = SLtd[0].GetBaseRate();
        break;
        }
    
    // All uncertainty value equal 0.
    case 1:{
        vector<xt::xarray<double>> TempListAdvice;
        for(const auto& i: SLtd){
            TempListAdvice.push_back(i.GetAdvice());
        }
        vector<std::size_t> shape_TempListAdvice = {1,TempListAdvice.size()};
        auto new_TempListAdvice = xt::xadapt(TempListAdvice,shape_TempListAdvice);
        auto _sumTempListAdvice = xt::sum(new_TempListAdvice,{1})();
        AvgTrustFusion_advice = _sumTempListAdvice / SLtd.size();
        AvgTrustFusion_u = 0;
        AvgTrustFusion_a = SLtd[0].GetBaseRate();

        break;
        }
        
    default:
        break;
    }
    ObjectOpinion AvgTrustFusion(AvgTrustFusion_advice, AvgTrustFusion_u, AvgTrustFusion_a);
    return AvgTrustFusion;
}

/*********************************************************************************
 * @brief Cumulative Trust Fusion
 * *******************************************************************************/
ObjectOpinion SubjectiveLogicFusion::CumTrustFusion(vector<ObjectOpinion> EgoTrust, vector<ObjectOpinion> SourceAdvice){

    xt::xarray<double> CumTrustFusion_advice;
    double CumTrustFusion_u;
    xt::xarray<double> CumTrustFusion_a;

    vector<ObjectOpinion> SLtd = this->TrustDiscount(EgoTrust,SourceAdvice);
    int _case = 0;
    for(const auto& i: SLtd){
        if(i.GetU() != 0){
            break;
        }else{
            _case = 1;
        }
    }

    switch (_case)
    {

    // Existing an uncertainty value which doesn't equal 0. 
    case 0:{
        vector<xt::xarray<double>> TempListProdresU;
        vector<xt::xarray<double>> TempListAdvice;

        double TempProdU = 1;
        for(const auto& i: SLtd){

            // deepcopy SLtd
            vector<ObjectOpinion> resSLtd;
            resSLtd.assign(SLtd.begin(),SLtd.end());
            auto find_i = std::find(resSLtd.begin(),resSLtd.end(),i);
            resSLtd.erase(find_i);

            double TempProdresU = 1;
            for(const auto& g : resSLtd){
                TempProdresU *= g.GetU();
            }
            xt::xarray<double> _TempProdresU = {TempProdresU};
            TempListProdresU.push_back(_TempProdresU);
            auto TempAdvice = i.GetAdvice() * TempProdresU;
            TempListAdvice.push_back(TempAdvice);
            TempProdU *= i.GetU();
        }
        vector<std::size_t> shape_TempListAdvice = {1,TempListAdvice.size()};
        vector<std::size_t> shape_TempListProdresU = {1,TempListProdresU.size()};
        auto new_TempListAdvice = xt::xadapt(TempListAdvice,shape_TempListAdvice);
        auto new_TempListProdresU = xt::xadapt(TempListProdresU,shape_TempListProdresU);
        auto _sumTempListAdvice = xt::sum(new_TempListAdvice,{1})();
        auto _sumTempListProdresU = xt::sum(new_TempListProdresU)();
        CumTrustFusion_advice = _sumTempListAdvice / (_sumTempListProdresU - (SLtd.size()-1) * TempProdU);
        CumTrustFusion_u = TempProdU / (_sumTempListProdresU - (SLtd.size()-1) * TempProdU)();
        CumTrustFusion_a = SLtd[0].GetBaseRate();

        break;
        }

    // All uncertainty value equal 0.    
    case 1:{
        vector<xt::xarray<double>> TempListAdvice;
        for(const auto& i: SLtd){
            TempListAdvice.push_back(i.GetAdvice());
        }        
        vector<std::size_t> shape_TempListAdvice = {1,TempListAdvice.size()};
        auto new_TempListAdvice = xt::xadapt(TempListAdvice,shape_TempListAdvice);
        auto _sumTempListAdvice = xt::sum(new_TempListAdvice,{1})();
        CumTrustFusion_advice = _sumTempListAdvice / SLtd.size();
        CumTrustFusion_u = 0;
        CumTrustFusion_a = SLtd[0].GetBaseRate();

        break;
        }
        
    default:
        break;
    }
    ObjectOpinion CumTrustFusion(CumTrustFusion_advice, CumTrustFusion_u, CumTrustFusion_a);
    return CumTrustFusion;
}


/*********************************************************************************
 * @brief Project Probability Distance
 * *******************************************************************************/
xt::xarray<double> SubjectiveLogicFusion::ProjectProbabilityDistance(){
    vector<ObjectOpinion> SLtd = this->TrustDiscount(this->EgoTrust,this->SourceAdvice);
    ObjectOpinion SLavg = this->AvgTrustFusion(this->EgoTrust,this->SourceAdvice);
    xt::xarray<double> Tempsub;

    // calculate the vector of project probabilty distance
    for(int i=0; i< SLtd.size(); i++){
        if (i==0){
            Tempsub = {SLtd[i].GetProjProba() - SLavg.GetProjProba()};
        }else{
            Tempsub = xt::concatenate(std::tuple(Tempsub,SLtd[i].GetProjProba() - SLavg.GetProjProba()));
        }
    }
    Tempsub.reshape({SLtd.size(),2});
    auto ProbaDistance = xt::sum(xt::abs(Tempsub),{1})/2;
    return ProbaDistance;
}

/*********************************************************************************
 * @brief Conjunctive Confidence
 * *******************************************************************************/
xt::xarray<double> SubjectiveLogicFusion::ConjunctiveConfidence(){
    vector<ObjectOpinion> SLtd = this->TrustDiscount(this->EgoTrust,this->SourceAdvice);
    ObjectOpinion SLavg = this->AvgTrustFusion(this->EgoTrust,this->SourceAdvice);
    xt::xarray<double> ConjConf;

    // calculate the vector of conjunctive confidence
    for(int i=0; i< SLtd.size(); i++){
        xt::xarray<double> TempResult = {(1-SLavg.GetU())*(1-SLtd[i].GetU())};
        if(i==0){
            ConjConf = TempResult;
        }else{
            ConjConf = xt::concatenate(std::tuple(ConjConf,TempResult));
        }
    }
    return ConjConf;
}

/*********************************************************************************
 * @brief Belief Conflict
 * *******************************************************************************/
xt::xarray<double> SubjectiveLogicFusion::BeliefConflict(){
    const xt::xarray<double> SLpd = this->ProjectProbabilityDistance();
    const xt::xarray<double> SLcc = this->ConjunctiveConfidence();

    // BC = PD * CC
    auto Beliefconflict = SLpd * SLcc;
    return Beliefconflict;
}

/*********************************************************************************
 * @brief Maximum of Belief Conflict
 * *******************************************************************************/
double SubjectiveLogicFusion::MaxConflict(){
    xt::xarray<double> SLbc = this->BeliefConflict();
    return xt::amax(SLbc)();
}

/*********************************************************************************
 * @brief Average of Belief Conflict
 * *******************************************************************************/
double SubjectiveLogicFusion::AvgConflict(){
    xt::xarray<double> SLbc = this->BeliefConflict();
    return xt::mean(SLbc)();
}

/*********************************************************************************
 * @brief Revision Weight
 * *******************************************************************************/
xt::xarray<double> SubjectiveLogicFusion::RevisionWeight(){
    xt::xarray<double> SLbc = this->BeliefConflict();
    double SLmc = xt::amax(SLbc)();
    double SLac = xt::mean(SLbc)();
    xt::xarray<double> SLrw;
    xt::xarray<double> TempRW;

    // only the belief conflict of the source opinion greater than average conflict, the trust need to be revised.
    for(int i=0; i<SLbc.size();i++){
        if((SLbc[i]-SLac) > 0){
            TempRW = {SLmc * (SLbc[i]-SLac) / (SLmc-SLac)};
        }else{
            TempRW = {0.};
        }

        if(i==0){
            SLrw = TempRW;
        }else{
            SLrw = xt::concatenate(std::tuple(SLrw,TempRW));
        }
    }

    return SLrw;
}

/*********************************************************************************
 * @brief Trust Revised Opinion
 * *******************************************************************************/
vector<ObjectOpinion> SubjectiveLogicFusion::TrustRevisedOpinion(){
    
    xt::xarray<double> SLrw = this->RevisionWeight();
    vector<ObjectOpinion> TrustRevOp = this->EgoTrust;

    for(int i=0; i< TrustRevOp.size(); i++){
        auto NewTrust = TrustRevOp[i].GetAdvice()[0] - TrustRevOp[i].GetAdvice()[0] * SLrw(i);
        auto NewDistrust = TrustRevOp[i].GetAdvice()[1] + (1 - TrustRevOp[i].GetAdvice()[1]) * SLrw(i);
        auto NewU = TrustRevOp[i].GetU() - TrustRevOp[i].GetU() * SLrw(i);
        xt::xarray<double> NewAdvice = { NewTrust, NewDistrust};
        TrustRevOp[i].SetAdvice(NewAdvice);
        TrustRevOp[i].SetU(NewU);
    }
    return TrustRevOp;
}

/*********************************************************************************
 * @brief Trust Revised Average Fusion
 * *******************************************************************************/
ObjectOpinion SubjectiveLogicFusion::TrustRevisedAvgFusion(){
    vector<ObjectOpinion> TrustRevOp = this->TrustRevisedOpinion();
    ObjectOpinion RevAvgFusion = this->AvgTrustFusion(TrustRevOp,this->SourceAdvice);
    return RevAvgFusion;
}



/*********************************************************************************
 * @brief Trust Revised Cumlative Fusion
 * *******************************************************************************/
ObjectOpinion SubjectiveLogicFusion::TrustRevisedCumFusion(){
    vector<ObjectOpinion> TrustRevOp = this->TrustRevisedOpinion();
    ObjectOpinion RevCumFusion = this->CumTrustFusion(TrustRevOp,this->SourceAdvice);
    return RevCumFusion;
}