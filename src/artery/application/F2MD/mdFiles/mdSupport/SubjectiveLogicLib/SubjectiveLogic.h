/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    14/02/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef  __SubjectiveLogic__
#define  __SubjectiveLogic__

#include <iostream>
#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xreducer.hpp>
#include <iterator>
#include <vector>
#include "ObjectOpinion.h"

using namespace std;
using namespace xt;

class SubjectiveLogicFusion{
    
    private:
        vector<ObjectOpinion> EgoTrust;
        vector<ObjectOpinion> SourceAdvice;

    public:
        SubjectiveLogicFusion(vector<ObjectOpinion> Ego,vector<ObjectOpinion> Source);
        vector<ObjectOpinion> TrustDiscount(vector<ObjectOpinion> EgoTrust, vector<ObjectOpinion> SourceAdvice);
        ObjectOpinion AvgTrustFusion(vector<ObjectOpinion> EgoTrust, vector<ObjectOpinion> SourceAdvice);
        ObjectOpinion CumTrustFusion(vector<ObjectOpinion> EgoTrust, vector<ObjectOpinion> SourceAdvice);
        xt::xarray<double> ProjectProbabilityDistance();
        xt::xarray<double> ConjunctiveConfidence();
        xt::xarray<double> BeliefConflict();
        double MaxConflict();
        double AvgConflict();
        xt::xarray<double> RevisionWeight();
        vector<ObjectOpinion> TrustRevisedOpinion();
        ObjectOpinion TrustRevisedAvgFusion();
        ObjectOpinion TrustRevisedCumFusion();

};

#endif
