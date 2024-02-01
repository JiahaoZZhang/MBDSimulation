/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    14/02/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __ObjectOpinion__
#define __ObjectOpinion__

#include <iostream>
#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmath.hpp>

using namespace std;
using namespace xt;

class ObjectOpinion{

    private:
        xt::xarray<double> advice;
        double u;
        xt::xarray<double> a;
        
    public: 
        ObjectOpinion(){ this->advice = {0,0}; this->u = 0; this->a = {0,0}; };
        ObjectOpinion(const xt::xarray<double> advice,const double u,const xt::xarray<double> a);

        xt::xarray<double> GetProjProba() const;
        xt::xarray<double> GetAdvice() const;
        double GetU() const;
        xt::xarray<double> GetBaseRate() const;

        void SetAdvice(xt::xarray<double> _advice);
        void SetU(double _u);
        void SetBaseRate(xt::xarray<double> _a);

        bool operator ==(const ObjectOpinion& _opinion) const;
        
        ObjectOpinion operator =(const ObjectOpinion& e);

        ObjectOpinion(const ObjectOpinion& Obj);
        ~ObjectOpinion(){};
};

#endif