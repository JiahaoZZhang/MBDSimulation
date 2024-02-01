/*******************************************************************************
 * @author  Jiahao Zhang
 * @email   jiahao.zhang96@gmail.com
 * @date    14/02/2022
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "ObjectOpinion.h"

/******************************************************************
 * @brief Create an object opinon
 *  
 * @param advice List of opinion
 * @param u Uncertainty
 * @param a List of base rate relevant to opinion
 * ****************************************************************/
ObjectOpinion::ObjectOpinion(const xt::xarray<double> advice,const double u,const xt::xarray<double> a){
  try{

      if (advice.size() != a.size()){
        throw "Advice and BaseRate a should have same size!";
      }

      for (const auto& i : advice){
        if (i < 0. || i > 1.){
          throw "The value of opinion should be between 0 and 1!";
        }
      }

      if (u < 0. || u > 1.){
        throw "The value of uncertainty should be between 0 and 1!";
      }

      auto e = (xt::sum(advice)() + u);
      if (!(0.9999999 < e <= 1)){
        throw "The sum of probability should be 1!";
      }

      for (const auto& i : a){
        if (i < 0. || i > 1.){
          throw "The value of base rate should be between 0 and 1!";
        }
      } 
      
      this->advice = advice;
      this->u = u;
      this->a = a;

      
  }catch(const char* e){
    std::cout << e << std::endl; 
  }
}

/********************************************************************
 * @brief Get the advice
 ********************************************************************/
xt::xarray<double> ObjectOpinion::GetAdvice() const{
  return this->advice;
}

/********************************************************************
 * @brief Get the uncertainty
 ********************************************************************/
double ObjectOpinion::GetU() const{
  return this->u;
}

/********************************************************************
 * @brief Get the base rate
 ********************************************************************/
xt::xarray<double> ObjectOpinion::GetBaseRate() const{
  return this->a;
}

/********************************************************************
 * @brief Set the advice
 ********************************************************************/
void ObjectOpinion::SetAdvice(xt::xarray<double> _advice){
  this->advice = _advice;
}

/********************************************************************
 * @brief Set the uncertainty
 ********************************************************************/
void ObjectOpinion::SetU(double _u){
  this->u = _u;
}

/********************************************************************
 * @brief Set the base rate
 ********************************************************************/
void ObjectOpinion::SetBaseRate(xt::xarray<double> _a){
  this->a = _a;
}


/********************************************************************
 * @brief Get the project probability of an opinion
 ********************************************************************/
xt::xarray<double> ObjectOpinion::GetProjProba() const{
  auto GPP = this->advice + this->u * this->a;
  return GPP;
}

// Object comparator
bool ObjectOpinion::operator ==(const ObjectOpinion& _opinion) const{
        return (this->advice == _opinion.GetAdvice()) && (this->a == _opinion.GetBaseRate()) && (this->u == _opinion.GetU());
    }


// Deep Copy class ObjectOpinion
ObjectOpinion::ObjectOpinion(const ObjectOpinion& obj){
  this->advice = obj.advice;
  this->u = obj.u;
  this->a = obj.a;
}

ObjectOpinion ObjectOpinion::operator =(const ObjectOpinion& e){
  this->advice = e.advice;
  this->u = e.u;
  this->a = e.a;

  return *this;
}

