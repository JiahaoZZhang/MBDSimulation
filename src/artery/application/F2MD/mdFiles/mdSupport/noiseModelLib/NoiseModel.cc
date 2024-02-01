#include "NoiseModel.h"



NoiseModel::NoiseModel(/* args */)
{
}




double NoiseModel::getGaussianRand(double mean, double stddev){

    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);

    std::mt19937 rng(tm.tv_nsec);

    std::normal_distribution<> d { mean, stddev };
    return d(rng);
}


/********************************
    Noise model of confidence
 ********************************/

/**
 * @brief 
 * 
 * @param cursemiMinorConfidence 
 * @param cursemiMajorConfidence 
 * @param cursemiMajorOrientation 
 * @return vector<double> 
 */
vector<double> NoiseModel::noisedGeoPositionConfidence(double cursemiMinorConfidence, double cursemiMajorConfidence , double cursemiMajorOrientation){
    
}



/**
 * @brief 
 * 
 * @param curXdistanceConfidence 
 * @param curYdistanceConfidence 
 * @return vector<double> 
 */
vector<double> NoiseModel::noisedDistanceConfidence(double curXdistanceConfidence, double curYdistanceConfidence){

}




double NoiseModel::noisedObjOrientationConfidence(double curObjOrientationConfidence){

}



double NoiseModel::noisedSpeedConfidence(double curSpeedConfidence){

}



double NoiseModel::noisedSpeedDirectionConfidence(double curSpeedDirectionConfidence){

}



double NoiseModel::noisedAccelerationConfidence(double curAccelerationConfidence){

}