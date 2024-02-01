#ifndef __NOISE_MODEL_H__
#define __NOISE_MODEL_H__

#include <ctime>
#include <random>
#include <vector>

using namespace std;


class NoiseModel
{
private:

    double getGaussianRand(double mean, double stddev); // Gaussian random generator


public:
    NoiseModel(/* args */);

    // noise model of confidence
    vector<double> noisedGeoPositionConfidence(double cursemiMinorConfidence, double cursemiMajorConfidence , double cursemiMajorOrientation);
    vector<double> noisedDistanceConfidence(double curXdistanceConfidence, double curYdistanceConfidence);
    double noisedObjOrientationConfidence(double curObjOrientationConfidence);
    double noisedSpeedConfidence(double curSpeedConfidence);
    double noisedSpeedDirectionConfidence(double curSpeedDirectionConfidence);
    double noisedAccelerationConfidence(double curAccelerationConfidence);

    // noise model of kinematic data
    vector<double> noisedGeoPosition(double curLongitude, double curLatitude, double cursemiMajorOrientation);
    vector<double> noisedDistance(double curXdistance, double curYdistance);
    double noisedObjOrientation(double curObjOrientation);
    double noisedSpeed(double curSpeed);
    double noisedSpeedDirection(double curSpeedDirection);
    double noisedAcceleration(double curAcceleration);    

    ~NoiseModel(){};

};



#endif

