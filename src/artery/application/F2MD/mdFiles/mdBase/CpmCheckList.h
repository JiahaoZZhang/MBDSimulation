
#ifndef __CPMCHECKLIST__
#define __CPMCHECKLIST__

#include <vector>


using namespace std;

struct AttackedObject{
    int ObjectID;

    // distancePlausibility, bit 1
    // speedPlausibility, bit 2
    // speedConsistency, bit 3
    // positionSpeedConsistency, bit 4
    // kalmanPositionConsistency, bit 5
    // kalmanSpeedConsistency, bit 6
    // kalmanPositionSpeedConsistancyP, bit 7
    // kalmanPositionSpeedConsistancyS, bit 8
    // All activated, ff
    // All deactivated, 00 
    int Checkfailed;
};





class CpmCheckList{
    private:
        double distancePlausibility;
        double speedPlausibility;
        double speedConsistency;
        double positionSpeedConsistency;

        double kalmanPositionConsistency;
        double kalmanSpeedConsistency;
        double kalmanPositionSpeedConsistancyP;
        double kalmanPositionSpeedConsistancyS;
        
        vector<AttackedObject> attackedObject;
        bool report;

    public:
        CpmCheckList();

        void setdistancePlausibility(double val);
        double getdistancePlausibility();
        
        void setspeedPlausibility(double val);
        double getspeedPlausibility();
        
        void setspeedConsistency(double val);    
        double getspeedConsistency();

        void setpositionSpeedConsistency(double val);
        double getpositionSpeedConsistency();

        void setkalmanPositionConsistency(double val);
        double getkalmanPositionConsistency();

        void setkalmanSpeedConsistency(double val);
        double getkalmanSpeedConsistency();

        void setkalmanPositionSpeedConsistancyP(double val);
        double getkalmanPositionSpeedConsistancyP();

        void setkalmanPositionSpeedConsistancyS(double val);
        double getkalmanPositionSpeedConsistancyS(); 

        void setreport(bool val);
        bool getreport();

        void setAttackedObject(AttackedObject objectID);
        vector<AttackedObject> getAttackedObject();
        void resetAll();
};

#endif
