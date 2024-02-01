package artery.application.F2MD;
import artery.application.ItsG5Service;

simple F2MD_CPService like ItsG5Service
{
    // CPM Parameters filled in omnetpp.ini will be sent here to be stored in F2MD_CPParameters during the initialization
    // F2MD_CPParameters.h must not be modified manually. All modifications are made in the omnetpp.ini file of the scenario
    @signal[CpmSent](type=CpmObject); 
    @signal[CpmReceived](type=CpmObject);

    @signal[prediction](type=double);
    @signal[myID](type=double);
    @signal[percvdObj](type=double);
    @signal[senderID](type=double);

    @statistic[prediction](source=prediction;record=vector; interpolationmode=none);
    @statistic[myID](source=myID;record=vector; interpolationmode=none);
    @statistic[percvdObj](source=percvdObj;record=vector; interpolationmode=none);
    @statistic[senderID](source=senderID;record=vector; interpolationmode=none);

    double maxInterval @unit(s) = default(1.0s);
    bool fixedRate = default(true);
    int Test_mode = default(0);
    bool KeepSameID = default(true);


    // Attacks parameters
    bool MixCpLocalAttacks = default(false);
    bool RandomCpLocalMix = default(false);

    double CP_LOCAL_ATTACKER_PROB = default(0.00);
    int CP_LOCAL_ATTACK_TYPE = default(0);


    // Detection parameters
    double MaxRadarRange = default(200.0);
    double MaxRadarAngle = default(20.0);


    // Advanced parameters
    double START_CPM_ATTACK = default(10);
    double MAX_CPM_DELTATIME = default(5);
    double MAX_CPM_HISTORY_TIME = default(15);

    double cpParVar = default(0.55);
    double RandomDistOffsetX = default(70.0);
    double RandomDistOffsetY = default(70.0);
    double RandomSpeed = default(40.0);
    double RandomSpeedOffset = default(7.0);
    double RandomAccel = default(2.0);
    double MaxRadius = default(40.0);
    double MaxMapBoundary = default(200.0);

    double MAX_PLAUSIBLE_ANGLE_CHANGE = default(180.0);
    double TOLERANCE_EXCEED_SPEED = default(3.0);
    double TOLERANCE_DEARTH_SPEED = default(5.0);

    double MAX_PLAUSIBLE_SPEED = default(70.0);
    double MAX_PLAUSIBLE_ACCEL = default(3.0);
    double MAX_PLAUSIBLE_DECEL = default(4.5);

    int SegmentAngle = default(4); 

    // Kalman SETTING 
    double KALMAN_MIN_POS_RANGE = default(1.0);
    double MAX_KALMAN_TIME = default(3.1);
    double KALMAN_POS_RANGE = default(4.0);    
    double KALMAN_MIN_SPEED_RANGE = default(1.0);
    double KALMAN_SPEED_RANGE = default(4.0);

    // Vehicle Characteristics
    double VEHICLE_LENGTH = default(3.6);
    double VEHICLE_WIDTH = default(1.8);

    // PseudoChange Parameters
    double Period_Change_Time = default(240);
    int Tolerance_Buffer = default(10);
    double Period_Change_Distance = default(80);
    double Random_Change_Chance = default(0.1);
    bool EnablePC = default(false);
    int PC_TYPE = default(5);
    string pathToMacModule = default(".wlan[*].mac");

}