/*******************************************************************************
 * @author  Joseph Kamel, Maxime Georges
 * @email   josephekamel@gmail.com, maxime.georges059@gmail.com
 * @date    10/06/2021
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef F2MD_CAMFACILITY_H_
#define F2MD_CAMFACILITY_H_

#include "F2MD_msgs/BSM_m.h"
#include "F2MD_CAMParameters.h"

#include "mdFiles/mdEnumTypes/MbTypes.h"
#include "mdFiles/mdEnumTypes/AttackTypes.h"
#include "mdFiles/mdEnumTypes/ReportTypes.h"
#include "mdFiles/mdEnumTypes/MdAppTypes.h"
#include "mdFiles/mdAttacks/MDAttack.h"
#include "mdFiles/mdAttacks/MDGlobalAttack.h"
#include "mdFiles/mdSupport/GeneralLib.h"
#include "mdFiles/mdSupport/networkLinksLib/LinkControl.h"
#include "mdFiles/mdSupport/RelativeOffsetConf.h"
#include "mdFiles/mdSupport/RelativeOffset.h"
#include "mdFiles/mdSupport/VarThrePrintable.h"
#include "mdFiles/mdSupport/LocalAttackServer.h"
#include "mdFiles/mdSupport/VeReMiPrintable.h"
#include "mdFiles/mdPCPolicies/PCPolicy.h"
#include "mdFiles/mdApplications/MDApplication.h"
#include "mdFiles/mdApplications/ThresholdApp.h"
#include "mdFiles/mdApplications/AggregationApp.h"
#include "mdFiles/mdApplications/BehavioralApp.h"
#include "mdFiles/mdApplications/ExperiApp.h"
#include "mdFiles/mdApplications/ExperiApp.h"
#include "mdFiles/mdApplications/MachineLearningApp.h"
#include "mdFiles/mdApplications/CooperativeApp.h"
#include "mdFiles/mdChecks/CaTChChecks.h"
#include "mdFiles/mdChecks/ExperiChecks.h"
#include "mdFiles/mdChecks/LegacyChecks.h"
#include "mdFiles/mdReport/BasicCheckReport.h"
#include "mdFiles/mdReport/OneMessageReport.h"
#include "mdFiles/mdReport/EvidenceReport.h"
#include "mdFiles/mdReport/ProtocolEnforcer.h"
#include "mdFiles/mdReport/ProtocolReport.h"


#include "traci/LiteAPI.h"
#include "veins/base/utils/Coord.h"
#include "artery/traci/VehicleController.h"
#include "artery/networking/Router.h"
#include "artery/application/ItsG5Service.h"
#include <inet/linklayer/ieee80211/mac/Ieee80211Mac.h>
#include <artery/inet/InetMobility.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/router.hpp>
#include <vanetza/geonet/interface.hpp>
#include <inet/common/ModuleAccess.h>
#include <cmath>
#include <libsumo/TraCIDefs.h>
#include <unordered_map>
#include <chrono>
using namespace std::chrono;

// forward declaration
namespace traci { class VehicleController; }

// Global params
static MDStatistics mdStats = MDStatistics();
static LinkControl linkControl = LinkControl();
static bool linkInit = false;
static VarThrePrintable varThrePrintableV1 = VarThrePrintable("AppV1");
static VarThrePrintable varThrePrintableV2 = VarThrePrintable("AppV2");

static bool setDate = false;
static std::string curDate;

// CAM and attack generation params
static int LastLocalAttackIndex = -1;
static double VeReMiSliceStartTime = 0;

static unsigned long targetNodes[MAX_TARGET_LENGTH];
static int targetNodesLength = 0;
static double targetClearTime = 0;
static unsigned long accusedNodes[MAX_ACCUSED_LENGTH];
static int accusedNodesLength = 0;
static double accusedClearTime = 0;

static std::string attackHost = "localhost";
//static std::string attackHost = "192.168.60.144";
static int attackPort = 9975;
static LocalAttackServer localAttackServer = LocalAttackServer(attackPort, attackHost);


// Misbehavior Detection params
static std::unordered_map<int,veins::Coord> realDynamicMap;

#define mlHostV1 "localhost"
#define mlHostV2 "localhost"
#define mlPortV1 9997
#define mlPortV2 9998

static bool initV1, initV2 = false;
static double deltaTV1, deltaTV2, deltaTVS1, deltaTVS2 = 0;

static CooperativeApp CoopV1 = CooperativeApp(1, 0.5);
static CooperativeApp CoopV2 = CooperativeApp(2, 0.5);

static double meanTimeV1, meanTimeV2 = 0;
static unsigned long numTimeV1, numTimeV2 = 0;

using namespace omnetpp;
using namespace veins;

class F2MD_CAMFacility
{
    private:
        GeneralLib genLib = GeneralLib();

    public:
        F2MD_CAMFacility(){};
        F2MD_CAMFacility(const traci::VehicleController* VehController, cModule* containingNode, inet::MACAddress address, simtime_t beaconInterval, F2MD_CAMParameters& params);
        void initialize();
        void updateVehicleState();
        void treatAttackFlags();
        veins::BSM* createBSM();
        void onBSM(veins::BSM* BSMrecvd);
        void handleReportProtocol(bool lastTimeStep);

        void finish();
        NodeTable detectedNodes;

        /* stats */
        uint32_t generatedBSMs;
        uint32_t receivedBSMs;

    protected:

        F2MD_CAMParameters* params;

        BsmCheck bsmCheckV1;
        BsmCheck bsmCheckV2;

        const traci::VehicleController* mVehicleController;
        
        void LocalMisbehaviorDetection(veins::BSM* bsm, int version);
        void TestReception(veins::BSM* BSMrecvd);
        void setMDApp(mdAppTypes::App appTypeV1, mdAppTypes::App appTypeV2);

        void writeReport(MDReport reportBase, int version, std::string maversion, BsmCheck bsmCheck, BSM* bsm);
        void writeListReport(MDReport reportBase, int version, std::string maversion, BsmCheck bsmCheck, BSM* bsm);
        void sendReport(MDReport reportBase, int version,std::string maversion, BsmCheck bsmCheck, BSM* bsm);

        void writeMdBsm(std::string version, BsmCheck bsmCheck, BSM* bsm);
        void writeMdListBsm(std::string version, BsmCheck bsmCheck,BSM* bsm);
        void writeSelfBsm(BSM bsm);
        void writeSelfListBsm(BSM bsm);

        mbTypes::Mbs induceMisbehavior(double localAttacker, double globalAttacker);
        void addTargetNode(unsigned long id);
        void addAccusedNode(unsigned long id);
        bool isAccusedNode(unsigned long id);
        void clearAccusedNodes();
        void clearTargetNodes();
        bool isTargetNode(unsigned long id);
        void addMyBsm(BSM bsm);

        void printVehData();

        double MaxRandomPosX;
        double MaxRandomPosY;
        double MinRandomPosX;
        double MinRandomPosY;

        // F2MD
        double lastPositionUpdate;
        veins::Coord curPosition;
        veins::Coord curSpeed;
        veins::Coord curHeading;
        veins::Coord curAccel;
        veins::Coord curGPSCoordinates;
        double myWidth;
        double myLength;

        veins::Coord curPositionConfidence;
        veins::Coord curSpeedConfidence;
        veins::Coord curHeadingConfidence;
        veins::Coord curAccelConfidence;

        veins::Coord curPositionConfidenceOrig;
        veins::Coord curSpeedConfidenceOrig;
        veins::Coord curHeadingConfidenceOrig;
        veins::Coord curAccelConfidenceOrig;

        veins::Coord ConfPosMax;
        veins::Coord ConfSpeedMax;
        veins::Coord ConfHeadMax;
        veins::Coord ConfAccelMax;

        double deltaConfPos = 0;
        double deltaConfSpeed = 0;
        double deltaConfHead = 0;
        double deltaConfAccel = 0;

        double deltaRPosition = 0;
        double deltaThetaPosition = 0;
        double deltaSpeed = 0;
        double deltaHeading = 0;
        double deltaAccel = 0;

        inet::MACAddress myId;
        cModule* node;

        MDAttack mdAttack;
        MDGlobalAttack mdGlobalAttack;

        pseudoChangeTypes::PseudoChange myPcType;
        PCPolicy pcPolicy;

        mbTypes::Mbs myMdType;
        attackTypes::Attacks myAttackType;
        reportTypes::Report myReportType;
        std::string myVType;
        BSM myBsm[MYBSM_SIZE];
        int myBsmNum = 0;
        BSM attackBsm = BSM();
        BSM nextAttackBsm = BSM();

        unsigned long myPseudonym;
        int pseudoNum;

        simtime_t beaconInterval;
        simtime_t lastBeacon;

        //MbDetection

        double MAX_PLAUSIBLE_ACCEL = 0;
        double MAX_PLAUSIBLE_DECEL = 0;
        double MAX_PLAUSIBLE_SPEED = 0;

        MDApplication *AppV1;
        MDApplication *AppV2;

        ThresholdApp ThreV1 = ThresholdApp(1, 0.28125);
        ThresholdApp ThreV2 = ThresholdApp(2, 0.28125);

        AggrigationApp AggrV1 = AggrigationApp(1, 0.28125,0.5, 10.0, 3);
        AggrigationApp AggrV2 = AggrigationApp(2, 0.28125,0.5, 10.0, 3);

        BehavioralApp BehaV1 = BehavioralApp(1, 0.5);
        BehavioralApp BehaV2 = BehavioralApp(2, 0.5);

        ExperiApp ExperV1 = ExperiApp(1, 10.0, 10, 3);
        ExperiApp ExperV2 = ExperiApp(2, 10.0, 10, 3);

        MachineLearningApp PybgV1 = MachineLearningApp(1, mlPortV1, mlHostV1);
        MachineLearningApp PybgV2 = MachineLearningApp(2, mlPortV2, mlHostV2);

        ProtocolEnforcer reportProtocolEnforcerV1 = ProtocolEnforcer();
        ProtocolEnforcer reportProtocolEnforcerV2 = ProtocolEnforcer();

        VeReMiPrintable VeReMi = VeReMiPrintable();

};

#endif /* F2MD_CAMFACILITY_H_ */
