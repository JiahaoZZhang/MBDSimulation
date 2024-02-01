add_artery_feature(F2MD 
F2MD_CAMFacility.cc
F2MD_CPFacility.cc
F2MD_CPService.cc
F2MD_CAMService.cc
F2MD_RsuCPService.cc
F2MD_RsuCPFacility.cc
)

target_sources(F2MD PUBLIC 
    mdFiles/mdBase/NodeTable.cc
    mdFiles/mdBase/NodeHistory.cc
    mdFiles/mdBase/NodeMDMHistory.cc
    mdFiles/mdBase/BsmCheck.cc
    mdFiles/mdBase/CpmCheckList.cc
    
    #add NodeTypeClassification  --jiahao 09/05/2022
    # mdFiles/mdBase/NodeTypeClassification.cc

    mdFiles/mdBase/ExtendPerceptionTable.cc

    mdFiles/mdSupport/kalmanLib/KalmanFilterJ_SC.cc
    mdFiles/mdSupport/kalmanLib/KalmanFilterJ_SI.cc
    mdFiles/mdSupport/kalmanLib/KalmanFilterJ_SVI.cc
    mdFiles/mdSupport/kalmanLib/Kalman_SVI.cc
    mdFiles/mdSupport/kalmanLib/Kalman_SC.cc
    mdFiles/mdSupport/kalmanLib/Kalman_SI.cc
    mdFiles/mdSupport/kalmanLib/MatrixOp_SVI.cc
    mdFiles/mdSupport/kalmanLib/MatrixOp_SC.cc
    mdFiles/mdSupport/kalmanLib/MatrixOp_SI.cc
    mdFiles/mdSupport/networkLinksLib/LinkControl.cc
    mdFiles/mdSupport/networkLinksLib/Link.cc

    # modify 22/04/2022 Jiahao, adding SubjectiveLogicLib
    mdFiles/mdSupport/SubjectiveLogicLib/ObjectOpinion.cc
    mdFiles/mdSupport/SubjectiveLogicLib/SubjectiveLogic.cc
    
    # modify 16/05/2022 Jiahao, adding Conflict Event table
    # mdFiles/mdSupport/SubjectiveLogicLib/ConflictEventTable.cc
    # mdFiles/mdSupport/SubjectiveLogicLib/ConflictEventInfo.cc
    # mdFiles/mdSupport/SubjectiveLogicLib/OpinionConstruction.cc

    # modify 19/07/2022 Jiahao, add Trust Management Functions
    # mdFiles/mdSupport/SubjectiveLogicLib/TrustManagementFunc/TrustManagementFunc.cc

    # add Trust Table --Jiahao 01/06/2022
    # mdFiles/mdBase/NodeTrustTable.cc
    # mdFiles/mdSupport/SubjectiveLogicLib/ConflictEventTrustRevision.cc

    mdFiles/mdAttacks/MDGlobalAttack.cc
    mdFiles/mdAttacks/MDAttack.cc
    mdFiles/mdAttacks/MDCpmAttack.cc
    mdFiles/mdPCPolicies/PCPolicy.cc
    mdFiles/mdPCPolicies/PCPolicyCPM.cc

    #add CPMConflictCheck  --jiahao 30/05/2022
    # mdFiles/mdChecks/CPMConflictCheck.cc
    
    mdFiles/mdSupport/GeneralLib.cc
    mdFiles/mdSupport/XmlWriter.cc
    mdFiles/mdSupport/JsonWriter.cc
    mdFiles/mdSupport/MDMLib.cc
    mdFiles/mdSupport/rectIntLib/RectIntLib.cc
    mdFiles/mdSupport/rectIntLib/Vector.cc
    mdFiles/mdSupport/rectIntLib/Line.cc
    mdFiles/mdSupport/rectIntLib/Rectangle.cc
    mdFiles/mdSupport/ellipseIntLib/EllipseIntLib.cc
    mdFiles/mdSupport/ellipseIntLib/SupportEllipse.cc
    mdFiles/mdSupport/RelativeOffsetConf.cc
    mdFiles/mdSupport/RelativeOffset.cc
    mdFiles/mdSupport/AppPrintable.cc
    mdFiles/mdSupport/HTTPRequest.cc
    mdFiles/mdSupport/BsmPrintable.cc
    mdFiles/mdSupport/VarThrePrintable.cc
    mdFiles/mdSupport/VeReMiPrintable.cc
    mdFiles/mdSupport/LocalAttackServer.cc
    mdFiles/mdStats/MDStatistics.cc
    mdFiles/mdStats/MDSBase.cc
    
    mdFiles/mdBase/InterTest.cc
    mdFiles/mdBase/NodeHistoryCpm.cc
    mdFiles/mdBase/NodeTableCpm.cc

    mdFiles/mdReport/MDReport.cc
    mdFiles/mdReport/ProtocolEnforcer.cc
    mdFiles/mdReport/ReportPrintable.cc
    mdFiles/mdApplications/MachineLearningApp.cc
    mdFiles/mdApplications/MDApplication.cc
    mdFiles/mdApplications/AggregationApp.cc
    mdFiles/mdApplications/CooperativeApp.cc
    mdFiles/mdApplications/BehavioralApp.cc
    mdFiles/mdApplications/ThresholdApp.cc
    mdFiles/mdApplications/ExperiApp.cc
    
    mdFiles/mdChecks/CaTChChecks.cc
    mdFiles/mdChecks/LegacyChecks.cc
    mdFiles/mdChecks/ExperiChecks.cc
    mdFiles/mdChecks/CpmChecks.cc


    mdFiles/mdReport/OneMessageReport.cc
    mdFiles/mdReport/BasicCheckReport.cc
    mdFiles/mdReport/ProtocolReport.cc
    mdFiles/mdReport/EvidenceReport.cc
)

target_link_libraries(F2MD PUBLIC envmod)


generate_opp_message(BSM.msg TARGET F2MD DIRECTORY F2MD_msgs)