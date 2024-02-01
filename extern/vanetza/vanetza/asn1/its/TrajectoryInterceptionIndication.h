/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_TrajectoryInterceptionIndication_H_
#define	_TrajectoryInterceptionIndication_H_


#include "asn_application.h"

/* Including external dependencies */
#include "StationId.h"
#include "TrajectoryInterceptionProbability.h"
#include "TrajectoryInterceptionConfidence.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TrajectoryInterceptionIndication */
typedef struct TrajectoryInterceptionIndication {
	StationId_t	*subjectStation;	/* OPTIONAL */
	TrajectoryInterceptionProbability_t	 trajectoryInterceptionProbability;
	TrajectoryInterceptionConfidence_t	*trajectoryInterceptionConfidence;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TrajectoryInterceptionIndication_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TrajectoryInterceptionIndication;
extern asn_SEQUENCE_specifics_t asn_SPC_TrajectoryInterceptionIndication_specs_1;
extern asn_TYPE_member_t asn_MBR_TrajectoryInterceptionIndication_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _TrajectoryInterceptionIndication_H_ */
#include "asn_internal.h"
