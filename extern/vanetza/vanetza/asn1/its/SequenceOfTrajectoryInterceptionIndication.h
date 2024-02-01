/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_SequenceOfTrajectoryInterceptionIndication_H_
#define	_SequenceOfTrajectoryInterceptionIndication_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct TrajectoryInterceptionIndication;

/* SequenceOfTrajectoryInterceptionIndication */
typedef struct SequenceOfTrajectoryInterceptionIndication {
	A_SEQUENCE_OF(struct TrajectoryInterceptionIndication) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SequenceOfTrajectoryInterceptionIndication_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SequenceOfTrajectoryInterceptionIndication;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "TrajectoryInterceptionIndication.h"

#endif	/* _SequenceOfTrajectoryInterceptionIndication_H_ */
#include "asn_internal.h"
