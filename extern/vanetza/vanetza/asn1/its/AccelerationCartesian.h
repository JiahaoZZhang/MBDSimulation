/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_AccelerationCartesian_H_
#define	_AccelerationCartesian_H_


#include "asn_application.h"

/* Including external dependencies */
#include "AccelerationComponent.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct AccelerationComponent;

/* AccelerationCartesian */
typedef struct AccelerationCartesian {
	AccelerationComponent_t	 xAcceleration;
	AccelerationComponent_t	 yAcceleration;
	struct AccelerationComponent	*zAcceleration;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} AccelerationCartesian_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AccelerationCartesian;
extern asn_SEQUENCE_specifics_t asn_SPC_AccelerationCartesian_specs_1;
extern asn_TYPE_member_t asn_MBR_AccelerationCartesian_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "AccelerationComponent.h"

#endif	/* _AccelerationCartesian_H_ */
#include "asn_internal.h"