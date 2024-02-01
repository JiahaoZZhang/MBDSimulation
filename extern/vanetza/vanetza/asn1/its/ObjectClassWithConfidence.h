/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_ObjectClassWithConfidence_H_
#define	_ObjectClassWithConfidence_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ObjectClass.h"
#include "ConfidenceLevel.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ObjectClassWithConfidence */
typedef struct ObjectClassWithConfidence {
	ObjectClass_t	 objectClass;
	ConfidenceLevel_t	 confidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ObjectClassWithConfidence_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ObjectClassWithConfidence;
extern asn_SEQUENCE_specifics_t asn_SPC_ObjectClassWithConfidence_specs_1;
extern asn_TYPE_member_t asn_MBR_ObjectClassWithConfidence_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _ObjectClassWithConfidence_H_ */
#include "asn_internal.h"
