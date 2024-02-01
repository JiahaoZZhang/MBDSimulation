/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_DigitalMap_H_
#define	_DigitalMap_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ReferencePosition;

/* DigitalMap */
typedef struct DigitalMap {
	A_SEQUENCE_OF(struct ReferencePosition) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DigitalMap_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DigitalMap;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ReferencePosition.h"

#endif	/* _DigitalMap_H_ */
#include "asn_internal.h"
