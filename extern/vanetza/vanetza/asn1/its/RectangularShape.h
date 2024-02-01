/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_RectangularShape_H_
#define	_RectangularShape_H_


#include "asn_application.h"

/* Including external dependencies */
#include "StandardLength12b.h"
#include "Wgs84AngleValue.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct CartesianPosition3d;

/* RectangularShape */
typedef struct RectangularShape {
	struct CartesianPosition3d	*centerPoint;	/* OPTIONAL */
	StandardLength12b_t	 semiLength;
	StandardLength12b_t	 semiBreadth;
	Wgs84AngleValue_t	*orientation;	/* OPTIONAL */
	StandardLength12b_t	*height;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RectangularShape_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RectangularShape;
extern asn_SEQUENCE_specifics_t asn_SPC_RectangularShape_specs_1;
extern asn_TYPE_member_t asn_MBR_RectangularShape_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CartesianPosition3d.h"

#endif	/* _RectangularShape_H_ */
#include "asn_internal.h"