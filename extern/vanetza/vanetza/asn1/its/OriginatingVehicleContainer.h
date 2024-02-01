/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-OriginatingStationContainers"
 * 	found in "asn1/CPM_TS_0054/CPM-OriginatingStationContainers.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_OriginatingVehicleContainer_H_
#define	_OriginatingVehicleContainer_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Wgs84Angle.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct CartesianAngle;
struct TrailerDataSet;

/* OriginatingVehicleContainer */
typedef struct OriginatingVehicleContainer {
	Wgs84Angle_t	 orientationAngle;
	struct CartesianAngle	*pitchAngle;	/* OPTIONAL */
	struct CartesianAngle	*rollAngle;	/* OPTIONAL */
	struct TrailerDataSet	*trailerDataSet;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} OriginatingVehicleContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_OriginatingVehicleContainer;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CartesianAngle.h"
#include "TrailerDataSet.h"

#endif	/* _OriginatingVehicleContainer_H_ */
#include "asn_internal.h"
