/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_ClusterLeaveReason_H_
#define	_ClusterLeaveReason_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeEnumerated.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ClusterLeaveReason {
	ClusterLeaveReason_notProvided	= 0,
	ClusterLeaveReason_clusterLeaderLost	= 1,
	ClusterLeaveReason_clusterDisbandedByLeader	= 2,
	ClusterLeaveReason_outOfClusterBoundingBox	= 3,
	ClusterLeaveReason_outOfClusterSpeedRange	= 4,
	ClusterLeaveReason_joiningAnotherCluster	= 5,
	ClusterLeaveReason_cancelledJoin	= 6,
	ClusterLeaveReason_failedJoin	= 7,
	ClusterLeaveReason_safetyCondition	= 8,
	ClusterLeaveReason_max	= 15
} e_ClusterLeaveReason;

/* ClusterLeaveReason */
typedef long	 ClusterLeaveReason_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ClusterLeaveReason_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ClusterLeaveReason;
extern const asn_INTEGER_specifics_t asn_SPC_ClusterLeaveReason_specs_1;
asn_struct_free_f ClusterLeaveReason_free;
asn_struct_print_f ClusterLeaveReason_print;
asn_constr_check_f ClusterLeaveReason_constraint;
ber_type_decoder_f ClusterLeaveReason_decode_ber;
der_type_encoder_f ClusterLeaveReason_encode_der;
xer_type_decoder_f ClusterLeaveReason_decode_xer;
xer_type_encoder_f ClusterLeaveReason_encode_xer;
oer_type_decoder_f ClusterLeaveReason_decode_oer;
oer_type_encoder_f ClusterLeaveReason_encode_oer;
per_type_decoder_f ClusterLeaveReason_decode_uper;
per_type_encoder_f ClusterLeaveReason_encode_uper;
per_type_decoder_f ClusterLeaveReason_decode_aper;
per_type_encoder_f ClusterLeaveReason_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ClusterLeaveReason_H_ */
#include "asn_internal.h"
