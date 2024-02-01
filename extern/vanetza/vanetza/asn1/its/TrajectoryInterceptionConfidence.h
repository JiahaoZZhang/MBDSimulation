/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_TrajectoryInterceptionConfidence_H_
#define	_TrajectoryInterceptionConfidence_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TrajectoryInterceptionConfidence {
	TrajectoryInterceptionConfidence_lessthan50percent	= 0,
	TrajectoryInterceptionConfidence_between50and70Percent	= 1,
	TrajectoryInterceptionConfidence_between70and90Percent	= 2,
	TrajectoryInterceptionConfidence_above90Percent	= 3
} e_TrajectoryInterceptionConfidence;

/* TrajectoryInterceptionConfidence */
typedef long	 TrajectoryInterceptionConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_TrajectoryInterceptionConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_TrajectoryInterceptionConfidence;
asn_struct_free_f TrajectoryInterceptionConfidence_free;
asn_struct_print_f TrajectoryInterceptionConfidence_print;
asn_constr_check_f TrajectoryInterceptionConfidence_constraint;
ber_type_decoder_f TrajectoryInterceptionConfidence_decode_ber;
der_type_encoder_f TrajectoryInterceptionConfidence_encode_der;
xer_type_decoder_f TrajectoryInterceptionConfidence_decode_xer;
xer_type_encoder_f TrajectoryInterceptionConfidence_encode_xer;
oer_type_decoder_f TrajectoryInterceptionConfidence_decode_oer;
oer_type_encoder_f TrajectoryInterceptionConfidence_encode_oer;
per_type_decoder_f TrajectoryInterceptionConfidence_decode_uper;
per_type_encoder_f TrajectoryInterceptionConfidence_encode_uper;
per_type_decoder_f TrajectoryInterceptionConfidence_decode_aper;
per_type_encoder_f TrajectoryInterceptionConfidence_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _TrajectoryInterceptionConfidence_H_ */
#include "asn_internal.h"
