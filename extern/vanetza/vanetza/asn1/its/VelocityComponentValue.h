/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_VelocityComponentValue_H_
#define	_VelocityComponentValue_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VelocityComponentValue {
	VelocityComponentValue_negativeOutOfRange	= -16383,
	VelocityComponentValue_positiveOutOfRange	= 16382,
	VelocityComponentValue_unavailable	= 16383
} e_VelocityComponentValue;

/* VelocityComponentValue */
typedef long	 VelocityComponentValue_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_VelocityComponentValue_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_VelocityComponentValue;
asn_struct_free_f VelocityComponentValue_free;
asn_struct_print_f VelocityComponentValue_print;
asn_constr_check_f VelocityComponentValue_constraint;
ber_type_decoder_f VelocityComponentValue_decode_ber;
der_type_encoder_f VelocityComponentValue_encode_der;
xer_type_decoder_f VelocityComponentValue_decode_xer;
xer_type_encoder_f VelocityComponentValue_encode_xer;
oer_type_decoder_f VelocityComponentValue_decode_oer;
oer_type_encoder_f VelocityComponentValue_encode_oer;
per_type_decoder_f VelocityComponentValue_decode_uper;
per_type_encoder_f VelocityComponentValue_encode_uper;
per_type_decoder_f VelocityComponentValue_decode_aper;
per_type_encoder_f VelocityComponentValue_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _VelocityComponentValue_H_ */
#include "asn_internal.h"
