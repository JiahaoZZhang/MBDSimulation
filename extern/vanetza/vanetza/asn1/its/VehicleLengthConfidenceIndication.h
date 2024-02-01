/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_VehicleLengthConfidenceIndication_H_
#define	_VehicleLengthConfidenceIndication_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeEnumerated.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VehicleLengthConfidenceIndication {
	VehicleLengthConfidenceIndication_noTrailerPresent	= 0,
	VehicleLengthConfidenceIndication_trailerPresentWithKnownLength	= 1,
	VehicleLengthConfidenceIndication_trailerPresentWithUnknownLength	= 2,
	VehicleLengthConfidenceIndication_trailerPresenceIsUnknown	= 3,
	VehicleLengthConfidenceIndication_unavailable	= 4
} e_VehicleLengthConfidenceIndication;

/* VehicleLengthConfidenceIndication */
typedef long	 VehicleLengthConfidenceIndication_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_VehicleLengthConfidenceIndication_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_VehicleLengthConfidenceIndication;
extern const asn_INTEGER_specifics_t asn_SPC_VehicleLengthConfidenceIndication_specs_1;
asn_struct_free_f VehicleLengthConfidenceIndication_free;
asn_struct_print_f VehicleLengthConfidenceIndication_print;
asn_constr_check_f VehicleLengthConfidenceIndication_constraint;
ber_type_decoder_f VehicleLengthConfidenceIndication_decode_ber;
der_type_encoder_f VehicleLengthConfidenceIndication_encode_der;
xer_type_decoder_f VehicleLengthConfidenceIndication_decode_xer;
xer_type_encoder_f VehicleLengthConfidenceIndication_encode_xer;
oer_type_decoder_f VehicleLengthConfidenceIndication_decode_oer;
oer_type_encoder_f VehicleLengthConfidenceIndication_encode_oer;
per_type_decoder_f VehicleLengthConfidenceIndication_decode_uper;
per_type_encoder_f VehicleLengthConfidenceIndication_encode_uper;
per_type_decoder_f VehicleLengthConfidenceIndication_decode_aper;
per_type_encoder_f VehicleLengthConfidenceIndication_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _VehicleLengthConfidenceIndication_H_ */
#include "asn_internal.h"
