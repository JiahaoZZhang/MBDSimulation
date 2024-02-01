/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "AltitudeConfidence.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_type_AltitudeConfidence_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_AltitudeConfidence_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  15 }	/* (0..15) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_AltitudeConfidence_value2enum_1[] = {
	{ 0,	10,	"alt-000-01" },
	{ 1,	10,	"alt-000-02" },
	{ 2,	10,	"alt-000-05" },
	{ 3,	10,	"alt-000-10" },
	{ 4,	10,	"alt-000-20" },
	{ 5,	10,	"alt-000-50" },
	{ 6,	10,	"alt-001-00" },
	{ 7,	10,	"alt-002-00" },
	{ 8,	10,	"alt-005-00" },
	{ 9,	10,	"alt-010-00" },
	{ 10,	10,	"alt-020-00" },
	{ 11,	10,	"alt-050-00" },
	{ 12,	10,	"alt-100-00" },
	{ 13,	10,	"alt-200-00" },
	{ 14,	10,	"outOfRange" },
	{ 15,	11,	"unavailable" }
};
static const unsigned int asn_MAP_AltitudeConfidence_enum2value_1[] = {
	0,	/* alt-000-01(0) */
	1,	/* alt-000-02(1) */
	2,	/* alt-000-05(2) */
	3,	/* alt-000-10(3) */
	4,	/* alt-000-20(4) */
	5,	/* alt-000-50(5) */
	6,	/* alt-001-00(6) */
	7,	/* alt-002-00(7) */
	8,	/* alt-005-00(8) */
	9,	/* alt-010-00(9) */
	10,	/* alt-020-00(10) */
	11,	/* alt-050-00(11) */
	12,	/* alt-100-00(12) */
	13,	/* alt-200-00(13) */
	14,	/* outOfRange(14) */
	15	/* unavailable(15) */
};
const asn_INTEGER_specifics_t asn_SPC_AltitudeConfidence_specs_1 = {
	asn_MAP_AltitudeConfidence_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_AltitudeConfidence_enum2value_1,	/* N => "tag"; sorted by N */
	16,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_AltitudeConfidence_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_AltitudeConfidence = {
	"AltitudeConfidence",
	"AltitudeConfidence",
	&asn_OP_NativeEnumerated,
	asn_DEF_AltitudeConfidence_tags_1,
	sizeof(asn_DEF_AltitudeConfidence_tags_1)
		/sizeof(asn_DEF_AltitudeConfidence_tags_1[0]), /* 1 */
	asn_DEF_AltitudeConfidence_tags_1,	/* Same as above */
	sizeof(asn_DEF_AltitudeConfidence_tags_1)
		/sizeof(asn_DEF_AltitudeConfidence_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		&asn_OER_type_AltitudeConfidence_constr_1,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_AltitudeConfidence_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_AltitudeConfidence_specs_1	/* Additional specs */
};

