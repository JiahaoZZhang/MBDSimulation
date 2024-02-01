/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "CartesianCoordinateWithConfidence.h"

asn_TYPE_member_t asn_MBR_CartesianCoordinateWithConfidence_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CartesianCoordinateWithConfidence, value),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CartesianCoordinateLarge,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"value"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CartesianCoordinateWithConfidence, confidence),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CoordinateConfidence,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"confidence"
		},
};
static const ber_tlv_tag_t asn_DEF_CartesianCoordinateWithConfidence_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CartesianCoordinateWithConfidence_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* value */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* confidence */
};
asn_SEQUENCE_specifics_t asn_SPC_CartesianCoordinateWithConfidence_specs_1 = {
	sizeof(struct CartesianCoordinateWithConfidence),
	offsetof(struct CartesianCoordinateWithConfidence, _asn_ctx),
	asn_MAP_CartesianCoordinateWithConfidence_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CartesianCoordinateWithConfidence = {
	"CartesianCoordinateWithConfidence",
	"CartesianCoordinateWithConfidence",
	&asn_OP_SEQUENCE,
	asn_DEF_CartesianCoordinateWithConfidence_tags_1,
	sizeof(asn_DEF_CartesianCoordinateWithConfidence_tags_1)
		/sizeof(asn_DEF_CartesianCoordinateWithConfidence_tags_1[0]), /* 1 */
	asn_DEF_CartesianCoordinateWithConfidence_tags_1,	/* Same as above */
	sizeof(asn_DEF_CartesianCoordinateWithConfidence_tags_1)
		/sizeof(asn_DEF_CartesianCoordinateWithConfidence_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_CartesianCoordinateWithConfidence_1,
	2,	/* Elements count */
	&asn_SPC_CartesianCoordinateWithConfidence_specs_1	/* Additional specs */
};

