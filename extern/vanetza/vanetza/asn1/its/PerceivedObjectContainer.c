/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PerceivedObjectContainer"
 * 	found in "asn1/CPM_TS_0054/CPM-PerceivedObjectContainer.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "PerceivedObjectContainer.h"

static asn_TYPE_member_t asn_MBR_PerceivedObjectContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PerceivedObjectContainer, numberOfPerceivedObjects),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CardinalNumber1B,
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
		"numberOfPerceivedObjects"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PerceivedObjectContainer, perceivedObjects),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PerceivedObjects,
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
		"perceivedObjects"
		},
};
static const ber_tlv_tag_t asn_DEF_PerceivedObjectContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PerceivedObjectContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* numberOfPerceivedObjects */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* perceivedObjects */
};
static asn_SEQUENCE_specifics_t asn_SPC_PerceivedObjectContainer_specs_1 = {
	sizeof(struct PerceivedObjectContainer),
	offsetof(struct PerceivedObjectContainer, _asn_ctx),
	asn_MAP_PerceivedObjectContainer_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	2,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_PerceivedObjectContainer = {
	"PerceivedObjectContainer",
	"PerceivedObjectContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_PerceivedObjectContainer_tags_1,
	sizeof(asn_DEF_PerceivedObjectContainer_tags_1)
		/sizeof(asn_DEF_PerceivedObjectContainer_tags_1[0]), /* 1 */
	asn_DEF_PerceivedObjectContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_PerceivedObjectContainer_tags_1)
		/sizeof(asn_DEF_PerceivedObjectContainer_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_PerceivedObjectContainer_1,
	2,	/* Elements count */
	&asn_SPC_PerceivedObjectContainer_specs_1	/* Additional specs */
};

