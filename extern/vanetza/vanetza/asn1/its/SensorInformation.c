/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-SensorInformationContainer"
 * 	found in "asn1/CPM_TS_0054/CPM-SensorInformationContainer.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "SensorInformation.h"

asn_TYPE_member_t asn_MBR_SensorInformation_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SensorInformation, sensorId),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Identifier1B,
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
		"sensorId"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SensorInformation, sensorType),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SensorType,
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
		"sensorType"
		},
	{ ATF_POINTER, 2, offsetof(struct SensorInformation, perceptionRegionShape),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_Shape,
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
		"perceptionRegionShape"
		},
	{ ATF_POINTER, 1, offsetof(struct SensorInformation, perceptionRegionConfidence),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ConfidenceLevel,
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
		"perceptionRegionConfidence"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SensorInformation, shadowingApplies),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BOOLEAN,
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
		"shadowingApplies"
		},
};
static const int asn_MAP_SensorInformation_oms_1[] = { 2, 3 };
static const ber_tlv_tag_t asn_DEF_SensorInformation_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SensorInformation_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* sensorId */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* sensorType */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* perceptionRegionShape */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* perceptionRegionConfidence */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* shadowingApplies */
};
asn_SEQUENCE_specifics_t asn_SPC_SensorInformation_specs_1 = {
	sizeof(struct SensorInformation),
	offsetof(struct SensorInformation, _asn_ctx),
	asn_MAP_SensorInformation_tag2el_1,
	5,	/* Count of tags in the map */
	asn_MAP_SensorInformation_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	5,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SensorInformation = {
	"SensorInformation",
	"SensorInformation",
	&asn_OP_SEQUENCE,
	asn_DEF_SensorInformation_tags_1,
	sizeof(asn_DEF_SensorInformation_tags_1)
		/sizeof(asn_DEF_SensorInformation_tags_1[0]), /* 1 */
	asn_DEF_SensorInformation_tags_1,	/* Same as above */
	sizeof(asn_DEF_SensorInformation_tags_1)
		/sizeof(asn_DEF_SensorInformation_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_SensorInformation_1,
	5,	/* Elements count */
	&asn_SPC_SensorInformation_specs_1	/* Additional specs */
};

