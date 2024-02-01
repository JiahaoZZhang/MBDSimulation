/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "MessageSegmentationInfo.h"

asn_TYPE_member_t asn_MBR_MessageSegmentationInfo_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct MessageSegmentationInfo, totalMsgNo),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CardinalNumber3b,
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
		"totalMsgNo"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct MessageSegmentationInfo, thisMsgNo),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OrdinalNumber3b,
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
		"thisMsgNo"
		},
};
static const ber_tlv_tag_t asn_DEF_MessageSegmentationInfo_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_MessageSegmentationInfo_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* totalMsgNo */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* thisMsgNo */
};
asn_SEQUENCE_specifics_t asn_SPC_MessageSegmentationInfo_specs_1 = {
	sizeof(struct MessageSegmentationInfo),
	offsetof(struct MessageSegmentationInfo, _asn_ctx),
	asn_MAP_MessageSegmentationInfo_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_MessageSegmentationInfo = {
	"MessageSegmentationInfo",
	"MessageSegmentationInfo",
	&asn_OP_SEQUENCE,
	asn_DEF_MessageSegmentationInfo_tags_1,
	sizeof(asn_DEF_MessageSegmentationInfo_tags_1)
		/sizeof(asn_DEF_MessageSegmentationInfo_tags_1[0]), /* 1 */
	asn_DEF_MessageSegmentationInfo_tags_1,	/* Same as above */
	sizeof(asn_DEF_MessageSegmentationInfo_tags_1)
		/sizeof(asn_DEF_MessageSegmentationInfo_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_MessageSegmentationInfo_1,
	2,	/* Elements count */
	&asn_SPC_MessageSegmentationInfo_specs_1	/* Additional specs */
};

