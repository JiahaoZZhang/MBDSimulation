/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "asn1/CPM_TS_0054/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "DangerousGoodsBasic.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_type_DangerousGoodsBasic_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_DangerousGoodsBasic_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 5,  5,  0,  19 }	/* (0..19) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_DangerousGoodsBasic_value2enum_1[] = {
	{ 0,	11,	"explosives1" },
	{ 1,	11,	"explosives2" },
	{ 2,	11,	"explosives3" },
	{ 3,	11,	"explosives4" },
	{ 4,	11,	"explosives5" },
	{ 5,	11,	"explosives6" },
	{ 6,	14,	"flammableGases" },
	{ 7,	17,	"nonFlammableGases" },
	{ 8,	10,	"toxicGases" },
	{ 9,	16,	"flammableLiquids" },
	{ 10,	15,	"flammableSolids" },
	{ 11,	39,	"substancesLiableToSpontaneousCombustion" },
	{ 12,	52,	"substancesEmittingFlammableGasesUponContactWithWater" },
	{ 13,	19,	"oxidizingSubstances" },
	{ 14,	16,	"organicPeroxides" },
	{ 15,	15,	"toxicSubstances" },
	{ 16,	20,	"infectiousSubstances" },
	{ 17,	19,	"radioactiveMaterial" },
	{ 18,	19,	"corrosiveSubstances" },
	{ 19,	32,	"miscellaneousDangerousSubstances" }
};
static const unsigned int asn_MAP_DangerousGoodsBasic_enum2value_1[] = {
	18,	/* corrosiveSubstances(18) */
	0,	/* explosives1(0) */
	1,	/* explosives2(1) */
	2,	/* explosives3(2) */
	3,	/* explosives4(3) */
	4,	/* explosives5(4) */
	5,	/* explosives6(5) */
	6,	/* flammableGases(6) */
	9,	/* flammableLiquids(9) */
	10,	/* flammableSolids(10) */
	16,	/* infectiousSubstances(16) */
	19,	/* miscellaneousDangerousSubstances(19) */
	7,	/* nonFlammableGases(7) */
	14,	/* organicPeroxides(14) */
	13,	/* oxidizingSubstances(13) */
	17,	/* radioactiveMaterial(17) */
	12,	/* substancesEmittingFlammableGasesUponContactWithWater(12) */
	11,	/* substancesLiableToSpontaneousCombustion(11) */
	8,	/* toxicGases(8) */
	15	/* toxicSubstances(15) */
};
const asn_INTEGER_specifics_t asn_SPC_DangerousGoodsBasic_specs_1 = {
	asn_MAP_DangerousGoodsBasic_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_DangerousGoodsBasic_enum2value_1,	/* N => "tag"; sorted by N */
	20,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_DangerousGoodsBasic_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_DangerousGoodsBasic = {
	"DangerousGoodsBasic",
	"DangerousGoodsBasic",
	&asn_OP_NativeEnumerated,
	asn_DEF_DangerousGoodsBasic_tags_1,
	sizeof(asn_DEF_DangerousGoodsBasic_tags_1)
		/sizeof(asn_DEF_DangerousGoodsBasic_tags_1[0]), /* 1 */
	asn_DEF_DangerousGoodsBasic_tags_1,	/* Same as above */
	sizeof(asn_DEF_DangerousGoodsBasic_tags_1)
		/sizeof(asn_DEF_DangerousGoodsBasic_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		&asn_OER_type_DangerousGoodsBasic_constr_1,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_DangerousGoodsBasic_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_DangerousGoodsBasic_specs_1	/* Additional specs */
};

