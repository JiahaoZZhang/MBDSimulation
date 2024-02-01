file(REMOVE_RECURSE
  "../../iso/ISO14816.asn"
  "../../iso/ISO19091.asn"
  "../../iso/ISO24534-3.asn"
  "CMakeFiles/generate_asn1c"
  "asn1c_its_sources.txt"
  "asn1c_pki_sources.txt"
  "asn1c_security_sources.txt"
  "asn1c_support_sources.txt"
  "its"
  "pki"
  "security"
  "support"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/generate_asn1c.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
