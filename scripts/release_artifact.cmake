# Copies a RELEASE build's UF2 to <OUT_DIR>/DSPi-<tag>-<VARIANT>.uf2 and writes
# a matching .sha256 (sha256sum format).  The tag is read back from the
# build_info.h that was compiled in, so the file name can never disagree with
# what the image reports through REQ_GET_BUILD_INFO / picotool info.
# gen_build_info.cmake has already refused the build unless that stamp is an
# exact, clean tag.  See Documentation/RELEASING.md.
#
# Expected -D arguments: BUILD_INFO, UF2, OUT_DIR, VARIANT

file(STRINGS "${BUILD_INFO}" _line REGEX "^#define BUILD_GIT_DESCRIBE ")
string(REGEX REPLACE "^#define BUILD_GIT_DESCRIBE \"(.*)\"$" "\\1" _tag "${_line}")
if(_tag STREQUAL "" OR _tag MATCHES "-dirty$" OR _tag MATCHES "-g[0-9a-f]+$")
    message(FATAL_ERROR "release_artifact: '${_tag}' is not a release stamp")
endif()
if(NOT EXISTS "${UF2}")
    message(FATAL_ERROR "release_artifact: ${UF2} missing")
endif()

set(_name "DSPi-${_tag}-${VARIANT}.uf2")
file(MAKE_DIRECTORY "${OUT_DIR}")
configure_file("${UF2}" "${OUT_DIR}/${_name}" COPYONLY)
file(SHA256 "${OUT_DIR}/${_name}" _sum)
file(WRITE "${OUT_DIR}/${_name}.sha256" "${_sum}  ${_name}\n")
message(STATUS "Release image: ${OUT_DIR}/${_name}")
message(STATUS "sha256: ${_sum}")
