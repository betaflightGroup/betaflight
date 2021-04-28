#ifndef OLC_OPENLOCATIONCODE_H_
#define OLC_OPENLOCATIONCODE_H_

#include <stdlib.h>

#define OLC_VERSION_MAJOR 1
#define OLC_VERSION_MINOR 0
#define OLC_VERSION_PATCH 0

// OLC version number: 2.3.4 => 2003004
// Useful for checking against a particular version or above:
//
// #if OLC_VERSION_NUM < OLC_MAKE_VERSION_NUM(1, 0, 2)
// #error UNSUPPORTED OLC VERSION
// #endif
#define OLC_MAKE_VERSION_NUM(major, minor, patch) \
  ((major * 1000 + minor) * 1000 + patch)

// OLC version string: 2.3.4 => "2.3.4"
#define OLC_MAKE_VERSION_STR_IMPL(major, minor, patch) \
  (#major "." #minor "." #patch)
#define OLC_MAKE_VERSION_STR(major, minor, patch) \
  OLC_MAKE_VERSION_STR_IMPL(major, minor, patch)

// Current version, as a number and a string
#define OLC_VERSION_NUM \
  OLC_MAKE_VERSION_NUM(OLC_VERSION_MAJOR, OLC_VERSION_MINOR, OLC_VERSION_PATCH)
#define OLC_VERSION_STR \
  OLC_MAKE_VERSION_STR(OLC_VERSION_MAJOR, OLC_VERSION_MINOR, OLC_VERSION_PATCH)

// A pair of doubles representing latitude / longitude
typedef struct OLC_LatLon {
  double lat;
  double lon;
} OLC_LatLon;

// An area defined by two corners (lo and hi) and a code length
typedef struct OLC_CodeArea {
  OLC_LatLon lo;
  OLC_LatLon hi;
  size_t len;
} OLC_CodeArea;

// Get the center coordinates for an area
void OLC_GetCenter(const OLC_CodeArea* area, OLC_LatLon* center);

// Get the effective length for a code
size_t OLC_CodeLength(const char* code, size_t size);

// Check for the three obviously-named conditions
int OLC_IsValid(const char* code, size_t size);
int OLC_IsShort(const char* code, size_t size);
int OLC_IsFull(const char* code, size_t size);

// Encode location with given code length (indicates precision) into an OLC
// Return the string length of the code
int OLC_Encode(const OLC_LatLon* location, size_t code_length, char* code,
               int maxlen);

// Encode location with default code length into an OLC
// Return the string length of the code
int OLC_EncodeDefault(const OLC_LatLon* location, char* code, int maxlen);

// Decode OLC into the original location
int OLC_Decode(const char* code, size_t size, OLC_CodeArea* decoded);

// Compute a (shorter) OLC for a given code and a reference location
int OLC_Shorten(const char* code, size_t size, const OLC_LatLon* reference,
                char* buf, int maxlen);

// Given shorter OLC and reference location, compute original (full length) OLC
int OLC_RecoverNearest(const char* short_code, size_t size,
                       const OLC_LatLon* reference, char* code, int maxlen);

#endif
