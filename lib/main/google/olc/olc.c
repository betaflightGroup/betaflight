#include "olc.h"
#include <ctype.h>
#include <float.h>
#include <math.h>
#include <memory.h>
#include <stdio.h>
#include "olc_private.h"

#include "betaflight.h"

#define CORRECT_IF_SEPARATOR(var, info)      \
  do {                                       \
    (var) += (info)->sep_first >= 0 ? 1 : 0; \
  } while (0)

// Information about a code, produced by analyse();
typedef struct CodeInfo {
  // Original code.
  const char* code;
  // Total count of characters in the code including padding and separators.
  int size;
  // Count of valid digits (not including padding or separators).
  int len;
  // Index of the first separator in the code.
  int sep_first;
  // Index of the last separator in the code. (If there is only one, same as
  // sep_first.)
  int sep_last;
  // Index of the first padding character in the code.
  int pad_first;
  // Index of the last padding character in the code. (If there is only one,
  // same as pad_first.)
  int pad_last;
} CodeInfo;

// Helper functions
static int analyse(const char* code, size_t size, CodeInfo* info);
static int is_short(CodeInfo* info);
static int is_full(CodeInfo* info);
static int decode(CodeInfo* info, OLC_CodeArea* decoded);
static size_t code_length(CodeInfo* info);

static double pow_neg(double base, double exponent);
static double compute_latitude_precision(int length);
static double normalize_longitude(double lon_degrees);
static double adjust_latitude(double lat_degrees, size_t length);

void OLC_GetCenter(const OLC_CodeArea* area, OLC_LatLon* center) {
  center->lat = area->lo.lat + (area->hi.lat - area->lo.lat) / 2.0;
  if (center->lat > kLatMaxDegrees) {
    center->lat = kLatMaxDegrees;
  }

  center->lon = area->lo.lon + (area->hi.lon - area->lo.lon) / 2.0;
  if (center->lon > kLonMaxDegrees) {
    center->lon = kLonMaxDegrees;
  }
}

size_t OLC_CodeLength(const char* code, size_t size) {
  CodeInfo info;
  analyse(code, size, &info);
  return code_length(&info);
}

int OLC_IsValid(const char* code, size_t size) {
  CodeInfo info;
  return analyse(code, size, &info) > 0;
}

int OLC_IsShort(const char* code, size_t size) {
  CodeInfo info;
  if (analyse(code, size, &info) <= 0) {
    return 0;
  }
  return is_short(&info);
}

int OLC_IsFull(const char* code, size_t size) {
  CodeInfo info;
  if (analyse(code, size, &info) <= 0) {
    return 0;
  }
  return is_full(&info);
}

int OLC_Encode(const OLC_LatLon* location, size_t length, char* code,
               int maxlen) {
  // Limit the maximum number of digits in the code.
  if (length > kMaximumDigitCount) {
    length = kMaximumDigitCount;
  }
  // Adjust latitude and longitude so they fall into positive ranges.
  double latitude = adjust_latitude(location->lat, length);
  double longitude = normalize_longitude(location->lon);

  // Build up the code here, then copy it to the passed pointer.
  char fullcode[] = "12345678901234567";

  // Compute the code.
  // This approach converts each value to an integer after multiplying it by
  // the final precision. This allows us to use only integer operations, so
  // avoiding any accumulation of floating point representation errors.

  // Multiply values by their precision and convert to positive without any
  // floating point operations.
  long long int lat_val = kLatMaxDegrees * 2.5e7;
  long long int lng_val = kLonMaxDegrees * 8.192e6;
  lat_val += latitude * 2.5e7;
  lng_val += longitude * 8.192e6;

  size_t pos = kMaximumDigitCount;
  // Compute the grid part of the code if necessary.
  if (length > kPairCodeLength) {
    for (size_t i = 0; i < kGridCodeLength; i++) {
      int lat_digit = lat_val % kGridRows;
      int lng_digit = lng_val % kGridCols;
      int ndx = lat_digit * kGridCols + lng_digit;
      fullcode[pos--] = kAlphabet[ndx];
      // Note! Integer division.
      lat_val /= kGridRows;
      lng_val /= kGridCols;
    }
  } else {
    lat_val /= pow(kGridRows, kGridCodeLength);
    lng_val /= pow(kGridCols, kGridCodeLength);
  }
  pos = kPairCodeLength;
  // Compute the pair section of the code.
  for (size_t i = 0; i < kPairCodeLength / 2; i++) {
    int lat_ndx = lat_val % kEncodingBase;
    int lng_ndx = lng_val % kEncodingBase;
    fullcode[pos--] = kAlphabet[lng_ndx];
    fullcode[pos--] = kAlphabet[lat_ndx];
    // Note! Integer division.
    lat_val /= kEncodingBase;
    lng_val /= kEncodingBase;
    if (i == 0) {
      fullcode[pos--] = kSeparator;
    }
  }
  // Replace digits with padding if necessary.
  if (length < kSeparatorPosition) {
    for (size_t i = length; i < kSeparatorPosition; i++) {
      fullcode[i] = kPaddingCharacter;
    }
    fullcode[kSeparatorPosition] = kSeparator;
  }
  // Now copy the full code digits into the buffer.
  size_t char_count = length + 1;
  if (kSeparatorPosition + 1 > char_count) {
    char_count = kSeparatorPosition + 1;
  }
  for (size_t i = 0; i < char_count; i++) {
    code[i] = fullcode[i];
  }

  // Terminate the buffer.
  code[char_count] = '\0';

  return char_count;
}

int OLC_EncodeDefault(const OLC_LatLon* location, char* code, int maxlen) {
  return OLC_Encode(location, kPairCodeLength, code, maxlen);
}

int OLC_Decode(const char* code, size_t size, OLC_CodeArea* decoded) {
  CodeInfo info;
  if (analyse(code, size, &info) <= 0) {
    return 0;
  }
  return decode(&info, decoded);
}

int OLC_Shorten(const char* code, size_t size, const OLC_LatLon* reference,
                char* shortened, int maxlen) {
  CodeInfo info;
  if (analyse(code, size, &info) <= 0) {
    return 0;
  }
  if (info.pad_first > 0) {
    return 0;
  }
  if (!is_full(&info)) {
    return 0;
  }

  OLC_CodeArea code_area;
  decode(&info, &code_area);
  OLC_LatLon center;
  OLC_GetCenter(&code_area, &center);

  // Ensure that latitude and longitude are valid.
  double lat = adjust_latitude(reference->lat, info.len);
  double lon = normalize_longitude(reference->lon);

  // How close are the latitude and longitude to the code center.
  double alat = fabs(center.lat - lat);
  double alon = fabs(center.lon - lon);
  double range = alat > alon ? alat : alon;

  // Yes, magic numbers... sob.
  int start = 0;
  const double safety_factor = 0.3;
  const int removal_lengths[3] = {8, 6, 4};
  for (int j = 0; j < sizeof(removal_lengths) / sizeof(removal_lengths[0]);
       ++j) {
    // Check if we're close enough to shorten. The range must be less than
    // 1/2 the resolution to shorten at all, and we want to allow some
    // safety, so use 0.3 instead of 0.5 as a multiplier.
    int removal_length = removal_lengths[j];
    double area_edge =
        compute_latitude_precision(removal_length) * safety_factor;
    if (range < area_edge) {
      start = removal_length;
      break;
    }
  }
  int pos = 0;
  for (int j = start; j < info.size && code[j] != '\0'; ++j) {
    shortened[pos++] = code[j];
  }
  shortened[pos] = '\0';
  return pos;
}

int OLC_RecoverNearest(const char* short_code, size_t size,
                       const OLC_LatLon* reference, char* code, int maxlen) {
  CodeInfo info;
  if (analyse(short_code, size, &info) <= 0) {
    return 0;
  }
  // Check if it is a full code - then we just convert to upper case.
  if (is_full(&info)) {
    OLC_CodeArea code_area;
    decode(&info, &code_area);
    OLC_LatLon center;
    OLC_GetCenter(&code_area, &center);
    return OLC_Encode(&center, code_area.len, code, maxlen);
  }
  if (!is_short(&info)) {
    return 0;
  }
  int len = code_length(&info);

  // Ensure that latitude and longitude are valid.
  double lat = adjust_latitude(reference->lat, len);
  double lon = normalize_longitude(reference->lon);

  // Compute the number of digits we need to recover.
  size_t padding_length = kSeparatorPosition;
  if (info.sep_first >= 0) {
    padding_length -= info.sep_first;
  }

  // The resolution (height and width) of the padded area in degrees.
  double resolution = pow_neg(kEncodingBase, 2.0 - (padding_length / 2.0));

  // Distance from the center to an edge (in degrees).
  double half_res = resolution / 2.0;

  // Use the reference location to pad the supplied short code and decode it.
  OLC_LatLon latlon = {lat, lon};
  char encoded[256];
  OLC_EncodeDefault(&latlon, encoded, 256);

  char new_code[256];
  int pos = 0;
  for (int j = 0; encoded[j] != '\0'; ++j) {
    if (j >= padding_length) {
      break;
    }
    new_code[pos++] = encoded[j];
  }
  for (int j = 0; j < info.size && short_code[j] != '\0'; ++j) {
    new_code[pos++] = short_code[j];
  }
  new_code[pos] = '\0';
  if (analyse(new_code, pos, &info) <= 0) {
    return 0;
  }

  OLC_CodeArea code_area;
  decode(&info, &code_area);
  OLC_LatLon center;
  OLC_GetCenter(&code_area, &center);

  // How many degrees latitude is the code from the reference?
  if (lat + half_res < center.lat &&
      center.lat - resolution > -kLatMaxDegrees) {
    // If the proposed code is more than half a cell north of the reference
    // location, it's too far, and the best match will be one cell south.
    center.lat -= resolution;
  } else if (lat - half_res > center.lat &&
             center.lat + resolution < kLatMaxDegrees) {
    // If the proposed code is more than half a cell south of the reference
    // location, it's too far, and the best match will be one cell north.
    center.lat += resolution;
  }

  // How many degrees longitude is the code from the reference?
  if (lon + half_res < center.lon) {
    center.lon -= resolution;
  } else if (lon - half_res > center.lon) {
    center.lon += resolution;
  }

  return OLC_Encode(&center, len + padding_length, code, maxlen);
}

// private functions

static int analyse(const char* code, size_t size, CodeInfo* info) {
  memset(info, 0, sizeof(CodeInfo));

  // null code is not valid
  if (!code) {
    return 0;
  }
  if (!size) {
    size = strlen(code);
  }

  info->code = code;
  info->size = size < kMaximumDigitCount ? size : kMaximumDigitCount;
  info->sep_first = -1;
  info->sep_last = -1;
  info->pad_first = -1;
  info->pad_last = -1;
  int j = 0;
  for (j = 0; j <= size && code[j] != '\0'; ++j) {
    int ok = 0;

    // if this is a padding character, remember it
    if (!ok && code[j] == kPaddingCharacter) {
      if (info->pad_first < 0) {
        info->pad_first = j;
      }
      info->pad_last = j;
      ok = 1;
    }

    // if this is a separator character, remember it
    if (!ok && code[j] == kSeparator) {
      if (info->sep_first < 0) {
        info->sep_first = j;
      }
      info->sep_last = j;
      ok = 1;
    }

    // only accept characters in the valid character set
    if (!ok && get_alphabet_position(code[j]) >= 0) {
      ok = 1;
    }

    // didn't find anything expected => bail out
    if (!ok) {
      return 0;
    }
  }

  // so far, code only has valid characters -- good
  info->len = j < kMaximumDigitCount ? j : kMaximumDigitCount;

  // Cannot be empty
  if (info->len <= 0) {
    return 0;
  }

  // The separator is required.
  if (info->sep_first < 0) {
    return 0;
  }

  // There can be only one... separator.
  if (info->sep_last > info->sep_first) {
    return 0;
  }

  // separator cannot be the only character
  if (info->len == 1) {
    return 0;
  }

  // Is the separator in an illegal position?
  if (info->sep_first > kSeparatorPosition || (info->sep_first % 2)) {
    return 0;
  }

  // padding cannot be at the initial position
  if (info->pad_first == 0) {
    return 0;
  }

  // We can have an even number of padding characters before the separator,
  // but then it must be the final character.
  if (info->pad_first > 0) {
    // Short codes cannot have padding
    if (info->sep_first < kSeparatorPosition) {
      return 0;
    }

    // The first padding character needs to be in an odd position.
    if (info->pad_first % 2) {
      return 0;
    }

    // With padding, the separator must be the final character
    if (info->sep_last < info->len - 1) {
      return 0;
    }

    // After removing padding characters, we mustn't have anything left.
    if (info->pad_last < info->sep_first - 1) {
      return 0;
    }
  }

  // If there are characters after the separator, make sure there isn't just
  // one of them (not legal).
  if (info->len - info->sep_first - 1 == 1) {
    return 0;
  }

  return info->len;
}

static int is_short(CodeInfo* info) {
  if (info->len <= 0) {
    return 0;
  }

  // if there is a separator, it cannot be beyond the valid position
  if (info->sep_first >= kSeparatorPosition) {
    return 0;
  }

  return 1;
}

// checks that the first character of latitude or longitude is valid
static int valid_first_character(CodeInfo* info, int pos, double kMax) {
  if (info->len <= pos) {
    return 1;
  }

  // Work out what the first character indicates
  size_t firstValue = get_alphabet_position(info->code[pos]);
  firstValue *= kEncodingBase;
  return firstValue < kMax;
}

static int is_full(CodeInfo* info) {
  if (info->len <= 0) {
    return 0;
  }

  // If there are less characters than expected before the separator.
  if (info->sep_first < kSeparatorPosition) {
    return 0;
  }

  // check first latitude character, if any
  if (!valid_first_character(info, 0, kLatMaxDegreesT2)) {
    return 0;
  }

  // check first longitude character, if any
  if (!valid_first_character(info, 1, kLonMaxDegreesT2)) {
    return 0;
  }

  return 1;
}

static int decode(CodeInfo* info, OLC_CodeArea* decoded) {
  // Create a copy of the code, skipping padding and separators.
  char clean_code[256];
  int ci = 0;
  for (size_t i = 0; i < info->len + 1; i++) {
    if (info->code[i] != kPaddingCharacter && info->code[i] != kSeparator) {
      clean_code[ci] = info->code[i];
      ci++;
    }
  }
  clean_code[ci] = '\0';

  // Initialise the values for each section. We work them out as integers and
  // convert them to floats at the end. Using doubles all the way results in
  // multiplying small rounding errors until they become significant.
  int normal_lat = -kLatMaxDegrees * kPairPrecisionInverse;
  int normal_lng = -kLonMaxDegrees * kPairPrecisionInverse;
  int extra_lat = 0;
  int extra_lng = 0;

  // How many digits do we have to process?
  size_t digits = strlen(clean_code) < kPairCodeLength ? strlen(clean_code)
                                                       : kPairCodeLength;
  // Define the place value for the most significant pair.
  int pv = pow(kEncodingBase, kPairCodeLength / 2);
  for (size_t i = 0; i < digits - 1; i += 2) {
    pv /= kEncodingBase;
    normal_lat += get_alphabet_position(clean_code[i]) * pv;
    normal_lng += get_alphabet_position(clean_code[i + 1]) * pv;
  }
  // Convert the place value to a float in degrees.
  double lat_precision = (double)pv / kPairPrecisionInverse;
  double lng_precision = (double)pv / kPairPrecisionInverse;
  // Process any extra precision digits.
  if (strlen(clean_code) > kPairCodeLength) {
    // How many digits do we have to process?
    digits = strlen(clean_code) < kMaximumDigitCount ? strlen(clean_code)
                                                     : kMaximumDigitCount;
    // Initialise the place values for the grid.
    int row_pv = pow(kGridRows, kGridCodeLength);
    int col_pv = pow(kGridCols, kGridCodeLength);
    for (size_t i = kPairCodeLength; i < digits; i++) {
      row_pv /= kGridRows;
      col_pv /= kGridCols;
      int dval = get_alphabet_position(clean_code[i]);
      int row = dval / kGridCols;
      int col = dval % kGridCols;
      extra_lat += row * row_pv;
      extra_lng += col * col_pv;
    }
    // Adjust the precisions from the integer values to degrees.
    lat_precision = (double)row_pv / kGridLatPrecisionInverse;
    lng_precision = (double)col_pv / kGridLonPrecisionInverse;
  }
  // Merge the values from the normal and extra precision parts of the code.
  // Everything is ints so they all need to be cast to floats.
  double lat = (double)normal_lat / kPairPrecisionInverse +
               (double)extra_lat / kGridLatPrecisionInverse;
  double lng = (double)normal_lng / kPairPrecisionInverse +
               (double)extra_lng / kGridLonPrecisionInverse;
  decoded->lo.lat = lat;
  decoded->lo.lon = lng;
  decoded->hi.lat = lat + lat_precision;
  decoded->hi.lon = lng + lng_precision;
  decoded->len = strlen(clean_code);
  return decoded->len;
}

static size_t code_length(CodeInfo* info) {
  int len = info->len;
  if (info->sep_first >= 0) {
    --len;
  }
  if (info->pad_first >= 0) {
    len = info->pad_first;
  }
  return len;
}

// Raises a number to an exponent, handling negative exponents.
static double pow_neg(double base, double exponent) {
  if (exponent == 0) {
    return 1;
  }

  if (exponent > 0) {
    return pow(base, exponent);
  }

  return 1 / pow(base, -exponent);
}

// Compute the latitude precision value for a given code length.  Lengths <= 10
// have the same precision for latitude and longitude, but lengths > 10 have
// different precisions due to the grid method having fewer columns than rows.
static double compute_latitude_precision(int length) {
  // Magic numbers!
  if (length <= kPairCodeLength) {
    return pow_neg(kEncodingBase, floor((length / -2) + 2));
  }

  return pow_neg(kEncodingBase, -3) / pow(kGridRows, length - kPairCodeLength);
}

// Normalize a longitude into the range -180 to 180, not including 180.
static double normalize_longitude(double lon_degrees) {
  while (lon_degrees < -kLonMaxDegrees) {
    lon_degrees += kLonMaxDegreesT2;
  }
  while (lon_degrees >= kLonMaxDegrees) {
    lon_degrees -= kLonMaxDegreesT2;
  }
  return lon_degrees;
}

// Adjusts 90 degree latitude to be lower so that a legal OLC code can be
// generated.
static double adjust_latitude(double lat_degrees, size_t length) {
  if (lat_degrees < -kLatMaxDegrees) {
    lat_degrees = -kLatMaxDegrees;
  }
  if (lat_degrees > kLatMaxDegrees) {
    lat_degrees = kLatMaxDegrees;
  }
  if (lat_degrees < kLatMaxDegrees) {
    return lat_degrees;
  }
  // Subtract half the code precision to get the latitude into the code area.
  double precision = compute_latitude_precision(length);
  return lat_degrees - precision / 2;
}
