/*
 * We place these static definitions on a separate header file so that we can
 * include the file in both the library and the tests.
 */

#include <ctype.h>
#include <float.h>
#include <math.h>
#include <memory.h>

#define OLC_kEncodingBase 20
#define OLC_kGridCols 4
#define OLC_kLatMaxDegrees 90
#define OLC_kLonMaxDegrees 180

// Separates the first eight digits from the rest of the code.
static const char kSeparator = '+';
// Used to indicate null values before the separator.
static const char kPaddingCharacter = '0';
// Digits used in the codes.
static const char kAlphabet[] = "23456789CFGHJMPQRVWX";
// Number of digits in the alphabet.
static const size_t kEncodingBase = OLC_kEncodingBase;
// The max number of digits returned in a plus code. Roughly 1 x 0.5 cm.
static const size_t kMaximumDigitCount = 15;
// The number of code characters that are lat/lng pairs.
static const size_t kPairCodeLength = 10;
// The number of characters that combine lat and lng into a grid.
// kMaximumDigitCount - kPairCodeLength
static const size_t kGridCodeLength = 5;
// The number of columns in each grid step.
static const size_t kGridCols = OLC_kGridCols;
// The number of rows in each grid step.
static const size_t kGridRows = OLC_kEncodingBase / OLC_kGridCols;
// The number of digits before the separator.
static const size_t kSeparatorPosition = 8;
// Inverse of the precision of the last pair digits (in degrees).
static const size_t kPairPrecisionInverse = 8000;
// Inverse (1/) of the precision of the final grid digits in degrees.
// Latitude is kEncodingBase^3 * kGridRows^kGridCodeLength
static const size_t kGridLatPrecisionInverse = 2.5e7;
// Longitude is kEncodingBase^3 * kGridColumns^kGridCodeLength
static const size_t kGridLonPrecisionInverse = 8.192e6;
// Latitude bounds are -kLatMaxDegrees degrees and +kLatMaxDegrees degrees
// which we transpose to 0 and 180 degrees.
static const double kLatMaxDegrees = OLC_kLatMaxDegrees;
static const double kLatMaxDegreesT2 = 2 * OLC_kLatMaxDegrees;

// Longitude bounds are -kLonMaxDegrees degrees and +kLonMaxDegrees degrees
// which we transpose to 0 and 360 degrees.
static const double kLonMaxDegrees = OLC_kLonMaxDegrees;
static const double kLonMaxDegreesT2 = 2 * OLC_kLonMaxDegrees;

// Lookup table of the alphabet positions of characters 'C' through 'X',
// inclusive. A value of -1 means the character isn't part of the alphabet.
static const int kPositionLUT['X' - 'C' + 1] = {
    8,  -1, -1, 9,  10, 11, -1, 12, -1, -1, 13,
    -1, -1, 14, 15, 16, -1, -1, -1, 17, 18, 19,
};

// Returns the position of a char in the encoding alphabet, or -1 if invalid.
static int get_alphabet_position(char c) {
  char uc = toupper(c);
  // We use a lookup table for performance reasons.
  if (uc >= 'C' && uc <= 'X') return kPositionLUT[uc - 'C'];
  if (uc >= 'c' && uc <= 'x') return kPositionLUT[uc - 'c'];
  if (uc >= '2' && uc <= '9') return uc - '2';
  return -1;
}
