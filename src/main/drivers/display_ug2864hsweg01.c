/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/bus_i2c.h"

#include "display_ug2864hsweg01.h"

#ifdef USE_I2C_OLED_DISPLAY

#define INVERSE_CHAR_FORMAT 0x7f // 0b01111111
#define NORMAL_CHAR_FORMAT  0x00 // 0b00000000

unsigned char CHAR_FORMAT = NORMAL_CHAR_FORMAT;

static const uint8_t multiWiiFont[][5] = { // Refer to "Times New Roman" Font Database... 5 x 7 font
        { 0x00, 0x00, 0x00, 0x00, 0x00 }, { 0x00, 0x00, 0x4F, 0x00, 0x00 }, //   (  1)  ! - 0x0021 Exclamation Mark
                { 0x00, 0x07, 0x00, 0x07, 0x00 }, //   (  2)  " - 0x0022 Quotation Mark
                { 0x14, 0x7F, 0x14, 0x7F, 0x14 }, //   (  3)  # - 0x0023 Number Sign
                { 0x24, 0x2A, 0x7F, 0x2A, 0x12 }, //   (  4)  $ - 0x0024 Dollar Sign
                { 0x23, 0x13, 0x08, 0x64, 0x62 }, //   (  5)  % - 0x0025 Percent Sign
                { 0x36, 0x49, 0x55, 0x22, 0x50 }, //   (  6)  & - 0x0026 Ampersand
                { 0x00, 0x05, 0x03, 0x00, 0x00 }, //   (  7)  ' - 0x0027 Apostrophe
                { 0x00, 0x1C, 0x22, 0x41, 0x00 }, //   (  8)  ( - 0x0028 Left Parenthesis
                { 0x00, 0x41, 0x22, 0x1C, 0x00 }, //   (  9)  ) - 0x0029 Right Parenthesis
                { 0x14, 0x08, 0x3E, 0x08, 0x14 }, //   ( 10)  * - 0x002A Asterisk
                { 0x08, 0x08, 0x3E, 0x08, 0x08 }, //   ( 11)  + - 0x002B Plus Sign
                { 0x00, 0x50, 0x30, 0x00, 0x00 }, //   ( 12)  , - 0x002C Comma
                { 0x08, 0x08, 0x08, 0x08, 0x08 }, //   ( 13)  - - 0x002D Hyphen-Minus
                { 0x00, 0x60, 0x60, 0x00, 0x00 }, //   ( 14)  . - 0x002E Full Stop
                { 0x20, 0x10, 0x08, 0x04, 0x02 }, //   ( 15)  / - 0x002F Solidus
                { 0x3E, 0x51, 0x49, 0x45, 0x3E }, //   ( 16)  0 - 0x0030 Digit Zero
                { 0x00, 0x42, 0x7F, 0x40, 0x00 }, //   ( 17)  1 - 0x0031 Digit One
                { 0x42, 0x61, 0x51, 0x49, 0x46 }, //   ( 18)  2 - 0x0032 Digit Two
                { 0x21, 0x41, 0x45, 0x4B, 0x31 }, //   ( 19)  3 - 0x0033 Digit Three
                { 0x18, 0x14, 0x12, 0x7F, 0x10 }, //   ( 20)  4 - 0x0034 Digit Four
                { 0x27, 0x45, 0x45, 0x45, 0x39 }, //   ( 21)  5 - 0x0035 Digit Five
                { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, //   ( 22)  6 - 0x0036 Digit Six
                { 0x01, 0x71, 0x09, 0x05, 0x03 }, //   ( 23)  7 - 0x0037 Digit Seven
                { 0x36, 0x49, 0x49, 0x49, 0x36 }, //   ( 24)  8 - 0x0038 Digit Eight
                { 0x06, 0x49, 0x49, 0x29, 0x1E }, //   ( 25)  9 - 0x0039 Dight Nine
                { 0x00, 0x36, 0x36, 0x00, 0x00 }, //   ( 26)  : - 0x003A Colon
                { 0x00, 0x56, 0x36, 0x00, 0x00 }, //   ( 27)  ; - 0x003B Semicolon
                { 0x08, 0x14, 0x22, 0x41, 0x00 }, //   ( 28)  < - 0x003C Less-Than Sign
                { 0x14, 0x14, 0x14, 0x14, 0x14 }, //   ( 29)  = - 0x003D Equals Sign
                { 0x00, 0x41, 0x22, 0x14, 0x08 }, //   ( 30)  > - 0x003E Greater-Than Sign
                { 0x02, 0x01, 0x51, 0x09, 0x06 }, //   ( 31)  ? - 0x003F Question Mark
                { 0x32, 0x49, 0x79, 0x41, 0x3E }, //   ( 32)  @ - 0x0040 Commercial At
                { 0x7E, 0x11, 0x11, 0x11, 0x7E }, //   ( 33)  A - 0x0041 Latin Capital Letter A
                { 0x7F, 0x49, 0x49, 0x49, 0x36 }, //   ( 34)  B - 0x0042 Latin Capital Letter B
                { 0x3E, 0x41, 0x41, 0x41, 0x22 }, //   ( 35)  C - 0x0043 Latin Capital Letter C
                { 0x7F, 0x41, 0x41, 0x22, 0x1C }, //   ( 36)  D - 0x0044 Latin Capital Letter D
                { 0x7F, 0x49, 0x49, 0x49, 0x41 }, //   ( 37)  E - 0x0045 Latin Capital Letter E
                { 0x7F, 0x09, 0x09, 0x09, 0x01 }, //   ( 38)  F - 0x0046 Latin Capital Letter F
                { 0x3E, 0x41, 0x49, 0x49, 0x7A }, //   ( 39)  G - 0x0047 Latin Capital Letter G
                { 0x7F, 0x08, 0x08, 0x08, 0x7F }, //   ( 40)  H - 0x0048 Latin Capital Letter H
                { 0x00, 0x41, 0x7F, 0x41, 0x00 }, //   ( 41)  I - 0x0049 Latin Capital Letter I
                { 0x20, 0x40, 0x41, 0x3F, 0x01 }, //   ( 42)  J - 0x004A Latin Capital Letter J
                { 0x7F, 0x08, 0x14, 0x22, 0x41 }, //   ( 43)  K - 0x004B Latin Capital Letter K
                { 0x7F, 0x40, 0x40, 0x40, 0x40 }, //   ( 44)  L - 0x004C Latin Capital Letter L
                { 0x7F, 0x02, 0x0C, 0x02, 0x7F }, //   ( 45)  M - 0x004D Latin Capital Letter M
                { 0x7F, 0x04, 0x08, 0x10, 0x7F }, //   ( 46)  N - 0x004E Latin Capital Letter N
                { 0x3E, 0x41, 0x41, 0x41, 0x3E }, //   ( 47)  O - 0x004F Latin Capital Letter O
                { 0x7F, 0x09, 0x09, 0x09, 0x06 }, //   ( 48)  P - 0x0050 Latin Capital Letter P
                { 0x3E, 0x41, 0x51, 0x21, 0x5E }, //   ( 49)  Q - 0x0051 Latin Capital Letter Q
                { 0x7F, 0x09, 0x19, 0x29, 0x46 }, //   ( 50)  R - 0x0052 Latin Capital Letter R
                { 0x46, 0x49, 0x49, 0x49, 0x31 }, //   ( 51)  S - 0x0053 Latin Capital Letter S
                { 0x01, 0x01, 0x7F, 0x01, 0x01 }, //   ( 52)  T - 0x0054 Latin Capital Letter T
                { 0x3F, 0x40, 0x40, 0x40, 0x3F }, //   ( 53)  U - 0x0055 Latin Capital Letter U
                { 0x1F, 0x20, 0x40, 0x20, 0x1F }, //   ( 54)  V - 0x0056 Latin Capital Letter V
                { 0x3F, 0x40, 0x38, 0x40, 0x3F }, //   ( 55)  W - 0x0057 Latin Capital Letter W
                { 0x63, 0x14, 0x08, 0x14, 0x63 }, //   ( 56)  X - 0x0058 Latin Capital Letter X
                { 0x07, 0x08, 0x70, 0x08, 0x07 }, //   ( 57)  Y - 0x0059 Latin Capital Letter Y
                { 0x61, 0x51, 0x49, 0x45, 0x43 }, //   ( 58)  Z - 0x005A Latin Capital Letter Z
                { 0x00, 0x7F, 0x41, 0x41, 0x00 }, //   ( 59)  [ - 0x005B Left Square Bracket
                { 0x02, 0x04, 0x08, 0x10, 0x20 }, //   ( 60)  \ - 0x005C Reverse Solidus
                { 0x00, 0x41, 0x41, 0x7F, 0x00 }, //   ( 61)  ] - 0x005D Right Square Bracket
                { 0x04, 0x02, 0x01, 0x02, 0x04 }, //   ( 62)  ^ - 0x005E Circumflex Accent
                { 0x40, 0x40, 0x40, 0x40, 0x40 }, //   ( 63)  _ - 0x005F Low Line
                { 0x01, 0x02, 0x04, 0x00, 0x00 }, //   ( 64)  ` - 0x0060 Grave Accent
                { 0x20, 0x54, 0x54, 0x54, 0x78 }, //   ( 65)  a - 0x0061 Latin Small Letter A
                { 0x7F, 0x48, 0x44, 0x44, 0x38 }, //   ( 66)  b - 0x0062 Latin Small Letter B
                { 0x38, 0x44, 0x44, 0x44, 0x20 }, //   ( 67)  c - 0x0063 Latin Small Letter C
                { 0x38, 0x44, 0x44, 0x48, 0x7F }, //   ( 68)  d - 0x0064 Latin Small Letter D
                { 0x38, 0x54, 0x54, 0x54, 0x18 }, //   ( 69)  e - 0x0065 Latin Small Letter E
                { 0x08, 0x7E, 0x09, 0x01, 0x02 }, //   ( 70)  f - 0x0066 Latin Small Letter F
                { 0x06, 0x49, 0x49, 0x49, 0x3F }, //   ( 71)  g - 0x0067 Latin Small Letter G
                { 0x7F, 0x08, 0x04, 0x04, 0x78 }, //   ( 72)  h - 0x0068 Latin Small Letter H
                { 0x00, 0x44, 0x7D, 0x40, 0x00 }, //   ( 73)  i - 0x0069 Latin Small Letter I
                { 0x20, 0x40, 0x44, 0x3D, 0x00 }, //   ( 74)  j - 0x006A Latin Small Letter J
                { 0x7F, 0x10, 0x28, 0x44, 0x00 }, //   ( 75)  k - 0x006B Latin Small Letter K
                { 0x00, 0x41, 0x7F, 0x40, 0x00 }, //   ( 76)  l - 0x006C Latin Small Letter L
                { 0x7C, 0x04, 0x18, 0x04, 0x7C }, //   ( 77)  m - 0x006D Latin Small Letter M
                { 0x7C, 0x08, 0x04, 0x04, 0x78 }, //   ( 78)  n - 0x006E Latin Small Letter N
                { 0x38, 0x44, 0x44, 0x44, 0x38 }, //   ( 79)  o - 0x006F Latin Small Letter O
                { 0x7C, 0x14, 0x14, 0x14, 0x08 }, //   ( 80)  p - 0x0070 Latin Small Letter P
                { 0x08, 0x14, 0x14, 0x18, 0x7C }, //   ( 81)  q - 0x0071 Latin Small Letter Q
                { 0x7C, 0x08, 0x04, 0x04, 0x08 }, //   ( 82)  r - 0x0072 Latin Small Letter R
                { 0x48, 0x54, 0x54, 0x54, 0x20 }, //   ( 83)  s - 0x0073 Latin Small Letter S
                { 0x04, 0x3F, 0x44, 0x40, 0x20 }, //   ( 84)  t - 0x0074 Latin Small Letter T
                { 0x3C, 0x40, 0x40, 0x20, 0x7C }, //   ( 85)  u - 0x0075 Latin Small Letter U
                { 0x1C, 0x20, 0x40, 0x20, 0x1C }, //   ( 86)  v - 0x0076 Latin Small Letter V
                { 0x3C, 0x40, 0x30, 0x40, 0x3C }, //   ( 87)  w - 0x0077 Latin Small Letter W
                { 0x44, 0x28, 0x10, 0x28, 0x44 }, //   ( 88)  x - 0x0078 Latin Small Letter X
                { 0x0C, 0x50, 0x50, 0x50, 0x3C }, //   ( 89)  y - 0x0079 Latin Small Letter Y
                { 0x44, 0x64, 0x54, 0x4C, 0x44 }, //   ( 90)  z - 0x007A Latin Small Letter Z
                { 0x00, 0x08, 0x36, 0x41, 0x00 }, //   ( 91)  { - 0x007B Left Curly Bracket
                { 0x00, 0x00, 0x7F, 0x00, 0x00 }, //   ( 92)  | - 0x007C Vertical Line
                { 0x00, 0x41, 0x36, 0x08, 0x00 }, //   ( 93)  } - 0x007D Right Curly Bracket
                { 0x02, 0x01, 0x02, 0x04, 0x02 }, //   ( 94)  ~ - 0x007E Tilde
                { 0x3E, 0x55, 0x55, 0x41, 0x22 }, //   ( 95)  C - 0x0080 <Control>
                { 0x00, 0x00, 0x00, 0x00, 0x00 }, //   ( 96)    - 0x00A0 No-Break Space
                { 0x00, 0x00, 0x79, 0x00, 0x00 }, //   ( 97)  ! - 0x00A1 Inverted Exclamation Mark
                { 0x18, 0x24, 0x74, 0x2E, 0x24 }, //   ( 98)  c - 0x00A2 Cent Sign
                { 0x48, 0x7E, 0x49, 0x42, 0x40 }, //   ( 99)  L - 0x00A3 Pound Sign
                { 0x5D, 0x22, 0x22, 0x22, 0x5D }, //   (100)  o - 0x00A4 Currency Sign
                { 0x15, 0x16, 0x7C, 0x16, 0x15 }, //   (101)  Y - 0x00A5 Yen Sign
                { 0x00, 0x00, 0x77, 0x00, 0x00 }, //   (102)  | - 0x00A6 Broken Bar
                { 0x0A, 0x55, 0x55, 0x55, 0x28 }, //   (103)    - 0x00A7 Section Sign
                { 0x00, 0x01, 0x00, 0x01, 0x00 }, //   (104)  " - 0x00A8 Diaeresis
                { 0x00, 0x0A, 0x0D, 0x0A, 0x04 }, //   (105)    - 0x00AA Feminine Ordinal Indicator
                { 0x08, 0x14, 0x2A, 0x14, 0x22 }, //   (106) << - 0x00AB Left-Pointing Double Angle Quotation Mark
                { 0x04, 0x04, 0x04, 0x04, 0x1C }, //   (107)    - 0x00AC Not Sign
                { 0x00, 0x08, 0x08, 0x08, 0x00 }, //   (108)  - - 0x00AD Soft Hyphen
                { 0x01, 0x01, 0x01, 0x01, 0x01 }, //   (109)    - 0x00AF Macron
                { 0x00, 0x02, 0x05, 0x02, 0x00 }, //   (110)    - 0x00B0 Degree Sign
                { 0x44, 0x44, 0x5F, 0x44, 0x44 }, //   (111) +- - 0x00B1 Plus-Minus Sign
                { 0x00, 0x00, 0x04, 0x02, 0x01 }, //   (112)  ` - 0x00B4 Acute Accent
                { 0x7E, 0x20, 0x20, 0x10, 0x3E }, //   (113)  u - 0x00B5 Micro Sign
                { 0x06, 0x0F, 0x7F, 0x00, 0x7F }, //   (114)    - 0x00B6 Pilcrow Sign
                { 0x00, 0x18, 0x18, 0x00, 0x00 }, //   (115)  . - 0x00B7 Middle Dot
                { 0x00, 0x40, 0x50, 0x20, 0x00 }, //   (116)    - 0x00B8 Cedilla
                { 0x00, 0x0A, 0x0D, 0x0A, 0x00 }, //   (117)    - 0x00BA Masculine Ordinal Indicator
                { 0x22, 0x14, 0x2A, 0x14, 0x08 }, //   (118) >> - 0x00BB Right-Pointing Double Angle Quotation Mark
                { 0x17, 0x08, 0x34, 0x2A, 0x7D }, //   (119) /4 - 0x00BC Vulgar Fraction One Quarter
                { 0x17, 0x08, 0x04, 0x6A, 0x59 }, //   (120) /2 - 0x00BD Vulgar Fraction One Half
                { 0x30, 0x48, 0x45, 0x40, 0x20 }, //   (121)  ? - 0x00BE Inverted Question Mark
                { 0x42, 0x00, 0x42, 0x00, 0x42 }, //   (122)    - 0x00BF Horizontal Bargraph - 0 (empty)
                { 0x7E, 0x42, 0x00, 0x42, 0x00 }, //   (123)    - 0x00C0 Horizontal Bargraph - 1
                { 0x7E, 0x7E, 0x00, 0x42, 0x00 }, //   (124)    - 0x00C1 Horizontal Bargraph - 2
                { 0x7E, 0x7E, 0x7E, 0x42, 0x00 }, //   (125)    - 0x00C2 Horizontal Bargraph - 3
                { 0x7E, 0x7E, 0x7E, 0x7E, 0x00 }, //   (126)    - 0x00C3 Horizontal Bargraph - 4
                { 0x7E, 0x7E, 0x7E, 0x7E, 0x7E }, //   (127)    - 0x00C4 Horizontal Bargraph - 5 (full)
                { 0x5A, 0x00, 0x00, 0x00, 0x5A }, //   (128)    - 0x00C5 Vertical Bargraph - 0 (empty)
                { 0x5A, 0x40, 0x40, 0x40, 0x5A }, //   (129)    - 0x00C6 Vertical Bargraph - 1
                { 0x7A, 0x60, 0x60, 0x60, 0x7A }, //   (130)    - 0x00C7 Vertical Bargraph - 2
                { 0x7A, 0x70, 0x70, 0x70, 0x7A }, //   (131)    - 0x00C8 Vertical Bargraph - 3
                { 0x7A, 0x78, 0x78, 0x78, 0x7A }, //   (131)    - 0x00C8 Vertical Bargraph - 4
                { 0x7A, 0x7C, 0x7C, 0x7C, 0x7A }, //   (131)    - 0x00C8 Vertical Bargraph - 5
                { 0x7A, 0x7E, 0x7E, 0x7E, 0x7A }, //   (131)    - 0x00C8 Vertical Bargraph - 6 (full)
        };

static bool i2c_OLED_send_cmd(const extDevice_t *dev, uint8_t command)
{
    return i2cWrite(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, 0x80, command);
}

static bool i2c_OLED_send_cmdarray(const extDevice_t *dev, const uint8_t *commands, size_t len)
{
    for (size_t i = 0 ; i < len ; i++) {
        if (!i2c_OLED_send_cmd(dev, commands[i])) {
            return false;
        }
    }

    return true;
}

static bool i2c_OLED_send_byte(const extDevice_t *dev, uint8_t val)
{
    return i2cWrite(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, 0x40, val);
}

void i2c_OLED_clear_display_quick(const extDevice_t *dev)
{
    static const uint8_t i2c_OLED_cmd_clear_display_quick[] = {
        0xb0, // set page address to 0
        0x40, // Display start line register to 0
        0,    // Set low col address to 0
        0x10, // Set high col address to 0
    };

    i2c_OLED_send_cmdarray(dev, i2c_OLED_cmd_clear_display_quick, ARRAYLEN(i2c_OLED_cmd_clear_display_quick));

    for (uint16_t i = 0; i < 1024; i++) {      // fill the display's RAM with graphic... 128*64 pixel picture
        i2c_OLED_send_byte(dev, 0x00);  // clear
    }
}

void i2c_OLED_clear_display(const extDevice_t *dev)
{
    static const uint8_t i2c_OLED_cmd_clear_display_pre[] = {
        0xa6, // Set Normal Display
        0xae, // Display OFF
        0x20, // Set Memory Addressing Mode
        0x00, // Set Memory Addressing Mode to Horizontal addressing mode
    };

    i2c_OLED_send_cmdarray(dev, i2c_OLED_cmd_clear_display_pre, ARRAYLEN(i2c_OLED_cmd_clear_display_pre));

    i2c_OLED_clear_display_quick(dev);

    static const uint8_t i2c_OLED_cmd_clear_display_post[] = {
        0x81, // Setup CONTRAST CONTROL, following byte is the contrast Value... always a 2 byte instruction
        200,  // Here you can set the brightness 1 = dull, 255 is very bright
        0xaf, // display on
    };

    i2c_OLED_send_cmdarray(dev, i2c_OLED_cmd_clear_display_post, ARRAYLEN(i2c_OLED_cmd_clear_display_post));
}

void i2c_OLED_set_xy(const extDevice_t *dev, uint8_t col, uint8_t row)
{
    uint8_t i2c_OLED_cmd_set_xy[] = {
        0xb0 + row,                                            //set page address
        0x00 + ((CHARACTER_WIDTH_TOTAL * col) & 0x0f),         //set low col address
        0x10 + (((CHARACTER_WIDTH_TOTAL * col) >> 4) & 0x0f)   //set high col address
    };

    i2c_OLED_send_cmdarray(dev, i2c_OLED_cmd_set_xy, ARRAYLEN(i2c_OLED_cmd_set_xy));
}

void i2c_OLED_set_line(const extDevice_t *dev, uint8_t row)
{
    i2c_OLED_set_xy(dev, 0, row);
}

void i2c_OLED_send_char(const extDevice_t *dev, unsigned char ascii)
{
    unsigned char i;
    uint8_t buffer;
    for (i = 0; i < 5; i++) {
        buffer = multiWiiFont[ascii - 32][i];
        buffer ^= CHAR_FORMAT;  // apply
        i2c_OLED_send_byte(dev, buffer);
    }
    i2c_OLED_send_byte(dev, CHAR_FORMAT);    // the gap
}

void i2c_OLED_send_string(const extDevice_t *dev, const char *string)
{
    // Sends a string of chars until null terminator
    while (*string) {
        i2c_OLED_send_char(dev, *string);
        string++;
    }
}

/**
* according to http://www.adafruit.com/datasheets/UG-2864HSWEG01.pdf Chapter 4.4 Page 15
*/

bool ug2864hsweg01InitI2C(const extDevice_t *dev)
{

    // Set display OFF
    if (!i2c_OLED_send_cmd(dev, 0xAE)) {
        return false;
    }

    static const uint8_t i2c_OLED_cmd_init[] = {
        0xD4, // Set Display Clock Divide Ratio / OSC Frequency
        0x80, // Display Clock Divide Ratio / OSC Frequency
        0xA8, // Set Multiplex Ratio
        0x3F, // Multiplex Ratio for 128x64 (64-1)
        0xD3, // Set Display Offset
        0x00, // Display Offset
        0x40, // Set Display Start Line
        0x8D, // Set Charge Pump
        0x14, // Charge Pump (0x10 External, 0x14 Internal DC/DC)
        0xA1, // Set Segment Re-Map
        0xC8, // Set Com Output Scan Direction
        0xDA, // Set COM Hardware Configuration
        0x12, // COM Hardware Configuration
        0x81, // Set Contrast
        0xCF, // Contrast
        0xD9, // Set Pre-Charge Period
        0xF1, // Set Pre-Charge Period (0x22 External, 0xF1 Internal)
        0xDB, // Set VCOMH Deselect Level
        0x40, // VCOMH Deselect Level
        0xA4, // Set all pixels OFF
        0xA6, // Set display not inverted
        0xAF, // Set display On
    };

    i2c_OLED_send_cmdarray(dev, i2c_OLED_cmd_init, ARRAYLEN(i2c_OLED_cmd_init));

    i2c_OLED_clear_display(dev);

    return true;
}
#endif // USE_I2C_OLED_DISPLAY
