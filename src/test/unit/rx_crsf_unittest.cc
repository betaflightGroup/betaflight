/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>
#include <algorithm>

extern "C" {
    #include <platform.h>

    #include "build/debug.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "common/crc.h"
    #include "common/utils.h"

    #include "drivers/serial.h"
    #include "io/serial.h"

    #include "rx/rx.h"
    #include "rx/crsf.h"

    #include "telemetry/msp_shared.h"

    rssiSource_e rssiSource;

    void crsfDataReceive(uint16_t c);
    uint8_t crsfFrameCRC(void);
    uint8_t crsfFrameCmdCRC(void);
    uint8_t crsfFrameStatus(rxRuntimeState_t *rxRuntimeState);
    float crsfReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan);

    rxRuntimeState_t rxRuntimeState = {};

    extern bool crsfFrameDone;
    extern crsfFrame_t crsfFrame;
    extern crsfFrame_t crsfChannelDataFrame;

    uint32_t dummyTimeUs;

    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
const unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crc8_buf(const uint8_t * ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}

uint8_t crc8_dvb_s2_buf(const uint8_t * ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++) {
        crc = crc8_dvb_s2(crc, *ptr++);
    }
    return crc;
}

TEST(CrossFireTest, CRC)
{
    static const uint8_t buf1[] ="abcdefghijklmnopqrstuvwxyz";

    uint8_t crc1 = 0;
    uint8_t crc2 = 0;

    crc1 = crc8tab[1];
    crc2 = crc8_dvb_s2(0, 1);
    EXPECT_EQ(crc1, crc2);

    crc1 = crc8tab[2];
    crc2 = crc8_dvb_s2(0, 2);
    EXPECT_EQ(crc1, crc2);

    crc1 = crc8_buf(buf1, 26);
    crc2 = crc8_dvb_s2_buf(buf1, 26);
    EXPECT_EQ(crc1, crc2);
}

TEST(CrossFireTest, TestCrsfFrameStatus)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    crsfFrameDone = true;
    crsfFrame.frame.deviceAddress = CRSF_ADDRESS_CRSF_RECEIVER;
    crsfFrame.frame.frameLength = 0;
    crsfFrame.frame.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    memset(crsfFrame.frame.payload, 0, CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);
    const uint8_t crc = crsfFrameCRC();
    crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE] = crc;

    const uint8_t status = crsfFrameStatus(&rxRuntimeState);
    EXPECT_EQ(RX_FRAME_COMPLETE, status);
    EXPECT_FALSE(crsfFrameDone);

    EXPECT_EQ(CRSF_ADDRESS_CRSF_RECEIVER, crsfFrame.frame.deviceAddress);
    EXPECT_EQ(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, crsfFrame.frame.type);
    for (int ii = 0; ii < CRSF_MAX_CHANNEL; ++ii) {
        EXPECT_EQ(0, rxRuntimeState.channelXData[ii]);
    }
}

const uint8_t buadrateNegotiationFrame[] = {
    0xC8,0x0C,0x32,0xC8,0xEC,0x0A,0x70,0x01,0x00,0x1E,0x84,0x80,0x22,0x72
};

TEST(CrossFireTest, TestCrsfCmdFrameCrc)
{
    crsfFrame = *(const crsfFrame_t*)buadrateNegotiationFrame;
    crsfFrameDone = true;
    const uint8_t crsfCmdFrameCrc = crsfFrameCmdCRC();
    const uint8_t crsfFrameCrc = crsfFrameCRC();
    EXPECT_EQ(crsfCmdFrameCrc, crsfFrame.frame.payload[crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_ADDRESS - CRSF_FRAME_LENGTH_FRAMELENGTH - 1]);
    EXPECT_EQ(crsfFrameCrc, crsfFrame.frame.payload[crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_ADDRESS - CRSF_FRAME_LENGTH_FRAMELENGTH]);
}

/*
 * Frame is of form
 * <Device address> <Frame length> < Type>  <Payload> < CRC>
 * So RC channels frame is:
 * <0x00> <0x18> <0x16>  <22-bytes payload> < CRC>
 * 26 bytes altogther.
 */
TEST(CrossFireTest, TestCrsfFrameStatusUnpacking)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    crsfFrameDone = true;
    crsfFrame.frame.deviceAddress = CRSF_ADDRESS_CRSF_RECEIVER;
    crsfFrame.frame.frameLength = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;;
    crsfFrame.frame.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    // 16 11-bit channels packed into 22 bytes of data
    crsfFrame.frame.payload[0]  = 0xFF; // bits 0-7
    crsfFrame.frame.payload[1]  = 0xFF; // bits 8-15
    crsfFrame.frame.payload[2]  = 0x00; // bits 16-23
    crsfFrame.frame.payload[3]  = 0x00; // bits 24-31
    crsfFrame.frame.payload[4]  = 0x58; // bits 32-39  0101100.
    crsfFrame.frame.payload[5]  = 0x01; // bits 40-47  ....0001
    crsfFrame.frame.payload[6]  = 0x00; // bits 48-55  0.......
    crsfFrame.frame.payload[7]  = 0xf0; // bits 56-64  11110000
    crsfFrame.frame.payload[8]  = 0x01; // bits 65-71  ......01
    crsfFrame.frame.payload[9]  = 0x60; // bits 72-79  011.....
    crsfFrame.frame.payload[10] = 0xe2; // bits 80-87  11100010
    crsfFrame.frame.payload[11] = 0;
    crsfFrame.frame.payload[12] = 0;
    crsfFrame.frame.payload[13] = 0;
    crsfFrame.frame.payload[14] = 0;
    crsfFrame.frame.payload[15] = 0;
    crsfFrame.frame.payload[16] = 0;
    crsfFrame.frame.payload[17] = 0;
    crsfFrame.frame.payload[18] = 0;
    crsfFrame.frame.payload[19] = 0;
    crsfFrame.frame.payload[20] = 0;
    crsfFrame.frame.payload[21] = 0;
    const uint8_t crc = crsfFrameCRC();
    crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE] = crc;

    memcpy(&crsfChannelDataFrame, &crsfFrame, sizeof(crsfFrame));
    const uint8_t status = crsfFrameStatus(&rxRuntimeState);
    EXPECT_EQ(RX_FRAME_COMPLETE, status);
    EXPECT_FALSE(crsfFrameDone);

    EXPECT_EQ(CRSF_ADDRESS_CRSF_RECEIVER, crsfFrame.frame.deviceAddress);
    EXPECT_EQ(CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC, crsfFrame.frame.frameLength);
    EXPECT_EQ(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, crsfFrame.frame.type);
    EXPECT_EQ(0x7ff, rxRuntimeState.channelXData[0]);
    EXPECT_EQ(0x1f, rxRuntimeState.channelXData[1]);
    EXPECT_EQ(0, rxRuntimeState.channelXData[2]);
    EXPECT_EQ(172, rxRuntimeState.channelXData[3]);  //  172 = 0x0ac, 0001 0101100, bits 33-43
    EXPECT_EQ(0, rxRuntimeState.channelXData[4]);
    EXPECT_EQ(992, rxRuntimeState.channelXData[5]);  //  992 = 0x3e0, 01 1110000 0, bits 55-65
    EXPECT_EQ(0, rxRuntimeState.channelXData[6]);
    EXPECT_EQ(1811, rxRuntimeState.channelXData[7]); // 1811 = 0x713, 1110 0010 011, bits 77-87
    EXPECT_EQ(0, rxRuntimeState.channelXData[8]);
    EXPECT_EQ(0, rxRuntimeState.channelXData[9]);
    EXPECT_EQ(0, rxRuntimeState.channelXData[10]);
    EXPECT_EQ(0, rxRuntimeState.channelXData[11]);
    EXPECT_EQ(0, rxRuntimeState.channelXData[12]);
    EXPECT_EQ(0, rxRuntimeState.channelXData[13]);
    EXPECT_EQ(0, rxRuntimeState.channelXData[14]);
    EXPECT_EQ(0, rxRuntimeState.channelXData[15]);
}

// example of 0x16 RC frame
const uint8_t capturedData[] = {
    0x00,0x18,0x16,0xBD,0x08,0x9F,0xF4,0xAE,0xF7,0xBD,0xEF,0x7D,0xEF,0xFB,0xAD,0xFD,0x45,0x2B,0x5A,0x01,0x00,0x00,0x00,0x00,0x00,0x6C,
    0x00,0x18,0x16,0xBD,0x08,0x9F,0xF4,0xAA,0xF7,0xBD,0xEF,0x7D,0xEF,0xFB,0xAD,0xFD,0x45,0x2B,0x5A,0x01,0x00,0x00,0x00,0x00,0x00,0x94,
};

typedef struct crsfRcChannelsFrame_s {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_CRC];
} crsfRcChannelsFrame_t;


TEST(CrossFireTest, TestCapturedData)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    //const int frameCount = sizeof(capturedData) / sizeof(crsfRcChannelsFrame_t);
    const crsfRcChannelsFrame_t *framePtr = (const crsfRcChannelsFrame_t*)capturedData;
    crsfFrame = *(const crsfFrame_t*)framePtr;
    crsfFrameDone = true;
    memcpy(&crsfChannelDataFrame, &crsfFrame, sizeof(crsfFrame));
    uint8_t status = crsfFrameStatus(&rxRuntimeState);
    EXPECT_EQ(RX_FRAME_COMPLETE, status);
    EXPECT_FALSE(crsfFrameDone);
    EXPECT_EQ(RX_FRAME_COMPLETE, status);
    EXPECT_FALSE(crsfFrameDone);
    EXPECT_EQ(CRSF_ADDRESS_BROADCAST, crsfFrame.frame.deviceAddress);
    EXPECT_EQ(CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC, crsfFrame.frame.frameLength);
    EXPECT_EQ(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, crsfFrame.frame.type);
    EXPECT_EQ(189, rxRuntimeState.channelXData[0]);
    EXPECT_EQ(993, rxRuntimeState.channelXData[1]);
    EXPECT_EQ(978, rxRuntimeState.channelXData[2]);
    EXPECT_EQ(983, rxRuntimeState.channelXData[3]);
    uint8_t crc = crsfFrameCRC();
    EXPECT_EQ(crc, crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]);
    EXPECT_EQ(999, (uint16_t)crsfReadRawRC(&rxRuntimeState, 0));
    EXPECT_EQ(1501, (uint16_t)crsfReadRawRC(&rxRuntimeState, 1));
    EXPECT_EQ(1492, (uint16_t)crsfReadRawRC(&rxRuntimeState, 2));
    EXPECT_EQ(1495, (uint16_t)crsfReadRawRC(&rxRuntimeState, 3));

    ++framePtr;
    crsfFrame = *(const crsfFrame_t*)framePtr;
    crsfFrameDone = true;
    memcpy(&crsfChannelDataFrame, &crsfFrame, sizeof(crsfFrame));
    status = crsfFrameStatus(&rxRuntimeState);
    EXPECT_EQ(RX_FRAME_COMPLETE, status);
    EXPECT_FALSE(crsfFrameDone);
    EXPECT_EQ(RX_FRAME_COMPLETE, status);
    EXPECT_FALSE(crsfFrameDone);
    EXPECT_EQ(CRSF_ADDRESS_BROADCAST, crsfFrame.frame.deviceAddress);
    EXPECT_EQ(CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC, crsfFrame.frame.frameLength);
    EXPECT_EQ(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, crsfFrame.frame.type);
    EXPECT_EQ(189, rxRuntimeState.channelXData[0]);
    EXPECT_EQ(993, rxRuntimeState.channelXData[1]);
    EXPECT_EQ(978, rxRuntimeState.channelXData[2]);
    EXPECT_EQ(981, rxRuntimeState.channelXData[3]);
    crc = crsfFrameCRC();
    EXPECT_EQ(crc, crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]);
}

// example of 0x17 Subset RC frame
/* Notes of Frame Contents
*    frame type = 0x17 Subset RC Frame
*    first channel packed = 4, bits 0-4
*    channel resolution = 0x01 = 11-bit, bits 5-6
*    reserved configuration = 0, bit 7
*    first channel packed  (Ch4) = 0    = 0x000, 000 0000 0000, bits 8 - 18
*    second channel packed (Ch5) = 820  = 0x334, 011 0011 0100, bits 19 - 29
*    third channel packed  (Ch6) = 1959 = 0x7A7, 111 1010 0111, bits 30 - 40
*    fourth channel packed (Ch7) = 2047 = 0x7FF, 111 1111 1111, bits 41 - 51
*/
const uint8_t capturedSubsetData[] = {
    0xC8,0x09,0x17,0x24,0x00,0xA0,0xD9,0xE9,0xFF,0x0F,0xD1
};

TEST(CrossFireTest, TestCapturedSubsetData)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    crsfFrame = *(const crsfFrame_t*)capturedSubsetData;
    crsfFrameDone = true;
    memcpy(&crsfChannelDataFrame, &crsfFrame, sizeof(crsfFrame));

    uint8_t status = crsfFrameStatus(&rxRuntimeState);
    EXPECT_EQ(RX_FRAME_COMPLETE, status);
    EXPECT_FALSE(crsfFrameDone);
    EXPECT_EQ(CRSF_SYNC_BYTE, crsfFrame.frame.deviceAddress);
    EXPECT_EQ(CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED, crsfFrame.frame.type);

    uint8_t crc = crsfFrameCRC();
    uint8_t startChannel = crsfFrame.frame.payload[0] & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
    uint8_t channelRes = (crsfFrame.frame.payload[0] >> CRSF_SUBSET_RC_STARTING_CHANNEL_BITS) & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK;
    uint8_t reservedBit = (crsfFrame.frame.payload[0] >> (CRSF_SUBSET_RC_STARTING_CHANNEL_BITS + CRSF_SUBSET_RC_RES_CONFIGURATION_MASK)) & CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS;
    EXPECT_EQ(crc, crsfFrame.frame.payload[crsfFrame.frame.frameLength - 2]);
    EXPECT_EQ(4, startChannel);
    EXPECT_EQ(1, channelRes);
    EXPECT_EQ(0, reservedBit);

    EXPECT_EQ(0, rxRuntimeState.channelXData[4]);
    EXPECT_EQ(820, rxRuntimeState.channelXData[5]);
    EXPECT_EQ(1959, rxRuntimeState.channelXData[6]);
    EXPECT_EQ(2047, rxRuntimeState.channelXData[7]);

    EXPECT_EQ(988, (uint16_t)crsfReadRawRC(&rxRuntimeState, 4));
    EXPECT_EQ(1398, (uint16_t)crsfReadRawRC(&rxRuntimeState, 5));
    EXPECT_EQ(1967, (uint16_t)crsfReadRawRC(&rxRuntimeState, 6));
    EXPECT_EQ(2011, (uint16_t)crsfReadRawRC(&rxRuntimeState, 7));
}

TEST(CrossFireTest, TestCrsfDataReceive)
{
    crsfFrameDone = false;
    const uint8_t *pData = capturedData;
    for (unsigned int ii = 0; ii < sizeof(crsfRcChannelsFrame_t); ++ii) {
        crsfDataReceive(*pData++);
    }
    EXPECT_FALSE(crsfFrameDone); // data is not a valid rc channels frame so don't expect crsfFrameDone to be true
    EXPECT_EQ(CRSF_ADDRESS_BROADCAST, crsfFrame.frame.deviceAddress);
    EXPECT_EQ(CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC, crsfFrame.frame.frameLength);
    EXPECT_EQ(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, crsfFrame.frame.type);
    uint8_t crc = crsfFrameCRC();
    for (int ii = 0; ii < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE; ++ii) {
        EXPECT_EQ(capturedData[ii + 3], crsfFrame.frame.payload[ii]);
    }
    EXPECT_EQ(crc, crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]);
}

// STUBS

extern "C" {

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint32_t micros(void) { return dummyTimeUs; }
    uint32_t microsISR(void) {return micros();}
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
    const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e ) {return NULL;}
    bool telemetryCheckRxPortShared(const serialPortConfig_t *) {return false;}
    serialPort_t *telemetrySharedPort = NULL;
    void crsfScheduleDeviceInfoResponse(void) {};
    void crsfScheduleMspResponse(void) {};
    bool bufferMspFrame(uint8_t *, int) {return true;}
    bool isBatteryVoltageAvailable(void) { return true; }
    bool isAmperageAvailable(void) { return true; }
    timeUs_t rxFrameTimeUs(void) { return 0; }

}
