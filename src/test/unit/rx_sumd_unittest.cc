/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

extern "C" {
#include "platform.h"
#include "pg/pg.h"
#include "pg/rx.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "rx/sumd.h"
#include "telemetry/telemetry.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


extern "C" {
    uint8_t batteryCellCount = 3;
    float rcCommand[4] = {0, 0, 0, 0};
    int16_t telemTemperature1 = 0;
    baro_t baro = { .baroTemperature = 50 };
    telemetryConfig_t telemetryConfig_System;

    timeUs_t rxFrameTimeUs(void) { return 0; }
}


bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig, const SerialRXType serialrxProvider)
{
    //TODO: implement
    UNUSED(portConfig);
    UNUSED(serialrxProvider);

    return false;
}

serialPort_t *telemetrySharedPort = NULL;

static uint16_t vbat = 100;
uint16_t getVbat(void)
{
    return vbat;
}

uint32_t microseconds_stub_value = 0;
uint32_t micros(void)
{
    return microseconds_stub_value;
}

uint32_t microsISR(void)
{
    return micros();
}

#define SERIAL_BUFFER_SIZE 256
#define SERIAL_PORT_DUMMY_IDENTIFIER  (serialPortIdentifier_e)0x12

typedef struct serialPortStub_s {
    uint8_t buffer[SERIAL_BUFFER_SIZE];
    int pos = 0;
    int end = 0;
} serialPortStub_t;

static serialPort_t serialTestInstance;
static serialPortConfig_t serialTestInstanceConfig = {
    .identifier = SERIAL_PORT_DUMMY_IDENTIFIER,
    .functionMask = 0
};

static serialReceiveCallbackPtr stub_serialRxCallback;
static serialPortConfig_t *findSerialPortConfig_stub_retval;
static bool openSerial_called = false;
static serialPortStub_t serialWriteStub;
static bool portIsShared = false;

static portMode_e serialExpectedMode = MODE_RX;
static portOptions_e serialExpectedOptions = SERIAL_UNIDIR;


const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
    EXPECT_EQ(function, FUNCTION_RX_SERIAL);
    return findSerialPortConfig_stub_retval;
}

serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr callback,
    void *callbackData,
    uint32_t baudrate,
    portMode_e mode,
    portOptions_e options
)
{
    openSerial_called = true;
    EXPECT_FALSE(NULL == callback);
    EXPECT_TRUE(NULL == callbackData);
    EXPECT_EQ(identifier, SERIAL_PORT_DUMMY_IDENTIFIER);
    EXPECT_EQ(options, serialExpectedOptions);
    EXPECT_EQ(function, FUNCTION_RX_SERIAL);
    EXPECT_EQ(baudrate, 115200);
    EXPECT_EQ(mode, serialExpectedMode);
    stub_serialRxCallback = callback;
    return &serialTestInstance;
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_LT(serialWriteStub.pos, sizeof(serialWriteStub.buffer));
    serialWriteStub.buffer[serialWriteStub.pos++] = ch;
}

void serialTestResetPort()
{
    openSerial_called = false;
    stub_serialRxCallback = NULL;
    portIsShared = false;
    serialExpectedMode = MODE_RX;
    serialExpectedOptions = SERIAL_UNIDIR;
}

class SumdRxInitUnitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        serialTestResetPort();
    }
};


TEST_F(SumdRxInitUnitTest, Test_SumdRxNotEnabled)
{
    const rxConfig_t initialRxConfig = {};
    rxRuntimeState_t rxRuntimeState = {};
    findSerialPortConfig_stub_retval = NULL;

    EXPECT_FALSE(sumdInit(&initialRxConfig, &rxRuntimeState));

    EXPECT_EQ(18, rxRuntimeState.channelCount);
    EXPECT_EQ(11000, rxRuntimeState.rxRefreshRate);
    EXPECT_FALSE(NULL == rxRuntimeState.rcReadRawFn);
    EXPECT_FALSE(NULL == rxRuntimeState.rcFrameStatusFn);
}


TEST_F(SumdRxInitUnitTest, Test_SumdRxEnabled)
{
    const rxConfig_t initialRxConfig = {};
    rxRuntimeState_t rxRuntimeState = {};
    findSerialPortConfig_stub_retval = &serialTestInstanceConfig;

    EXPECT_TRUE(sumdInit(&initialRxConfig, &rxRuntimeState));

    EXPECT_EQ(18, rxRuntimeState.channelCount);
    EXPECT_EQ(11000, rxRuntimeState.rxRefreshRate);
    EXPECT_FALSE(NULL == rxRuntimeState.rcReadRawFn);
    EXPECT_FALSE(NULL == rxRuntimeState.rcFrameStatusFn);

    EXPECT_TRUE(openSerial_called);
}



class SumdRxProtocollUnitTest : public ::testing::Test
{
protected:
    rxRuntimeState_t rxRuntimeState = {};
    virtual void SetUp()
    {
        serialTestResetPort();

        const rxConfig_t initialRxConfig = {};
        findSerialPortConfig_stub_retval = &serialTestInstanceConfig;

        EXPECT_TRUE(sumdInit(&initialRxConfig, &rxRuntimeState));
        microseconds_stub_value += 5000;
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    }

    virtual void checkValidChannels()
    {
        //report frame complete once
        EXPECT_EQ(RX_FRAME_COMPLETE, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

        EXPECT_EQ(900, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 0));
        EXPECT_EQ(1100, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 1));
        EXPECT_EQ(1500, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 2));
        EXPECT_EQ(1900, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 3));
        EXPECT_EQ(2100, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 4));
    }

    /*
     * the structure of a sumd packet is
     * 0xA8 = sync byte, 0x01 = status byte, number of channels
     * Channel data two bytes per channel
     * two bytes checksum
     */
    virtual void sendValidPacket()
    {
        uint8_t packet[] = {0xA8, 0x01, 20,
                            0x1c, 0x20, 0x22, 0x60, 0x2e, 0xe0, 0x3b, 0x60, 0x41, 0xa0,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x06, 0x3f
                           };

        for (size_t i = 0; i < sizeof(packet); i++) {
            EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
            stub_serialRxCallback(packet[i], NULL);
        }
        checkValidChannels();
    }

    virtual void sendValidLongPacket()
    {
        uint8_t packet[] = {0xA8, 0x01, 32,
                            0x1c, 0x20, 0x22, 0x60, 0x2e, 0xe0, 0x3b, 0x60, 0x41, 0xa0,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20,
                            0xeb, 0x61
                           };

        for (size_t i = 0; i < sizeof(packet); i++) {
            EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
            stub_serialRxCallback(packet[i], NULL);
        }
        checkValidChannels();
    }

    virtual void sendValidShortPacket()
    {
        uint8_t packet[] = {0xA8, 0x01, 16,
                            0x1c, 0x20, 0x22, 0x60, 0x2e, 0xe0, 0x3b, 0x60, 0x41, 0xa0,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20,
                            0x37, 0x24
                           };

        for (size_t i = 0; i < sizeof(packet); i++) {
            EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
            stub_serialRxCallback(packet[i], NULL);
        }

        checkValidChannels();
    }

    virtual void sendValidFsPacket()
    {

        uint8_t packet[] = {0xA8, 0x81, 0x08,
                            0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0,
                            0xF9, 0x0F
                           };

        for (size_t i = 0; i < sizeof(packet); i++) {
            EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
            stub_serialRxCallback(packet[i], NULL);
        }

        EXPECT_EQ(RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

        for  (size_t i = 0; i < 8; i++) {
            EXPECT_EQ(1500, rxRuntimeState.rcReadRawFn(&rxRuntimeState, i));
        }
    }

    virtual void sendInvalidPacket()
    {
        uint8_t packet[] = {0xA8, 0x01, 20,
                            0x1c, 0x21, 0x22, 0x60, 0x2e, 0xe0, 0x3b, 0x60, 0x41, 0xa0,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                            0x06, 0x3f
                           };

        for (size_t i = 0; i < sizeof(packet); i++) {
            EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
            stub_serialRxCallback(packet[i], NULL);
        }

        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

        EXPECT_EQ(900, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 0));
        EXPECT_EQ(1100, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 1));
        EXPECT_EQ(1500, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 2));
        EXPECT_EQ(1900, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 3));
        EXPECT_EQ(2100, rxRuntimeState.rcReadRawFn(&rxRuntimeState, 4));
    }

    virtual void sendIncompletePacket()
    {
        uint8_t packet[] = {0xA8, 0x01, 20,
                            0x1c, 0x20, 0x22, 0x60, 0x2e, 0xe0, 0x3b, 0x60, 0x41, 0xa0
                           };

        for (size_t i = 0; i < sizeof(packet); i++) {
            EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
            stub_serialRxCallback(packet[i], NULL);
        }

        microseconds_stub_value += 5000;

        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    }
};


TEST_F(SumdRxProtocollUnitTest, Test_OnePacketReceived)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    sendValidPacket();
}

TEST_F(SumdRxProtocollUnitTest, Test_MultiplePacketsReceived)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    sendValidPacket();
    sendValidPacket();
}

TEST_F(SumdRxProtocollUnitTest, Test_Resync)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    sendIncompletePacket();
    sendValidPacket();
}

TEST_F(SumdRxProtocollUnitTest, Test_IgnoreInvalidCRC)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    sendValidPacket();
    sendInvalidPacket();
}

TEST_F(SumdRxProtocollUnitTest, Test_ShortPacketsReceived)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    sendValidShortPacket();
    sendValidShortPacket();
}

TEST_F(SumdRxProtocollUnitTest, TestShortPacketsReceivedWithFailsafe)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    sendValidFsPacket();
}

TEST_F(SumdRxProtocollUnitTest, Test_LongPacketsReceived)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;
    
    sendValidLongPacket();
    sendValidLongPacket();
}
