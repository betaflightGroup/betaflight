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

extern "C" {
#include "platform.h"
#include "pg/pg.h"
#include "pg/rx.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "rx/ibus.h"
#include "telemetry/ibus_shared.h"
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


bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig)
{
    //TODO: implement
    (void) portConfig;
    return false;
}

serialPort_t * telemetrySharedPort = NULL;

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

bool isSerialPortShared(const serialPortConfig_t *portConfig,
                        uint16_t functionMask,
                        serialPortFunction_e sharedWithFunction)
{
    EXPECT_EQ(portConfig, findSerialPortConfig_stub_retval);
    EXPECT_EQ(functionMask, FUNCTION_RX_SERIAL);
    EXPECT_EQ(sharedWithFunction, FUNCTION_TELEMETRY_IBUS);
    return portIsShared;
}

const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
    EXPECT_EQ(function, FUNCTION_RX_SERIAL);
    return findSerialPortConfig_stub_retval;
}

static portMode_e serialExpectedMode = MODE_RX;
static portOptions_e serialExpectedOptions = SERIAL_UNIDIR;

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
    //TODO serialReadStub.buffer[serialReadStub.end++] = ch; //characters echoes back on the shared wire
    //printf("w: %02d 0x%02x\n", serialWriteStub.pos, ch);
}


void serialTestResetPort()
{
    openSerial_called = false;
    stub_serialRxCallback = NULL;
    portIsShared = false;
    serialExpectedMode = MODE_RX;
    serialExpectedOptions = SERIAL_UNIDIR;
}


static bool isChecksumOkReturnValue = true;
bool isChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length)
{
    (void) ibusPacket;
    (void) length;
    return isChecksumOkReturnValue;
}


static bool initSharedIbusTelemetryCalled = false;
void initSharedIbusTelemetry(serialPort_t * port)
{
    EXPECT_EQ(port, &serialTestInstance);
    initSharedIbusTelemetryCalled = true;
}

static bool    stubTelemetryCalled = false;
static uint8_t stubTelemetryPacket[100];
static uint8_t stubTelemetryIgnoreRxChars = 0;

uint8_t respondToIbusRequest(uint8_t const * const ibusPacket) {
    uint8_t len = ibusPacket[0];
    EXPECT_LT(len, sizeof(stubTelemetryPacket));
    memcpy(stubTelemetryPacket, ibusPacket, len);
    stubTelemetryCalled = true;
    return stubTelemetryIgnoreRxChars;
}

void resetStubTelemetry(void)
{
    memset(stubTelemetryPacket, 0, sizeof(stubTelemetryPacket));
    stubTelemetryCalled = false;
    stubTelemetryIgnoreRxChars = 0;
    initSharedIbusTelemetryCalled = false;
    isChecksumOkReturnValue = true;
}


class IbusRxInitUnitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        serialTestResetPort();
    }
};


TEST_F(IbusRxInitUnitTest, Test_IbusRxNotEnabled)
{
    const rxConfig_t initialRxConfig = {};
    rxRuntimeState_t rxRuntimeState = {};
    findSerialPortConfig_stub_retval = NULL;

    EXPECT_FALSE(ibusInit(&initialRxConfig, &rxRuntimeState));

    //TODO: Question: I'd expect that runtime conf was not initialized unless there was a serial port to run but the implementation states otherwise
    // EXPECT_EQ(0, rxRuntimeState.channelCount);
    // EXPECT_EQ(0, rxRuntimeState.rxRefreshRate);
    // EXPECT_EQ(NULL, rxRuntimeState.rcReadRawFn);
    // EXPECT_EQ(NULL, rxRuntimeState.rcFrameStatusFn);

    EXPECT_EQ(18, rxRuntimeState.channelCount);
    EXPECT_EQ(20000, rxRuntimeState.rxRefreshRate);
    EXPECT_FALSE(NULL == rxRuntimeState.rcReadRawFn);
    EXPECT_FALSE(NULL == rxRuntimeState.rcFrameStatusFn);
}


TEST_F(IbusRxInitUnitTest, Test_IbusRxEnabled)
{
    const rxConfig_t initialRxConfig = {};
    rxRuntimeState_t rxRuntimeState = {};
    findSerialPortConfig_stub_retval = &serialTestInstanceConfig;

    EXPECT_TRUE(ibusInit(&initialRxConfig, &rxRuntimeState));

    EXPECT_EQ(18, rxRuntimeState.channelCount);
    EXPECT_EQ(20000, rxRuntimeState.rxRefreshRate);
    EXPECT_FALSE(NULL == rxRuntimeState.rcReadRawFn);
    EXPECT_FALSE(NULL == rxRuntimeState.rcFrameStatusFn);

    EXPECT_TRUE(openSerial_called);
}



class IbusRxProtocollUnitTest : public ::testing::Test
{
protected:
    rxRuntimeState_t rxRuntimeState = {};
    virtual void SetUp()
    {
        serialTestResetPort();
        resetStubTelemetry();
        portIsShared = true;
        serialExpectedOptions = SERIAL_BIDIR;
        serialExpectedMode = MODE_RXTX;

        const rxConfig_t initialRxConfig = {};
        findSerialPortConfig_stub_retval = &serialTestInstanceConfig;

        EXPECT_TRUE(ibusInit(&initialRxConfig, &rxRuntimeState));

        EXPECT_TRUE(initSharedIbusTelemetryCalled);

        //handle that internal ibus position is not set to zero at init
        microseconds_stub_value += 5000;
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    }

    virtual void receivePacket(uint8_t const * const packet, const size_t length)
    {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        for (size_t i=0; i < length; i++) {
            EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
            stub_serialRxCallback(packet[i], NULL);
        }
    }
};


TEST_F(IbusRxProtocollUnitTest, Test_InitialFrameState)
{
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

    //TODO: is it ok to have undefined channel values after init?
}


TEST_F(IbusRxProtocollUnitTest, Test_IA6B_OnePacketReceived)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    uint8_t packet[] = {0x20, 0x00, //length and reserved (unknown) bits
                        0x00, 0xE0, 0x01, 0x00, 0x02, 0x00, 0x03, 0xF0, 0x04, 0x00, //channel 1..5  + 15 + 16
                        0x05, 0x00, 0x06, 0x00, 0x07, 0x10, 0x08, 0x00, 0x09, 0x10, //channel 6..10 + 17 + 18(lsb)
                        0x0a, 0x10, 0x0b, 0x00, 0x0c, 0x00, 0x0d, 0x00,             //channel 11..14 + 18
                        0x84, 0xfd}; //checksum

    for (size_t i=0; i < sizeof(packet); i++) {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        stub_serialRxCallback(packet[i], NULL);
    }

    //report frame complete once
    EXPECT_EQ(RX_FRAME_COMPLETE, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

    //check that channel values have been updated
    for (int i=0; i<18; i++) {
        EXPECT_EQ(i, rxRuntimeState.rcReadRawFn(&rxRuntimeState, i));
    }
}


TEST_F(IbusRxProtocollUnitTest, Test_IA6B_OnePacketReceivedWithBadCrc)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    uint8_t packet[] = {0x20, 0x00, //length and reserved (unknown) bits
                        0x00, 0x33, 0x01, 0x33, 0x02, 0x33, 0x03, 0x33, 0x04, 0x33, //channel 1..5
                        0x05, 0x33, 0x06, 0x33, 0x07, 0x33, 0x08, 0x33, 0x09, 0x33, //channel 6..10
                        0x0a, 0x33, 0x0b, 0x33, 0x0c, 0x33, 0x0d, 0x33,             //channel 11..14
                        0x00, 0x00}; //checksum

    isChecksumOkReturnValue = false;
    for (size_t i=0; i < sizeof(packet); i++) {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        stub_serialRxCallback(packet[i], NULL);
    }

    //no frame complete
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

    //check that channel values have not been updated
    for (int i=0; i<14; i++) {
        EXPECT_NE(i + (0x33 << 8), rxRuntimeState.rcReadRawFn(&rxRuntimeState, i));
    }
}


TEST_F(IbusRxProtocollUnitTest, Test_IA6B_HalfPacketReceived_then_interPacketGapReset)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    const uint8_t packet_half[] = {0x20, 0x00, //length and reserved (unknown) bits
                                    0x00, 0xab, 0x01, 0xab, 0x02, 0xab, 0x03, 0xab, 0x04, 0xab, //channel 1..5
                                    0x05, 0xab};
    const uint8_t packet_full[] = {0x20, 0x00, //length and reserved (unknown) bits
                                    0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, //channel 1..5
                                    0x05, 0x00, 0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0x09, 0x00, //channel 6..10
                                    0x0a, 0x00, 0x0b, 0x00, 0x0c, 0x00, 0x0d, 0x00,             //channel 11..14
                                    0x84, 0xff}; //checksum

    for (size_t i=0; i < sizeof(packet_half); i++) {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        stub_serialRxCallback(packet_half[i], NULL);
    }

    microseconds_stub_value += 5000;
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

    for (size_t i=0; i < sizeof(packet_full); i++) {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        stub_serialRxCallback(packet_full[i], NULL);
    }

    //report frame complete once
    EXPECT_EQ(RX_FRAME_COMPLETE, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

    //check that channel values have been updated
    for (int i=0; i<14; i++) {
        EXPECT_EQ(i, rxRuntimeState.rcReadRawFn(&rxRuntimeState, i));
    }
}


TEST_F(IbusRxProtocollUnitTest, Test_IA6_OnePacketReceived)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    uint8_t packet[] = {0x55, //sync character
                        0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, //channel 1..5
                        0x05, 0x00, 0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0x09, 0x00, //channel 6..10
                        0x0a, 0x00, 0x0b, 0x00, 0x0c, 0x00, 0x0d, 0x00,             //channel 11..14
                        0x5b, 0x00}; //checksum

    for (size_t i=0; i < sizeof(packet); i++) {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        stub_serialRxCallback(packet[i], NULL);
    }

    //report frame complete once
    EXPECT_EQ(RX_FRAME_COMPLETE, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

    //check that channel values have been updated
    for (int i=0; i<14; i++) {
        EXPECT_EQ(i, rxRuntimeState.rcReadRawFn(&rxRuntimeState, i));
    }
}


TEST_F(IbusRxProtocollUnitTest, Test_IA6_OnePacketReceivedBadCrc)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;

    uint8_t packet[] = {0x55, //sync character
                        0x00, 0x33, 0x01, 0x33, 0x02, 0x33, 0x03, 0x33, 0x04, 0x33, //channel 1..5
                        0x05, 0x33, 0x06, 0x33, 0x07, 0x33, 0x08, 0x33, 0x09, 0x33, //channel 6..10
                        0x0a, 0x33, 0x0b, 0x33, 0x0c, 0x33, 0x0d, 0x33,             //channel 11..14
                        0x00, 0x00}; //checksum

    for (size_t i=0; i < sizeof(packet); i++) {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        stub_serialRxCallback(packet[i], NULL);
    }

    //no frame complete
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

    //check that channel values have not been updated
    for (int i=0; i<14; i++) {
        EXPECT_NE(i + (0x33 << 8), rxRuntimeState.rcReadRawFn(&rxRuntimeState, i));
    }
}


TEST_F(IbusRxProtocollUnitTest, Test_IA6B_OnePacketReceived_not_shared_port)
{
    static uint16_t channelXData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState.channelXData = channelXData;
    
    uint8_t packet[] = {0x20, 0x00, //length and reserved (unknown) bits
                        0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, //channel 1..5
                        0x05, 0x00, 0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0x09, 0x00, //channel 6..10
                        0x0a, 0x00, 0x0b, 0x00, 0x0c, 0x00, 0x0d, 0x00,             //channel 11..14
                        0x84, 0xff}; //checksum

    {

        serialTestResetPort();
        resetStubTelemetry();
        portIsShared = false;
        serialExpectedOptions = SERIAL_NOT_INVERTED;
        serialExpectedMode = MODE_RX;

        const rxConfig_t initialRxConfig = {};
        findSerialPortConfig_stub_retval = &serialTestInstanceConfig;

        EXPECT_TRUE(ibusInit(&initialRxConfig, &rxRuntimeState));
        EXPECT_FALSE(initSharedIbusTelemetryCalled);

        //handle that internal ibus position is not set to zero at init
        microseconds_stub_value += 5000;
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    }

    for (size_t i=0; i < sizeof(packet); i++) {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
        stub_serialRxCallback(packet[i], NULL);
    }

    //report frame complete once
    EXPECT_EQ(RX_FRAME_COMPLETE, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));

    //check that channel values have been updated
    for (int i=0; i<14; i++) {
        EXPECT_EQ(i, rxRuntimeState.rcReadRawFn(&rxRuntimeState, i));
    }
}


TEST_F(IbusRxProtocollUnitTest, Test_OneTelemetryPacketReceived)
{
    uint8_t packet[] = {0x04, 0x81, 0x7a, 0xff}; //ibus sensor discovery
    resetStubTelemetry();

    receivePacket(packet, sizeof(packet));

    //no frame complete signal to rx system, but telemetry system is called
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeState.rcFrameStatusFn(&rxRuntimeState));
    EXPECT_TRUE(stubTelemetryCalled);
    EXPECT_TRUE( 0 == memcmp( stubTelemetryPacket, packet, sizeof(packet)));
}


TEST_F(IbusRxProtocollUnitTest, Test_OneTelemetryIgnoreTxEchoToRx)
{
    uint8_t packet[] = {0x04, 0x81, 0x7a, 0xff}; //ibus sensor discovery
    resetStubTelemetry();
    stubTelemetryIgnoreRxChars = 4;

    //given one packet received, that will respond with four characters to be ignored
    receivePacket(packet, sizeof(packet));
    rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
    EXPECT_TRUE(stubTelemetryCalled);

    //when those four bytes are sent and looped back
    resetStubTelemetry();
    rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
    receivePacket(packet, sizeof(packet));

    //then they are ignored
    EXPECT_FALSE(stubTelemetryCalled);

    //and then next packet can be received
    receivePacket(packet, sizeof(packet));
    rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
    EXPECT_TRUE(stubTelemetryCalled);
}


TEST_F(IbusRxProtocollUnitTest, Test_OneTelemetryShouldNotIgnoreTxEchoAfterInterFrameGap)
{
    uint8_t packet[] = {0x04, 0x81, 0x7a, 0xff}; //ibus sensor discovery
    resetStubTelemetry();
    stubTelemetryIgnoreRxChars = 4;

    //given one packet received, that will respond with four characters to be ignored
    receivePacket(packet, sizeof(packet));
    rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
    EXPECT_TRUE(stubTelemetryCalled);

    //when there is an interPacketGap
    microseconds_stub_value += 5000;
    resetStubTelemetry();
    rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);

    //then next packet can be received
    receivePacket(packet, sizeof(packet));
    rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
    EXPECT_TRUE(stubTelemetryCalled);
}
