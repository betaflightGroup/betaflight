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


#include "gtest/gtest.h"

extern "C" {
    #include <stdbool.h>
    #include <stdint.h>
    #include <ctype.h>

    #include "platform.h"

    #include "common/bitarray.h"
    #include "common/maths.h"
    #include "common/utils.h"
    #include "common/streambuf.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "flight/imu.h"

    #include "drivers/serial.h"

    #include "io/beeper.h"
    #include "io/serial.h"

    #include "scheduler/scheduler.h"
    #include "io/rcdevice_cam.h"
    #include "io/rcdevice.h"

    #include "osd/osd.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/vcd.h"
    #include "pg/rx.h"
    #include "pg/rcdevice.h"

    #include "rx/rx.h"

    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]

    extern rcdeviceSwitchState_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
    extern runcamDevice_t *camDevice;
    extern bool isButtonPressed;
    extern bool rcdeviceInMenu;
    extern rcdeviceWaitingResponseQueue waitingResponseQueue;
    PG_REGISTER_WITH_RESET_FN(rcdeviceConfig_t, rcdeviceConfig, PG_RCDEVICE_CONFIG, 0);
    bool unitTestIsSwitchActivited(boxId_e boxId)
    {
        uint8_t adjustBoxID = boxId - BOXCAMERA1;
        rcdeviceSwitchState_s switchState = switchStates[adjustBoxID];
        return switchState.isActivated;
    }


    void pgResetFn_rcdeviceConfig(rcdeviceConfig_t *rcdeviceConfig)
    {
        rcdeviceConfig->initDeviceAttempts = 4;
        rcdeviceConfig->initDeviceAttemptInterval = 1000;

        rcdeviceConfig->feature = 0;
        rcdeviceConfig->protocolVersion = 0;
    }

    uint32_t millis(void);
    int minTimeout = 180;

    void rcdeviceSend5KeyOSDCableSimualtionEvent(rcdeviceCamSimulationKeyEvent_e key);
    rcdeviceResponseParseContext_t* rcdeviceRespCtxQueueShift(rcdeviceWaitingResponseQueue *queue);
}

#define MAX_RESPONSES_COUNT 10
#define FIVE_KEY_JOYSTICK_MIN FIVE_KEY_CABLE_JOYSTICK_MIN - 1
#define FIVE_KEY_JOYSTICK_MID FIVE_KEY_CABLE_JOYSTICK_MID_START + 1
#define FIVE_KEY_JOYSTICK_MAX FIVE_KEY_CABLE_JOYSTICK_MAX + 1

typedef struct testData_s {
    bool isRunCamSplitPortConfigurated;
    bool isRunCamSplitOpenPortSupported;
    int8_t maxTimesOfRespDataAvailable;
    bool isAllowBufferReadWrite;
    uint8_t indexOfCurrentRespBuf;
    uint8_t responseBufCount;
    uint8_t responesBufs[MAX_RESPONSES_COUNT][RCDEVICE_PROTOCOL_MAX_PACKET_SIZE];
    uint8_t responseBufsLen[MAX_RESPONSES_COUNT];
    uint8_t responseDataReadPos;
    uint32_t millis;
} testData_t;

static testData_t testData;
extern rcdeviceWaitingResponseQueue waitingResponseQueue;

static void clearResponseBuff()
{
    testData.indexOfCurrentRespBuf = 0;
    testData.responseBufCount = 0;
    memset(testData.responseBufsLen, 0, MAX_RESPONSES_COUNT);
    memset(testData.responesBufs, 0, MAX_RESPONSES_COUNT * 60);

    while (rcdeviceRespCtxQueueShift(&waitingResponseQueue)) {

    }
}

static void resetRCDeviceStatus()
{
    isButtonPressed = false;
    rcdeviceInMenu = false;
    PG_RESET(rcdeviceConfig);
    clearResponseBuff();
}



static void addResponseData(uint8_t *data, uint8_t dataLen, bool withDataForFlushSerial)
{
    UNUSED(withDataForFlushSerial);
    memcpy(testData.responesBufs[testData.responseBufCount], data, dataLen);
    testData.responseBufsLen[testData.responseBufCount] = dataLen;
    testData.responseBufCount++;
}

TEST(RCDeviceTest, TestRCSplitInitWithoutPortConfigurated)
{
    runcamDevice_t device;
    
    resetRCDeviceStatus();

    waitingResponseQueue.headPos = 0;
    waitingResponseQueue.tailPos = 0;
    waitingResponseQueue.itemCount = 0;
    memset(&testData, 0, sizeof(testData));
    runcamDeviceInit(&device);
    EXPECT_FALSE(device.isReady);
}

TEST(RCDeviceTest, TestRCSplitInitWithoutOpenPortConfigurated)
{
    runcamDevice_t device;

    resetRCDeviceStatus();

    waitingResponseQueue.headPos = 0;
    waitingResponseQueue.tailPos = 0;
    waitingResponseQueue.itemCount = 0;
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = false;
    testData.isRunCamSplitPortConfigurated = true;

    runcamDeviceInit(&device);
    EXPECT_FALSE(device.isReady);
}

TEST(RCDeviceTest, TestInitDevice)
{
    runcamDevice_t device;

    resetRCDeviceStatus();

    // test correct response
    waitingResponseQueue.headPos = 0;
    waitingResponseQueue.tailPos = 0;
    waitingResponseQueue.itemCount = 0;
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    
    
    runcamDeviceInit(&device);
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    addResponseData(responseData, sizeof(responseData), true);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    rcdeviceReceive(millis() * 1000);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(device.isReady);
}

TEST(RCDeviceTest, TestInitDeviceWithInvalidResponse)
{
    runcamDevice_t device;

    resetRCDeviceStatus();

    // test correct response data with incorrect len
    waitingResponseQueue.headPos = 0;
    waitingResponseQueue.tailPos = 0;
    waitingResponseQueue.itemCount = 0;
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;

    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD, 0x33 };
    addResponseData(responseData, sizeof(responseData), true);
    runcamDeviceInit(&device);
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    rcdeviceReceive(millis() * 1000);
    EXPECT_TRUE(device.isReady);
    clearResponseBuff();
    testData.millis += minTimeout;

    // invalid crc
    uint8_t responseDataWithInvalidCRC[] = { 0xCC, 0x01, 0x37, 0x00, 0xBE };
    addResponseData(responseDataWithInvalidCRC, sizeof(responseDataWithInvalidCRC), true);
    runcamDeviceInit(&device);
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    rcdeviceReceive(millis() * 1000);
    EXPECT_FALSE(device.isReady);
    clearResponseBuff();
    testData.millis += minTimeout;

    // incomplete response data
    uint8_t incompleteResponseData[] = { 0xCC, 0x01, 0x37 };
    addResponseData(incompleteResponseData, sizeof(incompleteResponseData), true);
    runcamDeviceInit(&device);
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(device.isReady);
    clearResponseBuff();
    testData.millis += minTimeout;

    // test timeout
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    runcamDeviceInit(&device);
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    rcdeviceReceive(millis() * 1000);
    EXPECT_FALSE(device.isReady);
    clearResponseBuff();
    testData.millis += minTimeout;
}

TEST(RCDeviceTest, TestWifiModeChangeWithDeviceUnready)
{
    resetRCDeviceStatus();

    // test correct response
    waitingResponseQueue.headPos = 0;
    waitingResponseQueue.tailPos = 0;
    waitingResponseQueue.itemCount = 0;
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBC }; // wrong response
    addResponseData(responseData, sizeof(responseData), true);
    rcdeviceInit();
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(camDevice->isReady);

    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    for (uint8_t i = 0; i <= (BOXCAMERA3 - BOXCAMERA1); i++) {
        memset(modeActivationConditionsMutable(i), 0, sizeof(modeActivationCondition_t));
    }

    // bind aux1 to wifi button with range [900,1600]
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXCAMERA1;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    // bind aux2 to power button with range [1900, 2100]
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXCAMERA2;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    // bind aux3 to change mode with range [1300, 1600]
    modeActivationConditionsMutable(2)->auxChannelIndex = 2;
    modeActivationConditionsMutable(2)->modeId = BOXCAMERA3;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    analyzeModeActivationConditions();
    
    // make the binded mode inactive
    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1800;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 900;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 900;

    updateActivatedModes();

    // runn process loop
    rcdeviceUpdate(0);

    // remove all request from queue
    for (int i = 0; i < 10; i++) {
        testData.millis += 500000;
        rcdeviceReceive(millis());
    }

    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA3));
}

TEST(RCDeviceTest, TestWifiModeChangeWithDeviceReady)
{
    resetRCDeviceStatus();

    // test correct response
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    addResponseData(responseData, sizeof(responseData), true);

    camDevice->info.features = 15;
    rcdeviceInit();
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    

    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(camDevice->isReady);

    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    for (uint8_t i = 0; i <= BOXCAMERA3 - BOXCAMERA1; i++) {
        memset(modeActivationConditionsMutable(i), 0, sizeof(modeActivationCondition_t));
    }


    // bind aux1 to wifi button with range [900,1600]
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXCAMERA1;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    // bind aux2 to power button with range [1900, 2100]
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXCAMERA2;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    // bind aux3 to change mode with range [1300, 1600]
    modeActivationConditionsMutable(2)->auxChannelIndex = 2;
    modeActivationConditionsMutable(2)->modeId = BOXCAMERA3;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    analyzeModeActivationConditions();

    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1700;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 2000;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1700;

    updateActivatedModes();

    // runn process loop
    int8_t randNum = rand() % 127 + 6;
    testData.maxTimesOfRespDataAvailable = randNum;
    rcdeviceUpdate((timeUs_t)0);

    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_TRUE(unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA3));

    // remove all request from queue
    for (int i = 0; i < 10; i++) {
        testData.millis += 500000;
        rcdeviceReceive(millis());
    }
}

TEST(RCDeviceTest, TestWifiModeChangeCombine)
{
    resetRCDeviceStatus();

    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    addResponseData(responseData, sizeof(responseData), true);
    rcdeviceInit();
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(camDevice->isReady);

    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    for (uint8_t i = 0; i <= BOXCAMERA3 - BOXCAMERA1; i++) {
        memset(modeActivationConditionsMutable(i), 0, sizeof(modeActivationCondition_t));
    }


    // bind aux1 to wifi button with range [900,1600]
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXCAMERA1;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    // bind aux2 to power button with range [1900, 2100]
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXCAMERA2;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    // bind aux3 to change mode with range [1300, 1600]
    modeActivationConditionsMutable(2)->auxChannelIndex = 2;
    modeActivationConditionsMutable(2)->modeId = BOXCAMERA3;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    analyzeModeActivationConditions();

    // // make the binded mode inactive
    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1700;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 2000;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1700;
    updateActivatedModes();

    // runn process loop
    int8_t randNum = rand() % 127 + 6;
    testData.maxTimesOfRespDataAvailable = randNum;
    rcdeviceUpdate((timeUs_t)0);

    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_TRUE(unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA3));


    // // make the binded mode inactive
    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1500;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1300;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1900;
    updateActivatedModes();
    rcdeviceUpdate((timeUs_t)0);
    EXPECT_TRUE(unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_TRUE(unitTestIsSwitchActivited(BOXCAMERA3));


    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1899;
    updateActivatedModes();
    rcdeviceUpdate((timeUs_t)0);
    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA3));

    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 2001;
    updateActivatedModes();
    rcdeviceUpdate((timeUs_t)0);
    EXPECT_TRUE(unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_TRUE(unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_FALSE(unitTestIsSwitchActivited(BOXCAMERA3));

    // remove all request from queue
    for (int i = 0; i < 10; i++) {
        testData.millis += 500000;
        rcdeviceReceive(millis());
    }
}

TEST(RCDeviceTest, Test5KeyOSDCableSimulationProtocol)
{
    resetRCDeviceStatus();

    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    addResponseData(responseData, sizeof(responseData), true);
    rcdeviceInit();
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(camDevice->isReady);
    clearResponseBuff();

    // test timeout of open connection
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_OPEN);
    rcdeviceReceive(millis() * 1000);
    testData.millis += 3000;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(rcdeviceInMenu);
    clearResponseBuff();

    // open connection with correct response
    uint8_t responseDataOfOpenConnection[] = { 0xCC, 0x11, 0xe7 };
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_OPEN);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(rcdeviceInMenu);
    clearResponseBuff();

    // open connection with correct response but wrong data length 
    uint8_t incorrectResponseDataOfOpenConnection1[] = { 0xCC, 0x11, 0xe7, 0x55 };
    addResponseData(incorrectResponseDataOfOpenConnection1, sizeof(incorrectResponseDataOfOpenConnection1), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_OPEN);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(rcdeviceInMenu);
    clearResponseBuff();
    
    // open connection with invalid crc
    uint8_t incorrectResponseDataOfOpenConnection2[] = { 0xCC, 0x10, 0x42 };
    addResponseData(incorrectResponseDataOfOpenConnection2, sizeof(incorrectResponseDataOfOpenConnection2), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_OPEN);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(rcdeviceInMenu); // when crc wrong won't change the menu state
    clearResponseBuff();

    // test timeout of close connection
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_CLOSE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += 3000;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(rcdeviceInMenu); // close menu timeout won't change the menu state
    clearResponseBuff();

    // close connection with correct response
    uint8_t responseDataOfCloseConnection[] = { 0xCC, 0x21, 0x11 };
    addResponseData(responseDataOfCloseConnection, sizeof(responseDataOfCloseConnection), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_CLOSE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(rcdeviceInMenu);
    clearResponseBuff();

    // close connection with correct response but wrong data length 
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_OPEN); // open menu again
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(rcdeviceInMenu);
    clearResponseBuff();

    uint8_t responseDataOfCloseConnection1[] = { 0xCC, 0x21, 0x11, 0xC1 };
    addResponseData(responseDataOfCloseConnection1, sizeof(responseDataOfCloseConnection1), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_CLOSE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(rcdeviceInMenu);
    clearResponseBuff();

    // close connection with response that invalid crc
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_OPEN); // open menu again
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(rcdeviceInMenu);
    clearResponseBuff();

    uint8_t responseDataOfCloseConnection2[] = { 0xCC, 0x21, 0xA1 };
    addResponseData(responseDataOfCloseConnection2, sizeof(responseDataOfCloseConnection2), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_CONNECTION_CLOSE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(rcdeviceInMenu);
    clearResponseBuff();

    // release button first
    uint8_t responseDataOfSimulation4[] = { 0xCC, 0xA5 };
    addResponseData(responseDataOfSimulation4, sizeof(responseDataOfSimulation4), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_RELEASE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(isButtonPressed);
    clearResponseBuff();

    // simulate press button with no response
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_ENTER);
    testData.millis += 2000;
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(isButtonPressed);
    clearResponseBuff();

    // simulate press button with correct response
    uint8_t responseDataOfSimulation1[] = { 0xCC, 0xA5 };
    addResponseData(responseDataOfSimulation1, sizeof(responseDataOfSimulation1), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_ENTER);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(isButtonPressed);
    clearResponseBuff();

    // simulate press button with correct response but wrong data length 
    addResponseData(responseDataOfSimulation4, sizeof(responseDataOfSimulation4), true); // release first
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_RELEASE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(isButtonPressed);
    clearResponseBuff();

    uint8_t responseDataOfSimulation2[] = { 0xCC, 0xA5, 0x22 };
    addResponseData(responseDataOfSimulation2, sizeof(responseDataOfSimulation2), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_ENTER);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(isButtonPressed);
    clearResponseBuff();

    // simulate press button event with incorrect response
    uint8_t responseDataOfSimulation3[] = { 0xCC, 0xB5, 0x22 };
    addResponseData(responseDataOfSimulation3, sizeof(responseDataOfSimulation3), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_ENTER);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(isButtonPressed);
    clearResponseBuff();

    // simulate release button with correct response
    addResponseData(responseDataOfSimulation4, sizeof(responseDataOfSimulation4), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_RELEASE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(isButtonPressed);
    clearResponseBuff();

    // simulate release button with correct response but wrong data length
    addResponseData(responseDataOfSimulation1, sizeof(responseDataOfSimulation1), true); // press first
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_ENTER);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_TRUE(isButtonPressed);
    clearResponseBuff();

    uint8_t responseDataOfSimulation5[] = { 0xCC, 0xA5, 0xFF };
    addResponseData(responseDataOfSimulation5, sizeof(responseDataOfSimulation5), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_RELEASE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(isButtonPressed);
    clearResponseBuff();

    // simulate release button with incorrect response
    uint8_t responseDataOfSimulation6[] = { 0xCC, 0x31, 0xFF };
    addResponseData(responseDataOfSimulation6, sizeof(responseDataOfSimulation6), true);
    rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_RELEASE);
    rcdeviceReceive(millis() * 1000);
    testData.millis += minTimeout;
    EXPECT_FALSE(isButtonPressed);
    clearResponseBuff();

    // remove all request from queue
    for (int i = 0; i < 300; i++) {
        testData.millis += 500000;
        rcdeviceReceive(millis());
    }
}

TEST(RCDeviceTest, Test5KeyOSDCableSimulationWithout5KeyFeatureSupport)
{
    resetRCDeviceStatus();
    
    // test simulation without device init
    rcData[THROTTLE] = FIVE_KEY_JOYSTICK_MID; // THROTTLE Mid
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MAX; // Yaw High
    rcdeviceUpdate(millis() * 1000);
    EXPECT_FALSE(rcdeviceInMenu);
    // remove all request from queue
    for (int i = 0; i < 10; i++) {
        testData.millis += 500000;
        rcdeviceReceive(millis());
    }

    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    
    rcdeviceInit();
    testData.millis += 3001;
    rcdeviceReceive(millis() * 1000);
    testData.millis += 200;
    testData.responseDataReadPos = 0;
    testData.indexOfCurrentRespBuf = 0;
    addResponseData(responseData, sizeof(responseData), true);
    rcdeviceReceive(millis() * 1000);
    testData.millis += 200;
    EXPECT_TRUE(camDevice->isReady);
    clearResponseBuff();

    // open connection, rcdeviceInMenu will be false if the codes is right
    uint8_t responseDataOfOpenConnection[] = { 0xCC, 0x11, 0xe7 };
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), false);
    rcdeviceUpdate(millis() * 1000);
    EXPECT_FALSE(rcdeviceInMenu);
    clearResponseBuff();

    // remove all request from queue
    for (int i = 0; i < 10; i++) {
        testData.millis += 500000;
        rcdeviceReceive(millis());
    }
}

extern "C" {
    serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e functionMask, serialReceiveCallbackPtr callback, void *callbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
    {
        UNUSED(identifier);
        UNUSED(functionMask);
        UNUSED(baudRate);
        UNUSED(mode);
        UNUSED(options);

        if (testData.isRunCamSplitOpenPortSupported) {
            static serialPort_t s;
            s.vTable = NULL;

            // common serial initialisation code should move to serialPort::init()
            s.rxBufferHead = s.rxBufferTail = 0;
            s.txBufferHead = s.txBufferTail = 0;
            s.rxBufferSize = 0;
            s.txBufferSize = 0;
            s.rxBuffer = s.rxBuffer;
            s.txBuffer = s.txBuffer;

            // callback works for IRQ-based RX ONLY
            s.rxCallback = callback;
            s.rxCallbackData = callbackData;
            s.baudRate = 0;

            return (serialPort_t *)&s;
        }

        return NULL;
    }

    const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
    {
        UNUSED(function);
        if (testData.isRunCamSplitPortConfigurated) {
            static serialPortConfig_t portConfig;

            portConfig.identifier = SERIAL_PORT_USART3;
            portConfig.msp_baudrateIndex = BAUD_115200;
            portConfig.gps_baudrateIndex = BAUD_57600;
            portConfig.telemetry_baudrateIndex = BAUD_AUTO;
            portConfig.blackbox_baudrateIndex = BAUD_115200;
            portConfig.functionMask = FUNCTION_MSP;

            return &portConfig;
        }

        return NULL;
    }

    uint32_t serialRxBytesWaiting(const serialPort_t *instance) 
    { 
        UNUSED(instance);

        uint8_t bufIndex = testData.indexOfCurrentRespBuf;
        uint8_t leftDataLen = 0;
        if (testData.responseDataReadPos + 1 > testData.responseBufsLen[bufIndex]) {
            return 0;
        } else {
            leftDataLen = testData.responseBufsLen[bufIndex] - (testData.responseDataReadPos);
        }

        if (leftDataLen) {
            return leftDataLen;
        }

        return 0;
    }

    uint8_t serialRead(serialPort_t *instance) 
    { 
        UNUSED(instance);

        uint8_t bufIndex = testData.indexOfCurrentRespBuf;
        uint8_t *buffer = NULL;
        uint8_t leftDataLen = 0;
        if (testData.responseDataReadPos >= testData.responseBufsLen[bufIndex]) {
            leftDataLen = 0;
        } else {
            buffer = testData.responesBufs[bufIndex];
            leftDataLen = testData.responseBufsLen[bufIndex] - testData.responseDataReadPos;
        }

        if (leftDataLen) {
            return buffer[testData.responseDataReadPos++];
        }

        return 0; 
    }

    void sbufWriteString(sbuf_t *dst, const char *string) 
    { 
        UNUSED(dst); UNUSED(string); 

        if (testData.isAllowBufferReadWrite) {
            sbufWriteData(dst, string, strlen(string));
        }
    }
    void sbufWriteU8(sbuf_t *dst, uint8_t val) 
    { 
        UNUSED(dst); UNUSED(val); 

        if (testData.isAllowBufferReadWrite) {
            *dst->ptr++ = val;
        }
    }
    
    void sbufWriteData(sbuf_t *dst, const void *data, int len)
    {
        UNUSED(dst); UNUSED(data); UNUSED(len); 

        if (testData.isAllowBufferReadWrite) {
            memcpy(dst->ptr, data, len);
            dst->ptr += len;
            
        }
    }

    // modifies streambuf so that written data are prepared for reading
    void sbufSwitchToReader(sbuf_t *buf, uint8_t *base)
    {
        UNUSED(buf); UNUSED(base); 

        if (testData.isAllowBufferReadWrite) {
            buf->end = buf->ptr;
            buf->ptr = base;
        }
    }

    uint8_t sbufReadU8(sbuf_t *src)
    {
        if (testData.isAllowBufferReadWrite) {
            return *src->ptr++;
        }

        return 0;
    }

    void sbufAdvance(sbuf_t *buf, int size)
    {
        if (testData.isAllowBufferReadWrite) {
            buf->ptr += size;
        }
    }

    int sbufBytesRemaining(sbuf_t *buf)
    {
        if (testData.isAllowBufferReadWrite) {
            return buf->end - buf->ptr;
        }
        return 0;
    }

    const uint8_t* sbufConstPtr(const sbuf_t *buf)
    {
        return buf->ptr;
    }

    void sbufReadData(sbuf_t *src, void *data, int len)
    {
        if (testData.isAllowBufferReadWrite) {
            memcpy(data, src->ptr, len);
        }
    }

    uint16_t sbufReadU16(sbuf_t *src)
    {
        uint16_t ret;
        ret = sbufReadU8(src);
        ret |= sbufReadU8(src) << 8;
        return ret;
    }

    void sbufWriteU16(sbuf_t *dst, uint16_t val)
    {
        sbufWriteU8(dst, val >> 0);
        sbufWriteU8(dst, val >> 8);
    }

    void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val)
    {
        sbufWriteU8(dst, val >> 8);
        sbufWriteU8(dst, (uint8_t)val);
    }

    bool featureIsEnabled(uint32_t) { return false; }

    void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count) 
    { 
        UNUSED(instance); UNUSED(data); UNUSED(count); 

        // reset the input buffer
        testData.responseDataReadPos = 0;
        testData.indexOfCurrentRespBuf++;
        if (testData.indexOfCurrentRespBuf >= testData.responseBufCount) {
            testData.indexOfCurrentRespBuf = 0;
        }
        // testData.maxTimesOfRespDataAvailable = testData.responseDataLen + 1;
    }

    const serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function)
    {
        UNUSED(function);

        return NULL;
    }

    void closeSerialPort(serialPort_t *serialPort)
    {
        UNUSED(serialPort);
    }

    uint8_t* sbufPtr(sbuf_t *buf)
    {
        return buf->ptr;
    }

    uint32_t sbufReadU32(sbuf_t *src)
    {
        uint32_t ret;
        ret = sbufReadU8(src);
        ret |= sbufReadU8(src) <<  8;
        ret |= sbufReadU8(src) << 16;
        ret |= sbufReadU8(src) << 24;
        return ret;
    }
    
    uint32_t millis(void) { return testData.millis++; }
    uint32_t micros(void) { return millis() * 1000; }
    void beeper(beeperMode_e mode) { UNUSED(mode); }
    uint8_t armingFlags = 0;
    bool cmsInMenu;
    uint32_t resumeRefreshAt = 0;
    int getArmingDisableFlags(void) {return 0;}
    void pinioBoxTaskControl(void) {}
    attitudeEulerAngles_t attitude = { { 0, 0, 0 } };
}
