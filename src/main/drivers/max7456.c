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
#include <string.h>

#include "platform.h"

#ifdef USE_MAX7456

#include "build/debug.h"

#include "pg/max7456.h"
#include "pg/vcd.h"

#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/max7456.h"
#include "drivers/nvic.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"


// 10 MHz max SPI frequency
#define MAX7456_MAX_SPI_CLK_HZ 10000000
#define MAX7456_INIT_MAX_SPI_CLK_HZ 5000000

// DEBUG_MAX7456_SIGNAL
#define DEBUG_MAX7456_SIGNAL_MODEREG       0
#define DEBUG_MAX7456_SIGNAL_SENSE         1
#define DEBUG_MAX7456_SIGNAL_REINIT        2
#define DEBUG_MAX7456_SIGNAL_ROWS          3

// DEBUG_MAX7456_SPICLOCK
#define DEBUG_MAX7456_SPICLOCK_OVERCLOCK   0
#define DEBUG_MAX7456_SPICLOCK_DEVTYPE     1
#define DEBUG_MAX7456_SPICLOCK_DIVISOR     2

// VM0 bits
#define VIDEO_BUFFER_DISABLE        0x01
#define MAX7456_RESET               0x02
#define VERTICAL_SYNC_NEXT_VSYNC    0x04
#define OSD_ENABLE                  0x08

#define SYNC_MODE_AUTO              0x00
#define SYNC_MODE_INTERNAL          0x30
#define SYNC_MODE_EXTERNAL          0x20

#define VIDEO_MODE_PAL              0x40
#define VIDEO_MODE_NTSC             0x00
#define VIDEO_MODE_MASK             0x40
#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

#define VIDEO_SIGNAL_DEBOUNCE_MS    100 // Time to wait for input to stabilize

// VM1 bits

// duty cycle is on_off
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03

// blinking time
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C

// background mode brightness (percent)
#define BACKGROUND_BRIGHTNESS_0 0x00
#define BACKGROUND_BRIGHTNESS_7 0x01
#define BACKGROUND_BRIGHTNESS_14 0x02
#define BACKGROUND_BRIGHTNESS_21 0x03
#define BACKGROUND_BRIGHTNESS_28 0x04
#define BACKGROUND_BRIGHTNESS_35 0x05
#define BACKGROUND_BRIGHTNESS_42 0x06
#define BACKGROUND_BRIGHTNESS_49 0x07

#define BACKGROUND_MODE_GRAY 0x80

// STAT register bits

#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_NVR_BUSY 0x20

#define STAT_IS_PAL(val)  ((val) & STAT_PAL)
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))

// DMM register bits
#define DMM_AUTO_INC 0x01

// Kluege warning!
// There are occasions that NTSC is not detected even with !LOS (AB7456 specific?)
// When this happens, lower 3 bits of STAT register is read as zero.
// To cope with this case, this macro defines !LOS && !PAL as NTSC.
// Should be compatible with MAX7456 and non-problematic case.

#define VIN_IS_NTSC_alt(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))

#define MAX7456_SIGNAL_CHECK_INTERVAL_MS 1000 // msec
#define MAX7456_STALL_CHECK_INTERVAL_MS  1000 // msec

// DMM special bits
#define CLEAR_DISPLAY 0x04
#define CLEAR_DISPLAY_VERT 0x06
#define INVERT_PIXEL_COLOR 0x08

// Special address for terminating incremental write
#define END_STRING 0xff

#define MAX7456ADD_READ         0x80
#define MAX7456ADD_VM0          0x00  //0b0011100// 00 // 00             ,0011100
#define MAX7456ADD_VM1          0x01
#define MAX7456ADD_HOS          0x02
#define MAX7456ADD_VOS          0x03
#define MAX7456ADD_DMM          0x04
#define MAX7456ADD_DMAH         0x05
#define MAX7456ADD_DMAL         0x06
#define MAX7456ADD_DMDI         0x07
#define MAX7456ADD_CMM          0x08
#define MAX7456ADD_CMAH         0x09
#define MAX7456ADD_CMAL         0x0a
#define MAX7456ADD_CMDI         0x0b
#define MAX7456ADD_OSDM         0x0c
#define MAX7456ADD_RB0          0x10
#define MAX7456ADD_RB1          0x11
#define MAX7456ADD_RB2          0x12
#define MAX7456ADD_RB3          0x13
#define MAX7456ADD_RB4          0x14
#define MAX7456ADD_RB5          0x15
#define MAX7456ADD_RB6          0x16
#define MAX7456ADD_RB7          0x17
#define MAX7456ADD_RB8          0x18
#define MAX7456ADD_RB9          0x19
#define MAX7456ADD_RB10         0x1a
#define MAX7456ADD_RB11         0x1b
#define MAX7456ADD_RB12         0x1c
#define MAX7456ADD_RB13         0x1d
#define MAX7456ADD_RB14         0x1e
#define MAX7456ADD_RB15         0x1f
#define MAX7456ADD_OSDBL        0x6c
#define MAX7456ADD_STAT         0xA0

#define NVM_RAM_SIZE            54
#define WRITE_NVR               0xA0

// Device type
#define MAX7456_DEVICE_TYPE_MAX 0
#define MAX7456_DEVICE_TYPE_AT  1

#define CHARS_PER_LINE      30 // XXX Should be related to VIDEO_BUFFER_CHARS_*?

#define MAX7456_SUPPORTED_LAYER_COUNT (DISPLAYPORT_LAYER_BACKGROUND + 1)

typedef struct max7456Layer_s {
    uint8_t buffer[VIDEO_BUFFER_CHARS_PAL];
} max7456Layer_t;

static max7456Layer_t displayLayers[MAX7456_SUPPORTED_LAYER_COUNT];
static displayPortLayer_e activeLayer = DISPLAYPORT_LAYER_FOREGROUND;

extDevice_t max7456Device;
extDevice_t *dev = &max7456Device;

static bool max7456DeviceDetected = false;
static uint16_t max7456SpiClock;

uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

// We write everything to the active layer and then compare
// it with shadowBuffer to update only changed chars.
// This solution is faster then redrawing entire screen.

static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];

//Max bytes to update in one call to max7456DrawScreen()

#define MAX_BYTES2SEND          250
#define MAX_BYTES2SEND_POLLED   25

static DMA_DATA uint8_t spiBuf[MAX_BYTES2SEND];

static uint8_t  videoSignalCfg;
static uint8_t  videoSignalReg  = OSD_ENABLE; // OSD_ENABLE required to trigger first ReInit
static uint8_t  displayMemoryModeReg = 0;

static uint8_t  hosRegValue; // HOS (Horizontal offset register) value
static uint8_t  vosRegValue; // VOS (Vertical offset register) value

static bool fontIsLoading       = false;

static uint8_t max7456DeviceType;

static displayPortBackground_e deviceBackgroundType = DISPLAY_BACKGROUND_TRANSPARENT;

// previous states initialized outside the valid range to force update on first call
#define INVALID_PREVIOUS_REGISTER_STATE 255
static uint8_t previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
static uint8_t previousInvertRegister = INVALID_PREVIOUS_REGISTER_STATE;

static uint8_t *getLayerBuffer(displayPortLayer_e layer)
{
    return displayLayers[layer].buffer;
}

static uint8_t *getActiveLayerBuffer(void)
{
    return getLayerBuffer(activeLayer);
}

static void max7456SetRegisterVM1(void)
{
    uint8_t backgroundGray = BACKGROUND_BRIGHTNESS_28; // this is the device default background gray level
    uint8_t vm1Register = BLINK_TIME_1 | BLINK_DUTY_CYCLE_75_25; // device defaults
    if (deviceBackgroundType != DISPLAY_BACKGROUND_TRANSPARENT) {
        vm1Register |= BACKGROUND_MODE_GRAY;
        switch (deviceBackgroundType) {
        case DISPLAY_BACKGROUND_BLACK:
            backgroundGray = BACKGROUND_BRIGHTNESS_0;
            break;
        case DISPLAY_BACKGROUND_LTGRAY:
            backgroundGray = BACKGROUND_BRIGHTNESS_49;
            break;
        case DISPLAY_BACKGROUND_GRAY:
        default:
            backgroundGray = BACKGROUND_BRIGHTNESS_28;
            break;
        }
    }
    vm1Register |= (backgroundGray << 4);
    spiWriteReg(dev, MAX7456ADD_VM1, vm1Register);
}

uint8_t max7456GetRowsCount(void)
{
    return (videoSignalReg & VIDEO_MODE_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
}

// When clearing the shadow buffer we fill with 0 so that the characters will
// be flagged as changed when compared to the 0x20 used in the layer buffers.
static void max7456ClearShadowBuffer(void)
{
    memset(shadowBuffer, 0, maxScreenSize);
}

// Buffer is filled with the whitespace character (0x20)
static void max7456ClearLayer(displayPortLayer_e layer)
{
    memset(getLayerBuffer(layer), 0x20, VIDEO_BUFFER_CHARS_PAL);
}

void max7456ReInit(void)
{
    uint8_t srdata = 0;

    switch (videoSignalCfg) {
    case VIDEO_SYSTEM_PAL:
        videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        break;

    case VIDEO_SYSTEM_NTSC:
        videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        break;

    case VIDEO_SYSTEM_AUTO:
        srdata = spiReadRegMsk(dev, MAX7456ADD_STAT);

        if (VIN_IS_NTSC(srdata)) {
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        } else if (VIN_IS_PAL(srdata)) {
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        } else {
            // No valid input signal, fallback to default (XXX NTSC for now)
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        }
        break;
    }

    if (videoSignalReg & VIDEO_MODE_PAL) { //PAL
        maxScreenSize = VIDEO_BUFFER_CHARS_PAL;
    } else {              // NTSC
        maxScreenSize = VIDEO_BUFFER_CHARS_NTSC;
    }

    // Set all rows to same charactor black/white level
    previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
    max7456Brightness(0, 2);
    // Re-enable MAX7456 (last function call disables it)

    // Make sure the Max7456 is enabled
    spiWriteReg(dev, MAX7456ADD_VM0, videoSignalReg);
    spiWriteReg(dev, MAX7456ADD_HOS, hosRegValue);
    spiWriteReg(dev, MAX7456ADD_VOS, vosRegValue);

    max7456SetRegisterVM1();

    // Clear shadow to force redraw all screen
    max7456ClearShadowBuffer();
}

void max7456PreInit(const max7456Config_t *max7456Config)
{
    spiPreinitRegister(max7456Config->csTag, max7456Config->preInitOPU ? IOCFG_OUT_PP : IOCFG_IPU, 1);
}

// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)

max7456InitStatus_e max7456Init(const max7456Config_t *max7456Config, const vcdProfile_t *pVcdProfile, bool cpuOverclock)
{
    max7456DeviceDetected = false;
    deviceBackgroundType = DISPLAY_BACKGROUND_TRANSPARENT;

    // initialize all layers
    for (unsigned i = 0; i < MAX7456_SUPPORTED_LAYER_COUNT; i++) {
        max7456ClearLayer(i);
    }

    max7456HardwareReset();

    if (!max7456Config->csTag || !spiSetBusInstance(dev, max7456Config->spiDevice, OWNER_OSD_CS)) {
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    dev->busType_u.spi.csnPin = IOGetByTag(max7456Config->csTag);

    if (!IOIsFreeOrPreinit(dev->busType_u.spi.csnPin)) {
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);

    // Detect MAX7456 existence and device type. Do this at half the speed for safety.

    // Detect MAX7456 and compatible device by reading OSDM (OSD Insertion MUX) register.
    // This register is not modified in this driver, therefore ensured to remain at its default value (0x1B).

    spiSetClkDivisor(dev, spiCalculateDivider(MAX7456_INIT_MAX_SPI_CLK_HZ));

    // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
    spiWrite(dev, END_STRING);

    uint8_t osdm = spiReadRegMsk(dev, MAX7456ADD_OSDM);

    if (osdm != 0x1B) {
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
        return MAX7456_INIT_NOT_FOUND;
    }

    max7456DeviceDetected = true;
    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);

    // Detect device type by writing and reading CA[8] bit at CMAL[6].
    // This is a bit for accessing second half of character glyph storage, supported only by AT variant.

    spiWriteReg(dev, MAX7456ADD_CMAL, (1 << 6)); // CA[8] bit

    if (spiReadRegMsk(dev, MAX7456ADD_CMAL) & (1 << 6)) {
        max7456DeviceType = MAX7456_DEVICE_TYPE_AT;
    } else {
        max7456DeviceType = MAX7456_DEVICE_TYPE_MAX;
    }

#if defined(USE_OVERCLOCK)
    // Determine SPI clock divisor based on config and the device type.

    switch (max7456Config->clockConfig) {
    case MAX7456_CLOCK_CONFIG_HALF:
        max7456SpiClock = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ / 2);
        break;

    case MAX7456_CLOCK_CONFIG_OC:
        max7456SpiClock = (cpuOverclock && (max7456DeviceType == MAX7456_DEVICE_TYPE_MAX)) ? spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ / 2) : spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ);
        break;

    case MAX7456_CLOCK_CONFIG_FULL:
        max7456SpiClock = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ);
        break;
    }

    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_OVERCLOCK, cpuOverclock);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DEVTYPE, max7456DeviceType);
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DIVISOR, max7456SpiClock);
#else
    UNUSED(max7456Config);
    UNUSED(cpuOverclock);
#endif

    spiSetClkDivisor(dev, max7456SpiClock);

    // force soft reset on Max7456
    spiWriteReg(dev, MAX7456ADD_VM0, MAX7456_RESET);

    // Wait for 100us before polling for completion of reset
    delayMicroseconds(100);

    // Wait for reset to complete
    while ((spiReadRegMsk(dev, MAX7456ADD_VM0) & MAX7456_RESET) != 0x00);

    // Setup values to write to registers
    videoSignalCfg = pVcdProfile->video_system;
    hosRegValue = 32 - pVcdProfile->h_offset;
    vosRegValue = 16 - pVcdProfile->v_offset;

    // Real init will be made later when driver detect idle.
    return MAX7456_INIT_OK;
}

/**
 * Sets inversion of black and white pixels.
 */
void max7456Invert(bool invert)
{
    if (invert) {
        displayMemoryModeReg |= INVERT_PIXEL_COLOR;
    } else {
        displayMemoryModeReg &= ~INVERT_PIXEL_COLOR;
    }

    if (displayMemoryModeReg != previousInvertRegister) {
        // clear the shadow buffer so all characters will be
        // redrawn with the proper invert state
        max7456ClearShadowBuffer();
        previousInvertRegister = displayMemoryModeReg;
        spiWriteReg(dev, MAX7456ADD_DMM, displayMemoryModeReg);
    }
}

/**
 * Sets the brighness of black and white pixels.
 *
 * @param black Black brightness (0-3, 0 is darkest)
 * @param white White brightness (0-3, 0 is darkest)
 */
void max7456Brightness(uint8_t black, uint8_t white)
{
    const uint8_t reg = (black << 2) | (3 - white);

    if (reg != previousBlackWhiteRegister) {
        previousBlackWhiteRegister = reg;
        STATIC_DMA_DATA_AUTO uint8_t buf[32];
        for (int i = MAX7456ADD_RB0, j = 0; i <= MAX7456ADD_RB15; i++) {
            buf[j++] = i;
            buf[j++] = reg;
        }
        spiReadWriteBuf(dev, buf, NULL, sizeof (buf));
    }
}

void max7456ClearScreen(void)
{
    max7456ClearLayer(activeLayer);
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    uint8_t *buffer = getActiveLayerBuffer();
    if (x < CHARS_PER_LINE && y < VIDEO_LINES_PAL) {
        buffer[y * CHARS_PER_LINE + x] = c;
    }
}

void max7456Write(uint8_t x, uint8_t y, const char *buff)
{
    if (y < VIDEO_LINES_PAL) {
        uint8_t *buffer = getActiveLayerBuffer();
        for (int i = 0; buff[i] && x + i < CHARS_PER_LINE; i++) {
            buffer[y * CHARS_PER_LINE + x + i] = buff[i];
        }
    }
}

bool max7456LayerSupported(displayPortLayer_e layer)
{
    if (layer == DISPLAYPORT_LAYER_FOREGROUND || layer == DISPLAYPORT_LAYER_BACKGROUND) {
        return true;
    } else {
        return false;
    }
}

bool max7456LayerSelect(displayPortLayer_e layer)
{
    if (max7456LayerSupported(layer)) {
        activeLayer = layer;
        return true;
    } else {
        return false;
    }
}

bool max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    if ((sourceLayer != destLayer) && max7456LayerSupported(sourceLayer) && max7456LayerSupported(destLayer)) {
        memcpy(getLayerBuffer(destLayer), getLayerBuffer(sourceLayer), VIDEO_BUFFER_CHARS_PAL);
        return true;
    } else {
        return false;
    }
}

bool max7456DmaInProgress(void)
{
    return spiIsBusy(dev);
}

bool max7456BuffersSynced(void)
{
    for (int i = 0; i < maxScreenSize; i++) {
        if (displayLayers[DISPLAYPORT_LAYER_FOREGROUND].buffer[i] != shadowBuffer[i]) {
            return false;
        }
    }
    return true;
}

bool max7456ReInitIfRequired(bool forceStallCheck)
{
    static timeMs_t lastSigCheckMs = 0;
    static timeMs_t videoDetectTimeMs = 0;
    static uint16_t reInitCount = 0;
    static timeMs_t lastStallCheckMs = MAX7456_STALL_CHECK_INTERVAL_MS / 2; // offset so that it doesn't coincide with the signal check

    const timeMs_t nowMs = millis();

    bool stalled = false;
    if (forceStallCheck || (lastStallCheckMs + MAX7456_STALL_CHECK_INTERVAL_MS < nowMs)) {
        lastStallCheckMs = nowMs;

        // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
        spiWrite(dev, END_STRING);

        stalled = (spiReadRegMsk(dev, MAX7456ADD_VM0) != videoSignalReg);
    }

    if (stalled) {
        max7456ReInit();
    } else if ((videoSignalCfg == VIDEO_SYSTEM_AUTO)
              && ((nowMs - lastSigCheckMs) > MAX7456_SIGNAL_CHECK_INTERVAL_MS)) {

        // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
        spiWrite(dev, END_STRING);

        // Adjust output format based on the current input format.

        const uint8_t videoSense = spiReadRegMsk(dev, MAX7456ADD_STAT);

        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_MODEREG, videoSignalReg & VIDEO_MODE_MASK);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_SENSE, videoSense & 0x7);
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_ROWS, max7456GetRowsCount());

        if (videoSense & STAT_LOS) {
            videoDetectTimeMs = 0;
        } else {
            if ((VIN_IS_PAL(videoSense) && VIDEO_MODE_IS_NTSC(videoSignalReg))
              || (VIN_IS_NTSC_alt(videoSense) && VIDEO_MODE_IS_PAL(videoSignalReg))) {
                if (videoDetectTimeMs) {
                    if (millis() - videoDetectTimeMs > VIDEO_SIGNAL_DEBOUNCE_MS) {
                        max7456ReInit();
                        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_REINIT, ++reInitCount);
                    }
                } else {
                    // Wait for signal to stabilize
                    videoDetectTimeMs = millis();
                }
            }
        }

        lastSigCheckMs = nowMs;
    }

    return stalled;
}

// Return true if screen still being transferred
int max7456DrawScreen(void)
{
    static uint16_t pos = 0;
    // This routine doesn't block so need to use static data
    static busSegment_t segments[] = {
            {NULL, NULL, 0, true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    if (!fontIsLoading) {
        uint8_t *buffer = getActiveLayerBuffer();
        int spiBufIndex = 0;
        int maxSpiBufStartIndex;
        bool setAddress = true;
        bool autoInc = false;
        int posLimit = pos + (maxScreenSize / 2);

        maxSpiBufStartIndex = spiUseDMA(dev) ? MAX_BYTES2SEND : MAX_BYTES2SEND_POLLED;

        // Abort for now if the bus is still busy
        if (spiIsBusy(dev)) {
            // Not finished yet
            return true;
        }

        // Ensure any prior DMA has completed before overwriting the buffer
        spiWaitClaim(dev);

        // Allow for 8 bytes followed by an ESCAPE and reset of DMM at end of buffer
        maxSpiBufStartIndex -= 12;

        // Initialise the transfer buffer
        while ((spiBufIndex < maxSpiBufStartIndex) && (pos < posLimit)) {
            if (buffer[pos] != shadowBuffer[pos]) {
                if (buffer[pos] == 0xff) {
                    buffer[pos] = ' ';
                }

                if (setAddress || !autoInc) {
                    if (buffer[pos + 1] != shadowBuffer[pos + 1]) {
                        // It's worth auto incrementing
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
                        spiBuf[spiBufIndex++] = displayMemoryModeReg | DMM_AUTO_INC;
                        autoInc = true;
                    } else {
                        // It's not worth auto incrementing
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
                        spiBuf[spiBufIndex++] = displayMemoryModeReg;
                        autoInc = false;
                    }

                    spiBuf[spiBufIndex++] = MAX7456ADD_DMAH;
                    spiBuf[spiBufIndex++] =  pos >> 8;
                    spiBuf[spiBufIndex++] = MAX7456ADD_DMAL;
                    spiBuf[spiBufIndex++] =  pos & 0xff;

                    setAddress = false;
                }

                spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                spiBuf[spiBufIndex++] =  buffer[pos];

                shadowBuffer[pos] = buffer[pos];
            } else {
                if (!setAddress) {
                    setAddress = true;
                    if (autoInc) {
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                        spiBuf[spiBufIndex++] = END_STRING;
                    }
                }
            }

            if (++pos >= maxScreenSize) {
                pos = 0;
                break;
            }
        }

        if (autoInc) {
            if (!setAddress) {
                spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                spiBuf[spiBufIndex++] = END_STRING;
            }

            spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
            spiBuf[spiBufIndex++] = displayMemoryModeReg;
        }

        if (spiBufIndex) {
            segments[0].txData = spiBuf;
            segments[0].len = spiBufIndex;

            spiSequence(dev, &segments[0]);

            // Non-blocking, so transfer still in progress if using DMA
        }
    }

    return (pos != 0);
}

// should not be used when armed
void max7456RefreshAll(void)
{
    max7456ReInitIfRequired(true);
    while (max7456DrawScreen());
}

bool max7456WriteNvm(uint8_t char_address, const uint8_t *font_data)
{
    if (!max7456DeviceDetected) {
        return false;
    }

    // Block pending completion of any prior SPI access
    spiWait(dev);

    // disable display
    fontIsLoading = true;
    spiWriteReg(dev, MAX7456ADD_VM0, 0);

    spiWriteReg(dev, MAX7456ADD_CMAH, char_address); // set start address high

    for (int x = 0; x < 54; x++) {
        spiWriteReg(dev, MAX7456ADD_CMAL, x); //set start address low
        spiWriteReg(dev, MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }

    // Transfer 54 bytes from shadow ram to NVM

    spiWriteReg(dev, MAX7456ADD_CMM, WRITE_NVR);

    // Wait until bit 5 in the status register returns to 0 (12ms)

    while ((spiReadRegMsk(dev, MAX7456ADD_STAT) & STAT_NVR_BUSY) != 0x00);

    return true;
}

#ifdef MAX7456_NRST_PIN
static IO_t max7456ResetPin        = IO_NONE;
#endif

void max7456HardwareReset(void)
{
#ifdef MAX7456_NRST_PIN
#define IO_RESET_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)

    max7456ResetPin = IOGetByTag(IO_TAG(MAX7456_NRST_PIN));
    IOInit(max7456ResetPin, OWNER_OSD, 0);
    IOConfigGPIO(max7456ResetPin, IO_RESET_CFG);

    // RESET 50ms long pulse, followed by 100us pause
    IOLo(max7456ResetPin);
    delay(50);
    IOHi(max7456ResetPin);
    delayMicroseconds(100);
#else
    // Allow device 50ms to powerup
    delay(50);
#endif
}

bool max7456IsDeviceDetected(void)
{
    return max7456DeviceDetected;
}

void max7456SetBackgroundType(displayPortBackground_e backgroundType)
{
    deviceBackgroundType = backgroundType;

    max7456SetRegisterVM1();
}

#endif // USE_MAX7456
