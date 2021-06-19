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

#ifdef USE_SDCARD_SPI

#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "pg/bus_spi.h"
#include "pg/sdcard.h"

#include "sdcard.h"
#include "sdcard_impl.h"
#include "sdcard_standard.h"

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    #define SDCARD_PROFILING
#endif

#define SDCARD_INIT_NUM_DUMMY_BYTES                 10
#define SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY     8
// Chosen so that CMD8 will have the same CRC as CMD0:
#define SDCARD_IF_COND_CHECK_PATTERN                0xAB

/* Spec calls for under 400KHz */
#define SDCARD_MAX_SPI_INIT_CLK_HZ     400000

/* Operational speed <= 25MHz */
#define SDCARD_MAX_SPI_CLK_HZ          25000000

#define SDCARD_SPI_MODE                             SPI_MODE0_POL_LOW_EDGE_1ST
//#define SDCARD_SPI_MODE                             SPI_MODE3_POL_HIGH_EDGE_2ND

/* Break up 512-byte SD card sectors into chunks of this size when writing without DMA to reduce the peak overhead
 * per call to sdcard_poll().
 */
#define SDCARD_NON_DMA_CHUNK_SIZE                   256

/**
 * Returns true if the card has already been, or is currently, initializing and hasn't encountered enough errors to
 * trip our error threshold and be disabled (i.e. our card is in and working!)
 */
static bool sdcardSpi_isFunctional(void)
{
    return sdcard.state != SDCARD_STATE_NOT_PRESENT;
}

static void sdcard_select(void)
{
#ifdef USE_SPI_TRANSACTION
    spiBusTransactionBegin(&sdcard.busdev);
#else
    IOLo(sdcard.busdev.busdev_u.spi.csnPin);
#endif
}

static void sdcard_deselect(void)
{
    // As per the SD-card spec, give the card 8 dummy clocks so it can finish its operation
    //spiBusTransferByte(&sdcard.busdev, 0xFF);

    while (spiBusIsBusBusy(&sdcard.busdev)) {
    }

    delayMicroseconds(10);
#ifdef USE_SPI_TRANSACTION
    spiBusTransactionEnd(&sdcard.busdev);
#else
    IOHi(sdcard.busdev.busdev_u.spi.csnPin);
#endif
}

/**
 * Handle a failure of an SD card operation by resetting the card back to its initialization phase.
 *
 * Increments the failure counter, and when the failure threshold is reached, disables the card until
 * the next call to sdcard_init().
 */
static void sdcard_reset(void)
{
    if (!sdcard_isInserted()) {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        return;
    }

    if (sdcard.state >= SDCARD_STATE_READY) {
#ifdef USE_SPI_TRANSACTION
        spiBusTransactionInit(&sdcard.busdev, SDCARD_SPI_MODE, spiCalculateDivider(SDCARD_MAX_SPI_INIT_CLK_HZ));
#else
        spiSetDivisor(sdcard.busdev.busdev_u.spi.instance, spiCalculateDivider(SDCARD_MAX_SPI_INIT_CLK_HZ));
#endif
    }

    sdcard.failureCount++;
    if (sdcard.failureCount >= SDCARD_MAX_CONSECUTIVE_FAILURES) {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
    } else {
        sdcard.operationStartTime = millis();
        sdcard.state = SDCARD_STATE_RESET;
    }
}

/**
 * The SD card spec requires 8 clock cycles to be sent by us on the bus after most commands so it can finish its
 * processing of that command. The easiest way for us to do this is to just wait for the bus to become idle before
 * we transmit a command, sending at least 8-bits onto the bus when we do so.
 */
static bool sdcard_waitForIdle(int maxBytesToWait)
{
    while (maxBytesToWait > 0) {
        uint8_t b = spiBusTransferByte(&sdcard.busdev, 0xFF);
        if (b == 0xFF) {
            return true;
        }
        maxBytesToWait--;
    }

    return false;
}

/**
 * Wait for up to maxDelay 0xFF idle bytes to arrive from the card, returning the first non-idle byte found.
 *
 * Returns 0xFF on failure.
 */
static uint8_t sdcard_waitForNonIdleByte(int maxDelay)
{
    for (int i = 0; i < maxDelay + 1; i++) { // + 1 so we can wait for maxDelay '0xFF' bytes before reading a response byte afterwards
        uint8_t response = spiBusTransferByte(&sdcard.busdev, 0xFF);

        if (response != 0xFF) {
            return response;
        }
    }

    return 0xFF;
}

/**
 * Waits up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes for the card to become ready, send a command to the card
 * with the given argument, waits up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes for a reply, and returns the
 * first non-0xFF byte of the reply.
 *
 * You must select the card first with sdcard_select() and deselect it afterwards with sdcard_deselect().
 *
 * Upon failure, 0xFF is returned.
 */
static uint8_t sdcard_sendCommand(uint8_t commandCode, uint32_t commandArgument)
{
    const uint8_t command[6] = {
        0x40 | commandCode,
        commandArgument >> 24,
        commandArgument >> 16,
        commandArgument >> 8,
        commandArgument,
        0x95 /* Static CRC. This CRC is valid for CMD0 with a 0 argument, and CMD8 with 0x1AB argument, which are the only
        commands that require a CRC */
    };

    // Go ahead and send the command even if the card isn't idle if this is the reset command
    if (!sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY) && commandCode != SDCARD_COMMAND_GO_IDLE_STATE)
        return 0xFF;

    spiBusRawTransfer(&sdcard.busdev, command, NULL, sizeof(command));

    /*
     * The card can take up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes to send the response, in the meantime
     * it'll transmit 0xFF filler bytes.
     */
    return sdcard_waitForNonIdleByte(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY);
}

static uint8_t sdcard_sendAppCommand(uint8_t commandCode, uint32_t commandArgument)
{
    sdcard_sendCommand(SDCARD_COMMAND_APP_CMD, 0);

    return sdcard_sendCommand(commandCode, commandArgument);
}

/**
 * Sends an IF_COND message to the card to check its version and validate its voltage requirements. Sets the global
 * sdCardVersion with the detected version (0, 1, or 2) and returns true if the card is compatible.
 */
static bool sdcard_validateInterfaceCondition(void)
{
    uint8_t ifCondReply[4];

    sdcard.version = 0;

    sdcard_select();

    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SEND_IF_COND, (SDCARD_VOLTAGE_ACCEPTED_2_7_to_3_6 << 8) | SDCARD_IF_COND_CHECK_PATTERN);

    // Don't deselect the card right away, because we'll want to read the rest of its reply if it's a V2 card

    if (status == (SDCARD_R1_STATUS_BIT_ILLEGAL_COMMAND | SDCARD_R1_STATUS_BIT_IDLE)) {
        // V1 cards don't support this command
        sdcard.version = 1;
    } else if (status == SDCARD_R1_STATUS_BIT_IDLE) {
        spiBusRawTransfer(&sdcard.busdev, NULL, ifCondReply, sizeof(ifCondReply));

        /*
         * We don't bother to validate the SDCard's operating voltage range since the spec requires it to accept our
         * 3.3V, but do check that it echoed back our check pattern properly.
         */
        if (ifCondReply[3] == SDCARD_IF_COND_CHECK_PATTERN) {
            sdcard.version = 2;
        }
    }

    sdcard_deselect();

    return sdcard.version > 0;
}

static bool sdcard_readOCRRegister(uint32_t *result)
{
    sdcard_select();

    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_OCR, 0);

    uint8_t response[4];

    spiBusRawTransfer(&sdcard.busdev, NULL, response, sizeof(response));

    if (status == 0) {
        sdcard_deselect();

        *result = (response[0] << 24) | (response[1] << 16) | (response[2] << 8) | response[3];

        return true;
    } else {
        sdcard_deselect();

        return false;
    }
}

typedef enum {
    SDCARD_RECEIVE_SUCCESS,
    SDCARD_RECEIVE_BLOCK_IN_PROGRESS,
    SDCARD_RECEIVE_ERROR
} sdcardReceiveBlockStatus_e;

/**
 * Attempt to receive a data block from the SD card.
 *
 * Return true on success, otherwise the card has not responded yet and you should retry later.
 */
static sdcardReceiveBlockStatus_e sdcard_receiveDataBlock(uint8_t *buffer, int count)
{
    uint8_t dataToken = sdcard_waitForNonIdleByte(8);

    if (dataToken == 0xFF) {
        return SDCARD_RECEIVE_BLOCK_IN_PROGRESS;
    }

    if (dataToken != SDCARD_SINGLE_BLOCK_READ_START_TOKEN) {
        return SDCARD_RECEIVE_ERROR;
    }

    spiBusRawTransfer(&sdcard.busdev, NULL, buffer, count);

    // Discard trailing CRC, we don't care
    spiBusTransferByte(&sdcard.busdev, 0xFF);
    spiBusTransferByte(&sdcard.busdev, 0xFF);

    return SDCARD_RECEIVE_SUCCESS;
}

static bool sdcard_sendDataBlockFinish(void)
{
#ifdef USE_HAL_DRIVER
    // Drain anything left in the Rx FIFO (we didn't read it during the write)
    // This is necessary here as when using msc there is timing issue
    while (CHECK_SPI_RX_DATA_AVAILABLE(sdcard.busdev.busdev_u.spi.instance)) {
        SPI_RX_DATA_REGISTER(sdcard.busdev.busdev_u.spi.instance);
    }
#endif

    // Send a dummy CRC
    spiBusTransferByte(&sdcard.busdev, 0x00);
    spiBusTransferByte(&sdcard.busdev, 0x00);

    uint8_t dataResponseToken = spiBusTransferByte(&sdcard.busdev, 0xFF);

    /*
     * Check if the card accepted the write (no CRC error / no address error)
     *
     * The lower 5 bits are structured as follows:
     * | 0 | Status  | 1 |
     * | 0 | x  x  x | 1 |
     *
     * Statuses:
     * 010 - Data accepted
     * 101 - CRC error
     * 110 - Write error
     */
    return (dataResponseToken & 0x1F) == 0x05;
}

/**
 * Begin sending a buffer of SDCARD_BLOCK_SIZE bytes to the SD card.
 */
static void sdcard_sendDataBlockBegin(const uint8_t *buffer, bool multiBlockWrite)
{
    // Card wants 8 dummy clock cycles between the write command's response and a data block beginning:
    spiBusTransferByte(&sdcard.busdev, 0xFF);

    spiBusTransferByte(&sdcard.busdev, multiBlockWrite ? SDCARD_MULTIPLE_BLOCK_WRITE_START_TOKEN : SDCARD_SINGLE_BLOCK_WRITE_START_TOKEN);

    if (sdcard.useDMAForTx) {
#if defined(USE_HAL_DRIVER)
        LL_DMA_InitTypeDef init;

        LL_DMA_StructInit(&init);

#if defined(STM32G4) || defined(STM32H7)
        init.PeriphRequest = dmaGetChannel(sdcard.dmaChannel);
#else
        init.Channel = dmaGetChannel(sdcard.dmaChannel);
#endif
        init.Mode = LL_DMA_MODE_NORMAL;
        init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;

        init.PeriphOrM2MSrcAddress = (uint32_t)&SPI_RX_DATA_REGISTER(sdcard.busdev.busdev_u.spi.instance);
        init.Priority = LL_DMA_PRIORITY_LOW;
        init.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
        init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;

        init.MemoryOrM2MDstAddress = (uint32_t)buffer;
        init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
        init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;

        init.NbData = SDCARD_BLOCK_SIZE;

        LL_DMA_DeInit(sdcard.dma->dma, sdcard.dma->stream);
        LL_DMA_Init(sdcard.dma->dma, sdcard.dma->stream, &init);

#if defined(STM32G4)
        LL_DMA_EnableChannel(sdcard.dma->dma, sdcard.dma->stream);
#else
        LL_DMA_EnableStream(sdcard.dma->dma, sdcard.dma->stream);
#endif

        LL_SPI_EnableDMAReq_TX(sdcard.busdev.busdev_u.spi.instance);

#else

        DMA_InitTypeDef init;

        DMA_StructInit(&init);
#ifdef STM32F4
        init.DMA_Channel = dmaGetChannel(sdcard.dmaChannel);
        init.DMA_Memory0BaseAddr = (uint32_t) buffer;
        init.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#else
        init.DMA_M2M = DMA_M2M_Disable;
        init.DMA_MemoryBaseAddr = (uint32_t) buffer;
        init.DMA_DIR = DMA_DIR_PeripheralDST;
#endif
        init.DMA_PeripheralBaseAddr = (uint32_t)&SPI_RX_DATA_REGISTER(sdcard.busdev.busdev_u.spi.instance);
        init.DMA_Priority = DMA_Priority_Low;
        init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

        init.DMA_MemoryInc = DMA_MemoryInc_Enable;
        init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

        init.DMA_BufferSize = SDCARD_BLOCK_SIZE;
        init.DMA_Mode = DMA_Mode_Normal;

        xDMA_DeInit(sdcard.dma->ref);
        xDMA_Init(sdcard.dma->ref, &init);

        xDMA_Cmd(sdcard.dma->ref, ENABLE);

        SPI_I2S_DMACmd(sdcard.busdev.busdev_u.spi.instance, SPI_I2S_DMAReq_Tx, ENABLE);
#endif
    } else {
        // Send the first chunk now
        spiBusRawTransfer(&sdcard.busdev, buffer, NULL, SDCARD_NON_DMA_CHUNK_SIZE);
    }
}

static bool sdcard_receiveCID(void)
{
    uint8_t cid[16];

    if (sdcard_receiveDataBlock(cid, sizeof(cid)) != SDCARD_RECEIVE_SUCCESS) {
        return false;
    }

    sdcard.metadata.manufacturerID = cid[0];
    sdcard.metadata.oemID = (cid[1] << 8) | cid[2];
    sdcard.metadata.productName[0] = cid[3];
    sdcard.metadata.productName[1] = cid[4];
    sdcard.metadata.productName[2] = cid[5];
    sdcard.metadata.productName[3] = cid[6];
    sdcard.metadata.productName[4] = cid[7];
    sdcard.metadata.productRevisionMajor = cid[8] >> 4;
    sdcard.metadata.productRevisionMinor = cid[8] & 0x0F;
    sdcard.metadata.productSerial = (cid[9] << 24) | (cid[10] << 16) | (cid[11] << 8) | cid[12];
    sdcard.metadata.productionYear = (((cid[13] & 0x0F) << 4) | (cid[14] >> 4)) + 2000;
    sdcard.metadata.productionMonth = cid[14] & 0x0F;

    return true;
}

static bool sdcard_fetchCSD(void)
{
    uint32_t readBlockLen, blockCount, blockCountMult;
    uint64_t capacityBytes;

    sdcard_select();

    /* The CSD command's data block should always arrive within 8 idle clock cycles (SD card spec). This is because
     * the information about card latency is stored in the CSD register itself, so we can't use that yet!
     */
    bool success =
        sdcard_sendCommand(SDCARD_COMMAND_SEND_CSD, 0) == 0
        && sdcard_receiveDataBlock((uint8_t*) &sdcard.csd, sizeof(sdcard.csd)) == SDCARD_RECEIVE_SUCCESS
        && SDCARD_GET_CSD_FIELD(sdcard.csd, 1, TRAILER) == 1;

    if (success) {
        switch (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSD_STRUCTURE_VER)) {
            case SDCARD_CSD_STRUCTURE_VERSION_1:
                // Block size in bytes (doesn't have to be 512)
                readBlockLen = 1 << SDCARD_GET_CSD_FIELD(sdcard.csd, 1, READ_BLOCK_LEN);
                blockCountMult = 1 << (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSIZE_MULT) + 2);
                blockCount = (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSIZE) + 1) * blockCountMult;

                // We could do this in 32 bits but it makes the 2GB case awkward
                capacityBytes = (uint64_t) blockCount * readBlockLen;

                // Re-express that capacity (max 2GB) in our standard 512-byte block size
                sdcard.metadata.numBlocks = capacityBytes / SDCARD_BLOCK_SIZE;
            break;
            case SDCARD_CSD_STRUCTURE_VERSION_2:
                sdcard.metadata.numBlocks = (SDCARD_GET_CSD_FIELD(sdcard.csd, 2, CSIZE) + 1) * 1024;
            break;
            default:
                success = false;
        }
    }

    sdcard_deselect();

    return success;
}

/**
 * Check if the SD Card has completed its startup sequence. Must be called with sdcard.state == SDCARD_STATE_INITIALIZATION.
 *
 * Returns true if the card has finished its init process.
 */
static bool sdcard_checkInitDone(void)
{
    sdcard_select();

    uint8_t status = sdcard_sendAppCommand(SDCARD_ACOMMAND_SEND_OP_COND, sdcard.version == 2 ? 1 << 30 /* We support high capacity cards */ : 0);

    sdcard_deselect();

    // When card init is complete, the idle bit in the response becomes zero.
    return status == 0x00;
}

void sdcardSpi_preInit(const sdcardConfig_t *config)
{
    spiPreinitRegister(config->chipSelectTag, IOCFG_IPU, 1);
}

/**
 * Begin the initialization process for the SD card. This must be called first before any other sdcard_ routine.
 */
static void sdcardSpi_init(const sdcardConfig_t *config, const spiPinConfig_t *spiConfig)
{
#ifndef USE_DMA_SPEC
    UNUSED(spiConfig);
#endif

    sdcard.enabled = config->mode;
    if (!sdcard.enabled) {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        return;
    }

    SPIDevice spiDevice = SPI_CFG_TO_DEV(config->device);

    spiBusSetInstance(&sdcard.busdev, spiInstanceByDevice(spiDevice));

    if (config->useDma) {
        dmaIdentifier_e dmaIdentifier = DMA_NONE;

#ifdef USE_DMA_SPEC
        const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_TX, config->device, spiConfig[spiDevice].txDmaopt);

        if (dmaChannelSpec) {
            dmaIdentifier = dmaGetIdentifier(dmaChannelSpec->ref);
            sdcard.dmaChannel = dmaChannelSpec->channel; // XXX STM32F3 doesn't have this
        }
#else
        dmaIdentifier = config->dmaIdentifier;
#endif

        if (dmaIdentifier) {
            sdcard.dma = dmaGetDescriptorByIdentifier(dmaIdentifier);
            dmaInit(dmaIdentifier, OWNER_SDCARD, 0);
            sdcard.useDMAForTx = true;
        } else {
            sdcard.useDMAForTx = false;
        }
    }

    IO_t chipSelectIO;
    if (config->chipSelectTag) {
        chipSelectIO = IOGetByTag(config->chipSelectTag);
        IOInit(chipSelectIO, OWNER_SDCARD_CS, 0);
        IOConfigGPIO(chipSelectIO, SPI_IO_CS_CFG);
    } else {
        chipSelectIO = IO_NONE;
    }
    sdcard.busdev.busdev_u.spi.csnPin = chipSelectIO;

    // Max frequency is initially 400kHz

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionInit(&sdcard.busdev, SDCARD_SPI_MODE, spiCalculateDivider(SDCARD_MAX_SPI_INIT_CLK_HZ));
#else
    spiSetDivisor(sdcard.busdev.busdev_u.spi.instance, spiCalculateDivider(SDCARD_MAX_SPI_INIT_CLK_HZ));
#endif

    // SDCard wants 1ms minimum delay after power is applied to it
    delay(1000);

    // Transmit at least 74 dummy clock cycles with CS high so the SD card can start up
    IOHi(sdcard.busdev.busdev_u.spi.csnPin);
    spiBusRawTransfer(&sdcard.busdev, NULL, NULL, SDCARD_INIT_NUM_DUMMY_BYTES);

    // Wait for that transmission to finish before we enable the SDCard, so it receives the required number of cycles:
    int time = 100000;
    while (spiBusIsBusBusy(&sdcard.busdev)) {
        if (time-- == 0) {
            sdcard.state = SDCARD_STATE_NOT_PRESENT;
            sdcard.failureCount++;
            return;
        }
    }

    sdcard.operationStartTime = millis();
    sdcard.state = SDCARD_STATE_RESET;
    sdcard.failureCount = 0;
}

static bool sdcard_setBlockLength(uint32_t blockLen)
{
    sdcard_select();

    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SET_BLOCKLEN, blockLen);

    sdcard_deselect();

    return status == 0;
}

/*
 * Returns true if the card is ready to accept read/write commands.
 */
static bool sdcard_isReady(void)
{
    return sdcard.state == SDCARD_STATE_READY || sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
}

/**
 * Send the stop-transmission token to complete a multi-block write.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - We're now waiting for that stop to complete, the card will enter
 *                                    the SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE state.
 *     SDCARD_OPERATION_SUCCESS     - The multi-block write finished immediately, the card will enter
 *                                    the SDCARD_READY state.
 *
 */
static sdcardOperationStatus_e sdcard_endWriteBlocks(void)
{
    sdcard.multiWriteBlocksRemain = 0;

    // 8 dummy clocks to guarantee N_WR clocks between the last card response and this token
    spiBusTransferByte(&sdcard.busdev, 0xFF);

    spiBusTransferByte(&sdcard.busdev, SDCARD_MULTIPLE_BLOCK_WRITE_STOP_TOKEN);

    // Card may choose to raise a busy (non-0xFF) signal after at most N_BR (1 byte) delay
    if (sdcard_waitForNonIdleByte(1) == 0xFF) {
        sdcard.state = SDCARD_STATE_READY;
        return SDCARD_OPERATION_SUCCESS;
    } else {
        sdcard.state = SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE;
        sdcard.operationStartTime = millis();

        return SDCARD_OPERATION_IN_PROGRESS;
    }
}

/**
 * Call periodically for the SD card to perform in-progress transfers.
 *
 * Returns true if the card is ready to accept commands.
 */
static bool sdcardSpi_poll(void)
{
    if (!sdcard.enabled) {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        return false;
    }

    uint8_t initStatus;
    bool sendComplete;

#ifdef SDCARD_PROFILING
    bool profilingComplete;
#endif

    doMore:
    switch (sdcard.state) {
        case SDCARD_STATE_RESET:
            sdcard_select();

            initStatus = sdcard_sendCommand(SDCARD_COMMAND_GO_IDLE_STATE, 0);

            sdcard_deselect();

            if (initStatus == SDCARD_R1_STATUS_BIT_IDLE) {
                // Check card voltage and version
                if (sdcard_validateInterfaceCondition()) {

                    sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS;
                    goto doMore;
                } else {
                    // Bad reply/voltage, we ought to refrain from accessing the card.
                    sdcard.state = SDCARD_STATE_NOT_PRESENT;
                }
            }
        break;

        case SDCARD_STATE_CARD_INIT_IN_PROGRESS:
            if (sdcard_checkInitDone()) {
                if (sdcard.version == 2) {
                    // Check for high capacity card
                    uint32_t ocr;

                    if (!sdcard_readOCRRegister(&ocr)) {
                        sdcard_reset();
                        goto doMore;
                    }

                    sdcard.highCapacity = (ocr & (1 << 30)) != 0;
                } else {
                    // Version 1 cards are always low-capacity
                    sdcard.highCapacity = false;
                }

                // Now fetch the CSD and CID registers
                if (sdcard_fetchCSD()) {
                    sdcard_select();

                    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SEND_CID, 0);

                    if (status == 0) {
                        // Keep the card selected to receive the response block
                        sdcard.state = SDCARD_STATE_INITIALIZATION_RECEIVE_CID;
                        goto doMore;
                    } else {
                        sdcard_deselect();

                        sdcard_reset();
                        goto doMore;
                    }
                }
            }
        break;
        case SDCARD_STATE_INITIALIZATION_RECEIVE_CID:
            if (sdcard_receiveCID()) {
                sdcard_deselect();

                /* The spec is a little iffy on what the default block size is for Standard Size cards (it can be changed on
                 * standard size cards) so let's just set it to 512 explicitly so we don't have a problem.
                 */
                if (!sdcard.highCapacity && !sdcard_setBlockLength(SDCARD_BLOCK_SIZE)) {
                    sdcard_reset();
                    goto doMore;
                }

                // Now we're done with init and we can switch to the full speed clock (<25MHz)

#ifdef USE_SPI_TRANSACTION
                spiBusTransactionInit(&sdcard.busdev, SDCARD_SPI_MODE, spiCalculateDivider(SDCARD_MAX_SPI_CLK_HZ));
#else
                spiSetDivisor(sdcard.busdev.busdev_u.spi.instance, spiCalculateDivider(SDCARD_MAX_SPI_CLK_HZ));
#endif

                sdcard.multiWriteBlocksRemain = 0;

                sdcard.state = SDCARD_STATE_READY;
                goto doMore;
            } // else keep waiting for the CID to arrive
        break;
        case SDCARD_STATE_SENDING_WRITE:
            // Have we finished sending the write yet?
            sendComplete = false;

#if defined(USE_HAL_DRIVER)
            if (sdcard.useDMAForTx && DMA_GET_FLAG_STATUS(sdcard.dma, DMA_IT_TCIF)) {
                //Clear both flags after transfer
                DMA_CLEAR_FLAG(sdcard.dma, DMA_IT_TCIF);
                DMA_CLEAR_FLAG(sdcard.dma, DMA_IT_HTIF);
                // Drain anything left in the Rx FIFO (we didn't read it during the write)
                while (CHECK_SPI_RX_DATA_AVAILABLE(sdcard.busdev.busdev_u.spi.instance)) {
                    SPI_RX_DATA_REGISTER(sdcard.busdev.busdev_u.spi.instance);
                }

                // Wait for the final bit to be transmitted
                while (spiBusIsBusBusy(&sdcard.busdev)) {
                }

                LL_SPI_DisableDMAReq_TX(sdcard.busdev.busdev_u.spi.instance);

                sendComplete = true;
            }
#else
#ifdef STM32F4
            if (sdcard.useDMAForTx && xDMA_GetFlagStatus(sdcard.dma->ref, sdcard.dma->completeFlag) == SET) {
                xDMA_ClearFlag(sdcard.dma->ref, sdcard.dma->completeFlag);
#else
            if (sdcard.useDMAForTx && DMA_GetFlagStatus(sdcard.dma->completeFlag) == SET) {
                DMA_ClearFlag(sdcard.dma->completeFlag);
#endif

                xDMA_Cmd(sdcard.dma->ref, DISABLE);

                // Drain anything left in the Rx FIFO (we didn't read it during the write)
                while (SPI_I2S_GetFlagStatus(sdcard.busdev.busdev_u.spi.instance, SPI_I2S_FLAG_RXNE) == SET) {
                    SPI_RX_DATA_REGISTER(sdcard.busdev.busdev_u.spi.instance);
                }

                // Wait for the final bit to be transmitted
                while (spiBusIsBusBusy(&sdcard.busdev)) {
                }

                SPI_I2S_DMACmd(sdcard.busdev.busdev_u.spi.instance, SPI_I2S_DMAReq_Tx, DISABLE);

                sendComplete = true;
            }
#endif
            if (!sdcard.useDMAForTx) {
                // Send another chunk
                spiBusRawTransfer(&sdcard.busdev, sdcard.pendingOperation.buffer + SDCARD_NON_DMA_CHUNK_SIZE * sdcard.pendingOperation.chunkIndex, NULL, SDCARD_NON_DMA_CHUNK_SIZE);

                sdcard.pendingOperation.chunkIndex++;

                sendComplete = sdcard.pendingOperation.chunkIndex == SDCARD_BLOCK_SIZE / SDCARD_NON_DMA_CHUNK_SIZE;
            }

            if (sendComplete) {
                // Finish up by sending the CRC and checking the SD-card's acceptance/rejectance
                if (sdcard_sendDataBlockFinish()) {
                    // The SD card is now busy committing that write to the card
                    sdcard.state = SDCARD_STATE_WAITING_FOR_WRITE;
                    sdcard.operationStartTime = millis();

                    // Since we've transmitted the buffer we can go ahead and tell the caller their operation is complete
                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, sdcard.pendingOperation.buffer, sdcard.pendingOperation.callbackData);
                    }
                } else {
                    /* Our write was rejected! This could be due to a bad address but we hope not to attempt that, so assume
                     * the card is broken and needs reset.
                     */
                    sdcard_reset();

                    // Announce write failure:
                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, NULL, sdcard.pendingOperation.callbackData);
                    }

                    goto doMore;
                }
            }
        break;
        case SDCARD_STATE_WAITING_FOR_WRITE:
            if (sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY)) {
#ifdef SDCARD_PROFILING
                profilingComplete = true;
#endif

                sdcard.failureCount = 0; // Assume the card is good if it can complete a write

                // Still more blocks left to write in a multi-block chain?
                if (sdcard.multiWriteBlocksRemain > 1) {
                    sdcard.multiWriteBlocksRemain--;
                    sdcard.multiWriteNextBlock++;
                    sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
                } else if (sdcard.multiWriteBlocksRemain == 1) {
                    // This function changes the sd card state for us whether immediately succesful or delayed:
                    if (sdcard_endWriteBlocks() == SDCARD_OPERATION_SUCCESS) {
                        sdcard_deselect();
                    } else {
#ifdef SDCARD_PROFILING
                        // Wait for the multi-block write to be terminated before finishing timing
                        profilingComplete = false;
#endif
                    }
                } else {
                    sdcard.state = SDCARD_STATE_READY;
                    sdcard_deselect();
                }

#ifdef SDCARD_PROFILING
                if (profilingComplete && sdcard.profiler) {
                    sdcard.profiler(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                }
#endif
            } else if (millis() > sdcard.operationStartTime + SDCARD_TIMEOUT_WRITE_MSEC) {
                /*
                 * The caller has already been told that their write has completed, so they will have discarded
                 * their buffer and have no hope of retrying the operation. But this should be very rare and it allows
                 * them to reuse their buffer milliseconds faster than they otherwise would.
                 */
                sdcard_reset();
                goto doMore;
            }
        break;
        case SDCARD_STATE_READING:
            switch (sdcard_receiveDataBlock(sdcard.pendingOperation.buffer, SDCARD_BLOCK_SIZE)) {
                case SDCARD_RECEIVE_SUCCESS:
                    sdcard_deselect();

                    sdcard.state = SDCARD_STATE_READY;
                    sdcard.failureCount = 0; // Assume the card is good if it can complete a read

#ifdef SDCARD_PROFILING
                    if (sdcard.profiler) {
                        sdcard.profiler(SDCARD_BLOCK_OPERATION_READ, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                    }
#endif

                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(
                            SDCARD_BLOCK_OPERATION_READ,
                            sdcard.pendingOperation.blockIndex,
                            sdcard.pendingOperation.buffer,
                            sdcard.pendingOperation.callbackData
                        );
                    }
                break;
                case SDCARD_RECEIVE_BLOCK_IN_PROGRESS:
                    if (millis() <= sdcard.operationStartTime + SDCARD_TIMEOUT_READ_MSEC) {
                        break; // Timeout not reached yet so keep waiting
                    }
                    // Timeout has expired, so fall through to convert to a fatal error
                    FALLTHROUGH;

                case SDCARD_RECEIVE_ERROR:
                    sdcard_deselect();

                    sdcard_reset();

                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(
                            SDCARD_BLOCK_OPERATION_READ,
                            sdcard.pendingOperation.blockIndex,
                            NULL,
                            sdcard.pendingOperation.callbackData
                        );
                    }

                    goto doMore;
                break;
            }
        break;
        case SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE:
            if (sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY)) {
                sdcard_deselect();

                sdcard.state = SDCARD_STATE_READY;

#ifdef SDCARD_PROFILING
                if (sdcard.profiler) {
                    sdcard.profiler(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                }
#endif
            } else if (millis() > sdcard.operationStartTime + SDCARD_TIMEOUT_WRITE_MSEC) {
                sdcard_reset();
                goto doMore;
            }
        break;
        case SDCARD_STATE_NOT_PRESENT:
        default:
            ;
    }

    // Is the card's initialization taking too long?
    if (sdcard.state >= SDCARD_STATE_RESET && sdcard.state < SDCARD_STATE_READY
            && millis() - sdcard.operationStartTime > SDCARD_TIMEOUT_INIT_MILLIS) {
        sdcard_reset();
    }

    return sdcard_isReady();
}

/**
 * Write the 512-byte block from the given buffer into the block with the given index.
 *
 * If the write does not complete immediately, your callback will be called later. If the write was successful, the
 * buffer pointer will be the same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - Your buffer is currently being transmitted to the card and your callback will be
 *                                    called later to report the completion. The buffer pointer must remain valid until
 *                                    that time.
 *     SDCARD_OPERATION_SUCCESS     - Your buffer has been transmitted to the card now.
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - Your write was rejected by the card, card will be reset
 */
static sdcardOperationStatus_e sdcardSpi_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    uint8_t status;

#ifdef SDCARD_PROFILING
    sdcard.pendingOperation.profileStartTime = micros();
#endif

    doMore:
    switch (sdcard.state) {
        case SDCARD_STATE_WRITING_MULTIPLE_BLOCKS:
            // Do we need to cancel the previous multi-block write?
            if (blockIndex != sdcard.multiWriteNextBlock) {
                if (sdcard_endWriteBlocks() == SDCARD_OPERATION_SUCCESS) {
                    // Now we've entered the ready state, we can try again
                    goto doMore;
                } else {
                    return SDCARD_OPERATION_BUSY;
                }
            }

            // We're continuing a multi-block write
        break;
        case SDCARD_STATE_READY:
            // We're not continuing a multi-block write so we need to send a single-block write command
            sdcard_select();

            // Standard size cards use byte addressing, high capacity cards use block addressing
            status = sdcard_sendCommand(SDCARD_COMMAND_WRITE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);

            if (status != 0) {
                sdcard_deselect();

                sdcard_reset();

                return SDCARD_OPERATION_FAILURE;
            }
        break;
        default:
            return SDCARD_OPERATION_BUSY;
    }

    sdcard_sendDataBlockBegin(buffer, sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS);

    sdcard.pendingOperation.buffer = buffer;
    sdcard.pendingOperation.blockIndex = blockIndex;
    sdcard.pendingOperation.callback = callback;
    sdcard.pendingOperation.callbackData = callbackData;
    sdcard.pendingOperation.chunkIndex = 1; // (for non-DMA transfers) we've sent chunk #0 already
    sdcard.state = SDCARD_STATE_SENDING_WRITE;

    return SDCARD_OPERATION_IN_PROGRESS;
}

/**
 * Begin writing a series of consecutive blocks beginning at the given block index. This will allow (but not require)
 * the SD card to pre-erase the number of blocks you specifiy, which can allow the writes to complete faster.
 *
 * Afterwards, just call sdcard_writeBlock() as normal to write those blocks consecutively.
 *
 * It's okay to abort the multi-block write at any time by writing to a non-consecutive address, or by performing a read.
 *
 * Returns:
 *     SDCARD_OPERATION_SUCCESS     - Multi-block write has been queued
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - A fatal error occured, card will be reset
 */
static sdcardOperationStatus_e sdcardSpi_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount)
{
    if (sdcard.state != SDCARD_STATE_READY) {
        if (sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS) {
            if (blockIndex == sdcard.multiWriteNextBlock) {
                // Assume that the caller wants to continue the multi-block write they already have in progress!
                return SDCARD_OPERATION_SUCCESS;
            } else if (sdcard_endWriteBlocks() != SDCARD_OPERATION_SUCCESS) {
                return SDCARD_OPERATION_BUSY;
            } // Else we've completed the previous multi-block write and can fall through to start the new one
        } else {
            return SDCARD_OPERATION_BUSY;
        }
    }

    sdcard_select();

    if (
        sdcard_sendAppCommand(SDCARD_ACOMMAND_SET_WR_BLOCK_ERASE_COUNT, blockCount) == 0
        && sdcard_sendCommand(SDCARD_COMMAND_WRITE_MULTIPLE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE) == 0
    ) {
        sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
        sdcard.multiWriteBlocksRemain = blockCount;
        sdcard.multiWriteNextBlock = blockIndex;

        // Leave the card selected
        return SDCARD_OPERATION_SUCCESS;
    } else {
        sdcard_deselect();

        sdcard_reset();

        return SDCARD_OPERATION_FAILURE;
    }
}

/**
 * Read the 512-byte block with the given index into the given 512-byte buffer.
 *
 * When the read completes, your callback will be called. If the read was successful, the buffer pointer will be the
 * same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * You must keep the pointer to the buffer valid until the operation completes!
 *
 * Returns:
 *     true - The operation was successfully queued for later completion, your callback will be called later
 *     false - The operation could not be started due to the card being busy (try again later).
 */
static bool sdcardSpi_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    if (sdcard.state != SDCARD_STATE_READY) {
        if (sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS) {
            if (sdcard_endWriteBlocks() != SDCARD_OPERATION_SUCCESS) {
                return false;
            }
        } else {
            return false;
        }
    }

#ifdef SDCARD_PROFILING
    sdcard.pendingOperation.profileStartTime = micros();
#endif

    sdcard_select();

    // Standard size cards use byte addressing, high capacity cards use block addressing
    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_SINGLE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);

    if (status == 0) {
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = blockIndex;
        sdcard.pendingOperation.callback = callback;
        sdcard.pendingOperation.callbackData = callbackData;

        sdcard.state = SDCARD_STATE_READING;

        sdcard.operationStartTime = millis();

        // Leave the card selected for the whole transaction

        return true;
    } else {
        sdcard_deselect();

        return false;
    }
}

/**
 * Returns true if the SD card has successfully completed its startup procedures.
 */
static bool sdcardSpi_isInitialized(void)
{
    return sdcard.state >= SDCARD_STATE_READY;
}

static const sdcardMetadata_t* sdcardSpi_getMetadata(void)
{
    return &sdcard.metadata;
}

#ifdef SDCARD_PROFILING

static void sdcardSpi_setProfilerCallback(sdcard_profilerCallback_c callback)
{
    sdcard.profiler = callback;
}

#endif

sdcardVTable_t sdcardSpiVTable = {
    sdcardSpi_preInit,
    sdcardSpi_init,
    sdcardSpi_readBlock,
    sdcardSpi_beginWriteBlocks,
    sdcardSpi_writeBlock,
    sdcardSpi_poll,
    sdcardSpi_isFunctional,
    sdcardSpi_isInitialized,
    sdcardSpi_getMetadata,
#ifdef SDCARD_PROFILING
    sdcardSpi_setProfilerCallback,
#endif
};

#endif
