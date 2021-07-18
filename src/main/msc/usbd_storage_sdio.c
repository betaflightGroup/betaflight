/**
  ******************************************************************************
  * @file    usbd_storage_template.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    09-November-2015
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_SDCARD

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sdcard.h"
#include "drivers/sdmmc_sdio.h"
#include "drivers/usb_msc.h"

#include "pg/pg.h"
#include "pg/sdcard.h"
#include "pg/sdio.h"

#ifdef USE_HAL_DRIVER
#include "usbd_msc.h"
#else
#include "usbd_msc_mem.h"
#include "usbd_msc_core.h"
#endif

#include "usbd_storage.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB NVIC Priority has to be lower than both DMA and SDIO priority,
 * otherwise SDIO won't be able to preempt USB.
 */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

static int8_t STORAGE_Init (uint8_t lun);

#ifdef USE_HAL_DRIVER
static int8_t STORAGE_GetCapacity (uint8_t lun,
                           uint32_t *block_num,
                           uint16_t *block_size);
#else
static int8_t STORAGE_GetCapacity (uint8_t lun,
                           uint32_t *block_num,
                           uint32_t *block_size);
#endif

static int8_t  STORAGE_IsReady (uint8_t lun);

static int8_t  STORAGE_IsWriteProtected (uint8_t lun);

static int8_t STORAGE_Read (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len);

static int8_t STORAGE_Write (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len);

static int8_t STORAGE_GetMaxLun (void);

/* USB Mass storage Standard Inquiry Data */
static uint8_t  STORAGE_Inquirydata[] = {//36

  /* LUN 0 */
    0x00,
    0x80,
    0x02,
    0x02,
#ifdef USE_HAL_DRIVER
    (STANDARD_INQUIRY_DATA_LEN - 5),
#else
    (USBD_STD_INQUIRY_LENGTH - 5),
#endif
    0x00,
    0x00,
    0x00,
    'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
    'P', 'r', 'o', 'd', 'u', 't', ' ', ' ', /* Product      : 16 Bytes */
    ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
    '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
};

#ifdef USE_HAL_DRIVER
USBD_StorageTypeDef USBD_MSC_MICRO_SDIO_fops =
{
    STORAGE_Init,
    STORAGE_GetCapacity,
    STORAGE_IsReady,
    STORAGE_IsWriteProtected,
    STORAGE_Read,
    STORAGE_Write,
    STORAGE_GetMaxLun,
    (int8_t*)STORAGE_Inquirydata,
};
#else
USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SDIO_fops =
{
    STORAGE_Init,
    STORAGE_GetCapacity,
    STORAGE_IsReady,
    STORAGE_IsWriteProtected,
    STORAGE_Read,
    STORAGE_Write,
    STORAGE_GetMaxLun,
    (int8_t*)STORAGE_Inquirydata,
};
#endif

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t STORAGE_Init (uint8_t lun)
{
    //Initialize SD_DET
    const IO_t sd_det = IOGetByTag(sdcardConfig()->cardDetectTag);
    IOInit(sd_det, OWNER_SDCARD_DETECT, 0);
    IOConfigGPIO(sd_det, IOCFG_IPU);

    UNUSED(lun);
    LED0_OFF;

#ifdef USE_DMA_SPEC
#if defined(STM32H7) // H7 uses IDMA
    SD_Initialize_LL(0);
#else
    const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, sdioConfig()->dmaopt);

    if (!dmaChannelSpec) {
        return 1;
    }

    SD_Initialize_LL((DMA_ARCH_TYPE *)dmaChannelSpec->ref);
#endif
#else
    SD_Initialize_LL(SDCARD_SDIO_DMA_OPT);
#endif

    if (!sdcard_isInserted()) {
        return 1;
    }
    if (SD_Init() != SD_OK) {
        return 1;
    }

    mscSetActive();

    return 0;
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#ifdef USE_HAL_DRIVER
static int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint16_t *block_size)
#else
static int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint32_t *block_size)
#endif
{
    UNUSED(lun);
    if (!sdcard_isInserted()) {
        return -1;
    }
    SD_GetCardInfo();

    *block_num = SD_CardInfo.CardCapacity;
    *block_size = 512;
    return (0);
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t  STORAGE_IsReady (uint8_t lun)
{
    UNUSED(lun);
    int8_t ret = -1;
    if (SD_GetState() == true && sdcard_isInserted()) {
        ret = 0;
    }
    return ret;
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t  STORAGE_IsWriteProtected (uint8_t lun)
{
    UNUSED(lun);
    return  0;
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t STORAGE_Read (uint8_t lun,
                 uint8_t *buf,
                 uint32_t blk_addr,
                 uint16_t blk_len)
{
    UNUSED(lun);
    if (!sdcard_isInserted()) {
        return -1;
    }
    //buf should be 32bit aligned, but usually is so we don't do byte alignment
    if (SD_ReadBlocks_DMA(blk_addr, (uint32_t*) buf, 512, blk_len) == 0) {
        while (SD_CheckRead());
        while(SD_GetState() == false);
        mscSetActive();
        return 0;
    }
    return -1;
}
/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t STORAGE_Write (uint8_t lun,
                  uint8_t *buf,
                  uint32_t blk_addr,
                  uint16_t blk_len)
{
    UNUSED(lun);
    if (!sdcard_isInserted()) {
        return -1;
    }
    //buf should be 32bit aligned, but usually is so we don't do byte alignment
    if (SD_WriteBlocks_DMA(blk_addr, (uint32_t*) buf, 512, blk_len) == 0) {
        while (SD_CheckWrite());
        while(SD_GetState() == false);
        mscSetActive();
        return 0;
    }
    return -1;
}
/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t STORAGE_GetMaxLun (void)
{
    return (STORAGE_LUN_NBR - 1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#endif // USE_SDCARD
