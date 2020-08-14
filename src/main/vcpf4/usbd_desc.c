/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   This file provides the USBD descriptors and string formating method.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"
#include "usbd_conf.h"
#include "usb_regs.h"
#include "platform.h"
#include "build/version.h"

#include "pg/pg.h"
#include "pg/usb.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_DESC
  * @brief USBD descriptors module
  * @{
  */

/** @defgroup USBD_DESC_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_DESC_Private_Defines
  * @{
  */

#define USBD_VID                        0x0483

#ifdef USE_USB_CDC_HID
#define USBD_PID_COMPOSITE              0x3256
#endif
#define USBD_PID                        0x5740

/** @defgroup USB_String_Descriptors
  * @{
  */
#define USBD_LANGID_STRING              0x409
#define USBD_MANUFACTURER_STRING        FC_FIRMWARE_NAME

#define USBD_PRODUCT_HS_STRING          "STM32 Virtual ComPort in HS mode"
#define USBD_PRODUCT_FS_STRING          "STM32 Virtual ComPort in FS Mode"

#ifdef USBD_SERIALNUMBER_STRING
  #define USBD_SERIALNUMBER_HS_STRING          USBD_SERIALNUMBER_STRING
  #define USBD_SERIALNUMBER_FS_STRING          USBD_SERIALNUMBER_STRING
#else
  // start of STM32 flash
  #define USBD_SERIALNUMBER_HS_STRING     "0x8000000"
  #define USBD_SERIALNUMBER_FS_STRING     "0x8000000"
#endif /* USBD_SERIALNUMBER_STRING */

#define USBD_CONFIGURATION_HS_STRING    "VCP Config"
#define USBD_INTERFACE_HS_STRING        "VCP Interface"

#define USBD_CONFIGURATION_FS_STRING    "VCP Config"
#define USBD_INTERFACE_FS_STRING        "VCP Interface"
/**
  * @}
  */


/** @defgroup USBD_DESC_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_DESC_Private_Variables
  * @{
  */

USBD_DEVICE USR_desc =
{
  USBD_USR_DeviceDescriptor,
  USBD_USR_LangIDStrDescriptor,
  USBD_USR_ManufacturerStrDescriptor,
  USBD_USR_ProductStrDescriptor,
  USBD_USR_SerialStrDescriptor,
  USBD_USR_ConfigStrDescriptor,
  USBD_USR_InterfaceStrDescriptor,

};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
#ifdef USE_USB_CDC_HID
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_DeviceDesc_Composite[USB_SIZ_DEVICE_DESC] __ALIGN_END =
  {
    0x12,                                  /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,            /*bDescriptorType*/
    0x00, 0x02,                            /*bcdUSB */
    0xEF,                                  /*bDeviceClass*/
    0x02,                                  /*bDeviceSubClass*/
    0x01,                                  /*bDeviceProtocol*/
    USB_OTG_MAX_EP0_SIZE,                  /*bMaxPacketSize*/
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),    /*idVendor*/
    LOBYTE(USBD_PID_COMPOSITE),
    HIBYTE(USBD_PID_COMPOSITE),            /*idProduct*/
    0x00, 0x02,                            /*bcdDevice rel. 2.00*/
    USBD_IDX_MFC_STR,                      /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,                  /*Index of product string*/
    USBD_IDX_SERIAL_STR,                   /*Index of serial number string*/
    USBD_CFG_MAX_NUM                       /*bNumConfigurations*/
  } ; /* USB_DeviceDescriptor */
#endif
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END =
  {
    0x12,                                  /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,            /*bDescriptorType*/
    0x00, 0x02,                            /*bcdUSB */
    0x02,                                  /*bDeviceClass*/
    0x02,                                  /*bDeviceSubClass*/
    0x00,                                  /*bDeviceProtocol*/
    USB_OTG_MAX_EP0_SIZE,                  /*bMaxPacketSize*/
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),    /*idVendor*/
    LOBYTE(USBD_PID), HIBYTE(USBD_PID),    /*idProduct*/
    0x00, 0x02,                            /*bcdDevice rel. 2.00*/
    USBD_IDX_MFC_STR,                      /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,                  /*Index of product string*/
    USBD_IDX_SERIAL_STR,                   /*Index of serial number string*/
    USBD_CFG_MAX_NUM                       /*bNumConfigurations*/
  } ; /* USB_DeviceDescriptor */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END =
{
     USB_SIZ_STRING_LANGID,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING),
};
/**
  * @}
  */


/** @defgroup USBD_DESC_Private_FunctionPrototypes
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_DESC_Private_Functions
  * @{
  */

/**
* @brief  USBD_USR_DeviceDescriptor
*         return the device descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length)
{
    (void)speed;
#ifdef USE_USB_CDC_HID
    if (usbDevConfig()->type == COMPOSITE) {
        *length = sizeof(USBD_DeviceDesc_Composite);
        return USBD_DeviceDesc_Composite;
    }
#endif
    *length = sizeof(USBD_DeviceDesc);
    return USBD_DeviceDesc;
}

/**
* @brief  USBD_USR_LangIDStrDescriptor
*         return the LangID string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length)
{
    (void)speed;
  *length =  sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}


/**
* @brief  USBD_USR_ProductStrDescriptor
*         return the product string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ProductStrDescriptor( uint8_t speed , uint16_t *length)
{


  if (speed == 0)
    USBD_GetString ((uint8_t*)USBD_PRODUCT_HS_STRING, USBD_StrDesc, length);
  else
    USBD_GetString ((uint8_t*)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);

  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_ManufacturerStrDescriptor
*         return the manufacturer string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ManufacturerStrDescriptor( uint8_t speed , uint16_t *length)
{
    (void)speed;
  USBD_GetString ((uint8_t*)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_SerialStrDescriptor
*         return the serial number string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length)
{
  if (speed  == USB_OTG_SPEED_HIGH)
    USBD_GetString ((uint8_t*)USBD_SERIALNUMBER_HS_STRING, USBD_StrDesc, length);
  else
    USBD_GetString ((uint8_t*)USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);

  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_ConfigStrDescriptor
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length)
{
  if (speed  == USB_OTG_SPEED_HIGH)
    USBD_GetString ((uint8_t*)USBD_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
  else
    USBD_GetString ((uint8_t*)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length);

  return USBD_StrDesc;
}


/**
* @brief  USBD_USR_InterfaceStrDescriptor
*         return the interface string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length)
{
  if (speed == 0)
    USBD_GetString ((uint8_t*)USBD_INTERFACE_HS_STRING, USBD_StrDesc, length);
  else
    USBD_GetString ((uint8_t*)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);

  return USBD_StrDesc;
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
