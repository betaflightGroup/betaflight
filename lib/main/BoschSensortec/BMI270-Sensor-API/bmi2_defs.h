/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bmi2_defs.h
* @date       2020-11-04
* @version    v2.63.1
*
*/

#ifndef BMI2_DEFS_H_
#define BMI2_DEFS_H_

/******************************************************************************/
/*! @name       Header includes                           */
/******************************************************************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/******************************************************************************/
/*! @name       Common macros                         */
/******************************************************************************/
#ifdef __KERNEL__
#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)    S8_C(x)
#define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)   S16_C(x)
#define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)   S32_C(x)
#define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)   S64_C(x)
#define UINT64_C(x)  U64_C(x)
#endif
#endif

/*! @name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

/******************************************************************************/
/*! @name        General Macro Definitions                */
/******************************************************************************/
/*! @name  Utility macros */
#define BMI2_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     ((data << bitname##_POS) & bitname##_MASK))

#define BMI2_GET_BITS(reg_data, bitname) \
    ((reg_data & (bitname##_MASK)) >> \
     (bitname##_POS))

#define BMI2_SET_BIT_POS0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     (data & bitname##_MASK))

#define BMI2_GET_BIT_POS0(reg_data, bitname)      (reg_data & (bitname##_MASK))
#define BMI2_SET_BIT_VAL0(reg_data, bitname)      (reg_data & ~(bitname##_MASK))

/*! @name For getting LSB and MSB */
#define BMI2_GET_LSB(var)                         (uint8_t)(var & BMI2_SET_LOW_BYTE)
#define BMI2_GET_MSB(var)                         (uint8_t)((var & BMI2_SET_HIGH_BYTE) >> 8)

#ifndef BMI2_INTF_RETURN_TYPE
#define BMI2_INTF_RETURN_TYPE                     int8_t
#endif

/*! @name For defining absolute values */
#define BMI2_ABS(a)                               ((a) > 0 ? (a) : -(a))

/*! @name LSB and MSB mask definitions */
#define BMI2_SET_LOW_BYTE                         UINT16_C(0x00FF)
#define BMI2_SET_HIGH_BYTE                        UINT16_C(0xFF00)
#define BMI2_SET_LOW_NIBBLE                       UINT8_C(0x0F)

/*! @name For enable and disable */
#define BMI2_ENABLE                               UINT8_C(1)
#define BMI2_DISABLE                              UINT8_C(0)

/*! @name To define TRUE or FALSE */
#define BMI2_TRUE                                 UINT8_C(1)
#define BMI2_FALSE                                UINT8_C(0)

/*! @name To define sensor interface success code */
#define BMI2_INTF_RET_SUCCESS                     INT8_C(0)

/*! @name To define success code */
#define BMI2_OK                                   INT8_C(0)

/*! @name To define error codes */
#define BMI2_E_NULL_PTR                           INT8_C(-1)
#define BMI2_E_COM_FAIL                           INT8_C(-2)
#define BMI2_E_DEV_NOT_FOUND                      INT8_C(-3)
#define BMI2_E_OUT_OF_RANGE                       INT8_C(-4)
#define BMI2_E_ACC_INVALID_CFG                    INT8_C(-5)
#define BMI2_E_GYRO_INVALID_CFG                   INT8_C(-6)
#define BMI2_E_ACC_GYR_INVALID_CFG                INT8_C(-7)
#define BMI2_E_INVALID_SENSOR                     INT8_C(-8)
#define BMI2_E_CONFIG_LOAD                        INT8_C(-9)
#define BMI2_E_INVALID_PAGE                       INT8_C(-10)
#define BMI2_E_INVALID_FEAT_BIT                   INT8_C(-11)
#define BMI2_E_INVALID_INT_PIN                    INT8_C(-12)
#define BMI2_E_SET_APS_FAIL                       INT8_C(-13)
#define BMI2_E_AUX_INVALID_CFG                    INT8_C(-14)
#define BMI2_E_AUX_BUSY                           INT8_C(-15)
#define BMI2_E_SELF_TEST_FAIL                     INT8_C(-16)
#define BMI2_E_REMAP_ERROR                        INT8_C(-17)
#define BMI2_E_GYR_USER_GAIN_UPD_FAIL             INT8_C(-18)
#define BMI2_E_SELF_TEST_NOT_DONE                 INT8_C(-19)
#define BMI2_E_INVALID_INPUT                      INT8_C(-20)
#define BMI2_E_INVALID_STATUS                     INT8_C(-21)
#define BMI2_E_CRT_ERROR                          INT8_C(-22)
#define BMI2_E_ST_ALREADY_RUNNING                 INT8_C(-23)
#define BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT        INT8_C(-24)
#define BMI2_E_DL_ERROR                           INT8_C(-25)
#define BMI2_E_PRECON_ERROR                       INT8_C(-26)
#define BMI2_E_ABORT_ERROR                        INT8_C(-27)
#define BMI2_E_GYRO_SELF_TEST_ERROR               INT8_C(-28)
#define BMI2_E_GYRO_SELF_TEST_TIMEOUT             INT8_C(-29)
#define BMI2_E_WRITE_CYCLE_ONGOING                INT8_C(-30)
#define BMI2_E_WRITE_CYCLE_TIMEOUT                INT8_C(-31)
#define BMI2_E_ST_NOT_RUNING                      INT8_C(-32)
#define BMI2_E_DATA_RDY_INT_FAILED                INT8_C(-33)
#define BMI2_E_INVALID_FOC_POSITION               INT8_C(-34)

/*! @name To define warnings for FIFO activity */
#define BMI2_W_FIFO_EMPTY                         INT8_C(1)
#define BMI2_W_PARTIAL_READ                       INT8_C(2)

/*! @name Bit wise to define information */
#define BMI2_I_MIN_VALUE                          UINT8_C(1)
#define BMI2_I_MAX_VALUE                          UINT8_C(2)

/*! @name BMI2 register addresses */
#define BMI2_CHIP_ID_ADDR                         UINT8_C(0x00)
#define BMI2_STATUS_ADDR                          UINT8_C(0x03)
#define BMI2_AUX_X_LSB_ADDR                       UINT8_C(0x04)
#define BMI2_ACC_X_LSB_ADDR                       UINT8_C(0x0C)
#define BMI2_GYR_X_LSB_ADDR                       UINT8_C(0x12)
#define BMI2_EVENT_ADDR                           UINT8_C(0x1B)
#define BMI2_INT_STATUS_0_ADDR                    UINT8_C(0x1C)
#define BMI2_INT_STATUS_1_ADDR                    UINT8_C(0x1D)
#define BMI2_SC_OUT_0_ADDR                        UINT8_C(0x1E)
#define BMI2_SYNC_COMMAND_ADDR                    UINT8_C(0x1E)
#define BMI2_GYR_CAS_GPIO0_ADDR                   UINT8_C(0x1E)
#define BMI2_INTERNAL_STATUS_ADDR                 UINT8_C(0x21)
#define BMI2_FIFO_LENGTH_0_ADDR                   UINT8_C(0X24)
#define BMI2_FIFO_DATA_ADDR                       UINT8_C(0X26)
#define BMI2_FEAT_PAGE_ADDR                       UINT8_C(0x2F)
#define BMI2_FEATURES_REG_ADDR                    UINT8_C(0x30)
#define BMI2_ACC_CONF_ADDR                        UINT8_C(0x40)
#define BMI2_GYR_CONF_ADDR                        UINT8_C(0x42)
#define BMI2_AUX_CONF_ADDR                        UINT8_C(0x44)
#define BMI2_FIFO_DOWNS_ADDR                      UINT8_C(0X45)
#define BMI2_FIFO_WTM_0_ADDR                      UINT8_C(0X46)
#define BMI2_FIFO_WTM_1_ADDR                      UINT8_C(0X47)
#define BMI2_FIFO_CONFIG_0_ADDR                   UINT8_C(0X48)
#define BMI2_FIFO_CONFIG_1_ADDR                   UINT8_C(0X49)
#define BMI2_AUX_DEV_ID_ADDR                      UINT8_C(0x4B)
#define BMI2_AUX_IF_CONF_ADDR                     UINT8_C(0x4C)
#define BMI2_AUX_RD_ADDR                          UINT8_C(0x4D)
#define BMI2_AUX_WR_ADDR                          UINT8_C(0x4E)
#define BMI2_AUX_WR_DATA_ADDR                     UINT8_C(0x4F)
#define BMI2_INT1_IO_CTRL_ADDR                    UINT8_C(0x53)
#define BMI2_INT2_IO_CTRL_ADDR                    UINT8_C(0x54)
#define BMI2_INT_LATCH_ADDR                       UINT8_C(0x55)
#define BMI2_INT1_MAP_FEAT_ADDR                   UINT8_C(0x56)
#define BMI2_INT2_MAP_FEAT_ADDR                   UINT8_C(0x57)
#define BMI2_INT_MAP_DATA_ADDR                    UINT8_C(0x58)
#define BMI2_INIT_CTRL_ADDR                       UINT8_C(0x59)
#define BMI2_INIT_ADDR_0                          UINT8_C(0x5B)
#define BMI2_INIT_ADDR_1                          UINT8_C(0x5C)
#define BMI2_INIT_DATA_ADDR                       UINT8_C(0x5E)
#define BMI2_AUX_IF_TRIM                          UINT8_C(0x68)
#define BMI2_GYR_CRT_CONF_ADDR                    UINT8_C(0X69)
#define BMI2_NVM_CONF_ADDR                        UINT8_C(0x6A)
#define BMI2_IF_CONF_ADDR                         UINT8_C(0X6B)
#define BMI2_ACC_SELF_TEST_ADDR                   UINT8_C(0X6D)
#define BMI2_GYR_SELF_TEST_AXES_ADDR              UINT8_C(0x6E)
#define BMI2_SELF_TEST_MEMS_ADDR                  UINT8_C(0X6F)
#define BMI2_NV_CONF_ADDR                         UINT8_C(0x70)
#define BMI2_ACC_OFF_COMP_0_ADDR                  UINT8_C(0X71)
#define BMI2_GYR_OFF_COMP_3_ADDR                  UINT8_C(0X74)
#define BMI2_GYR_OFF_COMP_6_ADDR                  UINT8_C(0X77)
#define BMI2_GYR_USR_GAIN_0_ADDR                  UINT8_C(0X78)
#define BMI2_PWR_CONF_ADDR                        UINT8_C(0x7C)
#define BMI2_PWR_CTRL_ADDR                        UINT8_C(0x7D)
#define BMI2_CMD_REG_ADDR                         UINT8_C(0x7E)

/*! @name BMI2 I2C address */
#define BMI2_I2C_PRIM_ADDR                        UINT8_C(0x68)
#define BMI2_I2C_SEC_ADDR                         UINT8_C(0x69)

/*! @name BMI2 Commands */
#define BMI2_G_TRIGGER_CMD                        UINT8_C(0x02)
#define BMI2_USR_GAIN_CMD                         UINT8_C(0x03)
#define BMI2_NVM_PROG_CMD                         UINT8_C(0xA0)
#define BMI2_SOFT_RESET_CMD                       UINT8_C(0xB6)
#define BMI2_FIFO_FLUSH_CMD                       UINT8_C(0xB0)

/*! @name BMI2 sensor data bytes */

#define BMI2_ACC_GYR_NUM_BYTES                    UINT8_C(6)
#define BMI2_AUX_NUM_BYTES                        UINT8_C(8)
#define BMI2_CRT_CONFIG_FILE_SIZE                 UINT16_C(2048)
#define BMI2_FEAT_SIZE_IN_BYTES                   UINT8_C(16)
#define BMI2_ACC_CONFIG_LENGTH                    UINT8_C(2)

/*! @name BMI2 configuration load status */
#define BMI2_CONFIG_LOAD_SUCCESS                  UINT8_C(1)

/*! @name To define BMI2 pages */
#define BMI2_PAGE_0                               UINT8_C(0)
#define BMI2_PAGE_1                               UINT8_C(1)
#define BMI2_PAGE_2                               UINT8_C(2)
#define BMI2_PAGE_3                               UINT8_C(3)
#define BMI2_PAGE_4                               UINT8_C(4)
#define BMI2_PAGE_5                               UINT8_C(5)
#define BMI2_PAGE_6                               UINT8_C(6)
#define BMI2_PAGE_7                               UINT8_C(7)

/*! @name Array Parameter DefinItions */
#define BMI2_SENSOR_TIME_LSB_BYTE                 UINT8_C(0)
#define BMI2_SENSOR_TIME_XLSB_BYTE                UINT8_C(1)
#define BMI2_SENSOR_TIME_MSB_BYTE                 UINT8_C(2)

/*! @name Mask definitions for Gyro CRT  */
#define BMI2_GYR_RDY_FOR_DL_MASK                  UINT8_C(0x08)
#define BMI2_GYR_CRT_RUNNING_MASK                 UINT8_C(0x04)

/*! @name mask definition for status register */
#define BMI2_AUX_BUSY_MASK                        UINT8_C(0x04)
#define BMI2_CMD_RDY_MASK                         UINT8_C(0x10)
#define BMI2_DRDY_AUX_MASK                        UINT8_C(0x20)
#define BMI2_DRDY_GYR_MASK                        UINT8_C(0x40)
#define BMI2_DRDY_ACC_MASK                        UINT8_C(0x80)

/*! @name Mask definitions for SPI read/write address */
#define BMI2_SPI_RD_MASK                          UINT8_C(0x80)
#define BMI2_SPI_WR_MASK                          UINT8_C(0x7F)

/*! @name Mask definitions for power configuration register */
#define BMI2_ADV_POW_EN_MASK                      UINT8_C(0x01)

/*! @name Mask definitions for initialization control register */
#define BMI2_CONF_LOAD_EN_MASK                    UINT8_C(0x01)

/*! @name Mask definitions for power control register */
#define BMI2_AUX_EN_MASK                          UINT8_C(0x01)
#define BMI2_GYR_EN_MASK                          UINT8_C(0x02)
#define BMI2_ACC_EN_MASK                          UINT8_C(0x04)
#define BMI2_TEMP_EN_MASK                         UINT8_C(0x08)

/*! @name Mask definitions for sensor event flags */
#define BMI2_EVENT_FLAG_MASK                      UINT8_C(0x1C)

/*! @name Mask definitions to switch page */
#define BMI2_SWITCH_PAGE_EN_MASK                  UINT8_C(0x07)

/*! @name Mask definitions of NVM register */
#define BMI2_NV_ACC_OFFSET_MASK                   UINT8_C(0x08)

/*! @name Mask definition for config version */
#define BMI2_CONFIG_MAJOR_MASK                    UINT16_C(0x3C0)
#define BMI2_CONFIG_MINOR_MASK                    UINT8_C(0x3F)

/*! @name mask and bit position for activity recognition settings */
#define BMI2_ACT_RECG_POST_PROS_EN_DIS_MASK       UINT8_C(0x01)
#define BMI2_ACT_RECG_BUFF_SIZE_MASK              UINT8_C(0x0F)
#define BMI2_ACT_RECG_MIN_SEG_CONF_MASK           UINT8_C(0x0F)

/*! @name mask and bit position for activity recognition hc settings */
#define BMI2_HC_ACT_RECG_SEGMENT_SIZE_MASK        UINT8_C(0x03)
#define BMI2_HC_ACT_RECG_PP_EN_MASK               UINT8_C(0x01)
#define BMI2_HC_ACT_RECG_MIN_GDI_THRES_MASK       UINT16_C(0xFFFF)
#define BMI2_HC_ACT_RECG_MAX_GDI_THRES_MASK       UINT16_C(0xFFFF)
#define BMI2_HC_ACT_RECG_BUF_SIZE_MASK            UINT16_C(0xFFFF)
#define BMI2_HC_ACT_RECG_MIN_SEG_CONF_MASK        UINT16_C(0xFFFF)

#define BMI2_GYRO_CROSS_AXES_SENSE_MASK           UINT8_C(0x7F)
#define BMI2_GYRO_CROSS_AXES_SENSE_SIGN_BIT_MASK  UINT8_C(0x40)

/*! @name Bit position definitions for Gyro CRT */
#define BMI2_GYR_RDY_FOR_DL_POS                   UINT8_C(0x03)
#define BMI2_GYR_CRT_RUNNING_POS                  UINT8_C(0x02)

/*! @name Bit position for status register*/
#define BMI2_AUX_BUSY_POS                         UINT8_C(0x02)
#define BMI2_CMD_RDY_POS                          UINT8_C(0x04)
#define BMI2_DRDY_AUX_POS                         UINT8_C(0x05)
#define BMI2_DRDY_GYR_POS                         UINT8_C(0x06)
#define BMI2_DRDY_ACC_POS                         UINT8_C(0x07)

/*! @name Bit position definitions for power control register */
#define BMI2_GYR_EN_POS                           UINT8_C(0x01)
#define BMI2_ACC_EN_POS                           UINT8_C(0x02)
#define BMI2_TEMP_EN_POS                          UINT8_C(0x03)

/*! @name Bit position definitions for sensor event flags */
#define BMI2_EVENT_FLAG_POS                       UINT8_C(0x02)

/*! @name Bit position definitions of NVM register */
#define BMI2_NV_ACC_OFFSET_POS                    UINT8_C(0x03)

/*! @name Bit position for major version from config */
#define BMI2_CONFIG_MAJOR_POS                     UINT8_C(0x06)

/*! @name Accelerometer and Gyroscope Filter/Noise performance modes */
/* Power optimized mode */
#define BMI2_POWER_OPT_MODE                       UINT8_C(0)

/* Performance optimized  */
#define BMI2_PERF_OPT_MODE                        UINT8_C(1)

/*! @name index for config major minor information */
#define BMI2_CONFIG_INFO_LOWER                    UINT8_C(52)
#define BMI2_CONFIG_INFO_HIGHER                   UINT8_C(53)

/*! @name Sensor status */
#define BMI2_DRDY_ACC                             UINT8_C(0x80)
#define BMI2_DRDY_GYR                             UINT8_C(0x40)
#define BMI2_DRDY_AUX                             UINT8_C(0x20)
#define BMI2_CMD_RDY                              UINT8_C(0x10)
#define BMI2_AUX_BUSY                             UINT8_C(0x04)

/*! @name Macro to define accelerometer configuration value for FOC */
#define BMI2_FOC_ACC_CONF_VAL                     UINT8_C(0xB7)

/*! @name Macro to define gyroscope configuration value for FOC */
#define BMI2_FOC_GYR_CONF_VAL                     UINT8_C(0xB6)

/*! @name Macro to define X Y and Z axis for an array */
#define BMI2_X_AXIS                               UINT8_C(0)
#define BMI2_Y_AXIS                               UINT8_C(1)
#define BMI2_Z_AXIS                               UINT8_C(2)

/******************************************************************************/
/*! @name        Sensor Macro Definitions                 */
/******************************************************************************/
/*!  @name Macros to define BMI2 sensor/feature types */
#define BMI2_ACCEL                                UINT8_C(0)
#define BMI2_GYRO                                 UINT8_C(1)
#define BMI2_AUX                                  UINT8_C(2)
#define BMI2_SIG_MOTION                           UINT8_C(3)
#define BMI2_ANY_MOTION                           UINT8_C(4)
#define BMI2_NO_MOTION                            UINT8_C(5)
#define BMI2_STEP_DETECTOR                        UINT8_C(6)
#define BMI2_STEP_COUNTER                         UINT8_C(7)
#define BMI2_STEP_ACTIVITY                        UINT8_C(8)
#define BMI2_GYRO_GAIN_UPDATE                     UINT8_C(9)
#define BMI2_TILT                                 UINT8_C(10)
#define BMI2_UP_HOLD_TO_WAKE                      UINT8_C(11)
#define BMI2_GLANCE_DETECTOR                      UINT8_C(12)
#define BMI2_WAKE_UP                              UINT8_C(13)
#define BMI2_ORIENTATION                          UINT8_C(14)
#define BMI2_HIGH_G                               UINT8_C(15)
#define BMI2_LOW_G                                UINT8_C(16)
#define BMI2_FLAT                                 UINT8_C(17)
#define BMI2_EXT_SENS_SYNC                        UINT8_C(18)
#define BMI2_WRIST_GESTURE                        UINT8_C(19)
#define BMI2_WRIST_WEAR_WAKE_UP                   UINT8_C(20)
#define BMI2_WRIST_WEAR_WAKE_UP_WH                UINT8_C(21)
#define BMI2_WRIST_GESTURE_WH                     UINT8_C(22)
#define BMI2_PRIMARY_OIS                          UINT8_C(23)
#define BMI2_FREE_FALL_DET                        UINT8_C(24)
#define BMI2_SINGLE_TAP                           UINT8_C(25)
#define BMI2_DOUBLE_TAP                           UINT8_C(26)
#define BMI2_TRIPLE_TAP                           UINT8_C(27)
#define BMI2_TAP                                  UINT8_C(28)

/* Non virtual sensor features */
#define BMI2_STEP_COUNTER_PARAMS                  UINT8_C(29)
#define BMI2_TAP_DETECTOR_1                       UINT8_C(30)
#define BMI2_TAP_DETECTOR_2                       UINT8_C(31)
#define BMI2_TEMP                                 UINT8_C(32)
#define BMI2_ACCEL_SELF_TEST                      UINT8_C(33)
#define BMI2_GYRO_SELF_OFF                        UINT8_C(34)
#define BMI2_ACTIVITY_RECOGNITION                 UINT8_C(35)
#define BMI2_MAX_BURST_LEN                        UINT8_C(36)
#define BMI2_SENS_MAX_NUM                         UINT8_C(37)
#define BMI2_AXIS_MAP                             UINT8_C(38)
#define BMI2_NVM_STATUS                           UINT8_C(39)
#define BMI2_VFRM_STATUS                          UINT8_C(40)
#define BMI2_GYRO_CROSS_SENSE                     UINT8_C(41)
#define BMI2_CRT_GYRO_SELF_TEST                   UINT8_C(42)
#define BMI2_ABORT_CRT_GYRO_SELF_TEST             UINT8_C(43)
#define BMI2_NVM_PROG_PREP                        UINT8_C(44)
#define BMI2_ACTIVITY_RECOGNITION_SETTINGS        UINT8_C(45)
#define BMI2_OIS_OUTPUT                           UINT8_C(46)
#define BMI2_CONFIG_ID                            UINT8_C(47)

/*! @name Bit wise for selecting BMI2 sensors/features */
#define BMI2_ACCEL_SENS_SEL                       (1)
#define BMI2_GYRO_SENS_SEL                        (1 << BMI2_GYRO)
#define BMI2_AUX_SENS_SEL                         (1 << BMI2_AUX)
#define BMI2_TEMP_SENS_SEL                        ((uint64_t)1 << BMI2_TEMP)
#define BMI2_ANY_MOT_SEL                          (1 << BMI2_ANY_MOTION)
#define BMI2_NO_MOT_SEL                           (1 << BMI2_NO_MOTION)
#define BMI2_TILT_SEL                             (1 << BMI2_TILT)
#define BMI2_ORIENT_SEL                           (1 << BMI2_ORIENTATION)
#define BMI2_SIG_MOTION_SEL                       (1 << BMI2_SIG_MOTION)
#define BMI2_STEP_DETECT_SEL                      (1 << BMI2_STEP_DETECTOR)
#define BMI2_STEP_COUNT_SEL                       (1 << BMI2_STEP_COUNTER)
#define BMI2_STEP_ACT_SEL                         (1 << BMI2_STEP_ACTIVITY)
#define BMI2_GYRO_GAIN_UPDATE_SEL                 (1 << BMI2_GYRO_GAIN_UPDATE)
#define BMI2_UP_HOLD_TO_WAKE_SEL                  (1 << BMI2_UP_HOLD_TO_WAKE)
#define BMI2_GLANCE_DET_SEL                       (1 << BMI2_GLANCE_DETECTOR)
#define BMI2_WAKE_UP_SEL                          (1 << BMI2_WAKE_UP)
#define BMI2_TAP_SEL                              (1 << BMI2_TAP)
#define BMI2_HIGH_G_SEL                           (1 << BMI2_HIGH_G)
#define BMI2_LOW_G_SEL                            (1 << BMI2_LOW_G)
#define BMI2_FLAT_SEL                             (1 << BMI2_FLAT)
#define BMI2_EXT_SENS_SEL                         (1 << BMI2_EXT_SENS_SYNC)
#define BMI2_SINGLE_TAP_SEL                       (1 << BMI2_SINGLE_TAP)
#define BMI2_DOUBLE_TAP_SEL                       (1 << BMI2_DOUBLE_TAP)
#define BMI2_TRIPLE_TAP_SEL                       (1 << BMI2_TRIPLE_TAP)
#define BMI2_GYRO_SELF_OFF_SEL                    ((uint64_t)1 << BMI2_GYRO_SELF_OFF)
#define BMI2_WRIST_GEST_SEL                       (1 << BMI2_WRIST_GESTURE)
#define BMI2_WRIST_WEAR_WAKE_UP_SEL               (1 << BMI2_WRIST_WEAR_WAKE_UP)
#define BMI2_ACTIVITY_RECOGNITION_SEL             ((uint64_t)1 << BMI2_ACTIVITY_RECOGNITION)
#define BMI2_ACCEL_SELF_TEST_SEL                  ((uint64_t)1 << BMI2_ACCEL_SELF_TEST)
#define BMI2_WRIST_GEST_W_SEL                     (1 << BMI2_WRIST_GESTURE_WH)
#define BMI2_WRIST_WEAR_WAKE_UP_WH_SEL            (1 << BMI2_WRIST_WEAR_WAKE_UP_WH)
#define BMI2_PRIMARY_OIS_SEL                      (1 << BMI2_PRIMARY_OIS)
#define BMI2_FREE_FALL_DET_SEL                    (1 << BMI2_FREE_FALL_DET)

/*! @name Mask definitions for BMI2 wake-up feature configuration for bmi260 */
#define BMI2_WAKEUP_SENSITIVITY_MASK              UINT8_C(0x0E)
#define BMI2_WAKEUP_SINGLE_TAP_EN_MASK            UINT8_C(0x01)
#define BMI2_WAKEUP_DOUBLE_TAP_EN_MASK            UINT8_C(0x02)
#define BMI2_WAKEUP_TRIPLE_TAP_EN_MASK            UINT8_C(0x04)
#define BMI2_WAKEUP_DATA_REG_EN_MASK              UINT8_C(0x08)
#define BMI2_WAKEUP_AXIS_SEL_MASK                 UINT8_C(0x03)

/*! @name Bit position definitions for BMI2 wake-up feature configuration for bmi260 */
#define BMI2_WAKEUP_SENSITIVITY_POS               UINT8_C(0x01)
#define BMI2_WAKEUP_DOUBLE_TAP_EN_POS             UINT8_C(0x01)
#define BMI2_WAKEUP_TRIPLE_TAP_EN_POS             UINT8_C(0x02)
#define BMI2_WAKEUP_DATA_REG_EN_POS               UINT8_C(0x03)

/*! @name Mask definitions for BMI2 tap feature configuration for bmi260t */
#define BMI2_TAP_SENSITIVITY_MASK                 UINT8_C(0x0E)
#define BMI2_TAP_SINGLE_TAP_EN_MASK               UINT8_C(0x01)
#define BMI2_TAP_DOUBLE_TAP_EN_MASK               UINT8_C(0x02)
#define BMI2_TAP_TRIPLE_TAP_EN_MASK               UINT8_C(0x04)
#define BMI2_TAP_DATA_REG_EN_MASK                 UINT8_C(0x08)
#define BMI2_TAP_AXIS_SEL_MASK                    UINT8_C(0x03)

/*! @name Bit position definitions for BMI2 tap feature configuration for bmi260t */
#define BMI2_TAP_SENSITIVITY_POS                  UINT8_C(0x01)
#define BMI2_TAP_DOUBLE_TAP_EN_POS                UINT8_C(0x01)
#define BMI2_TAP_TRIPLE_TAP_EN_POS                UINT8_C(0x02)
#define BMI2_TAP_DATA_REG_EN_POS                  UINT8_C(0x03)

/*! @name Mask definitions for BMI2 wake-up feature configuration for other than bmi261 */
#define BMI2_WAKE_UP_SENSITIVITY_MASK             UINT16_C(0x000E)
#define BMI2_WAKE_UP_SINGLE_TAP_EN_MASK           UINT16_C(0x0010)

/*! @name Bit position definitions for BMI2 wake-up feature configuration for other than bmi261 */
#define BMI2_WAKE_UP_SENSITIVITY_POS              UINT8_C(0x01)
#define BMI2_WAKE_UP_SINGLE_TAP_EN_POS            UINT8_C(0x04)

/*! @name Offsets from feature start address for BMI2 feature enable/disable */
#define BMI2_ANY_MOT_FEAT_EN_OFFSET               UINT8_C(0x03)
#define BMI2_NO_MOT_FEAT_EN_OFFSET                UINT8_C(0x03)
#define BMI2_SIG_MOT_FEAT_EN_OFFSET               UINT8_C(0x0A)
#define BMI2_STEP_COUNT_FEAT_EN_OFFSET            UINT8_C(0x01)
#define BMI2_GYR_USER_GAIN_FEAT_EN_OFFSET         UINT8_C(0x05)
#define BMI2_HIGH_G_FEAT_EN_OFFSET                UINT8_C(0x03)
#define BMI2_LOW_G_FEAT_EN_OFFSET                 UINT8_C(0x03)
#define BMI2_TILT_FEAT_EN_OFFSET                  UINT8_C(0x00)

/*! @name Mask definitions for BMI2 feature enable/disable */
#define BMI2_ANY_NO_MOT_EN_MASK                   UINT8_C(0x80)
#define BMI2_TILT_FEAT_EN_MASK                    UINT8_C(0x01)
#define BMI2_ORIENT_FEAT_EN_MASK                  UINT8_C(0x01)
#define BMI2_SIG_MOT_FEAT_EN_MASK                 UINT8_C(0x01)
#define BMI2_STEP_DET_FEAT_EN_MASK                UINT8_C(0x08)
#define BMI2_STEP_COUNT_FEAT_EN_MASK              UINT8_C(0x10)
#define BMI2_STEP_ACT_FEAT_EN_MASK                UINT8_C(0x20)
#define BMI2_GYR_USER_GAIN_FEAT_EN_MASK           UINT8_C(0x08)
#define BMI2_UP_HOLD_TO_WAKE_FEAT_EN_MASK         UINT8_C(0x01)
#define BMI2_GLANCE_FEAT_EN_MASK                  UINT8_C(0x01)
#define BMI2_WAKE_UP_FEAT_EN_MASK                 UINT8_C(0x01)
#define BMI2_TAP_FEAT_EN_MASK                     UINT8_C(0x01)
#define BMI2_HIGH_G_FEAT_EN_MASK                  UINT8_C(0x80)
#define BMI2_LOW_G_FEAT_EN_MASK                   UINT8_C(0x10)
#define BMI2_FLAT_FEAT_EN_MASK                    UINT8_C(0x01)
#define BMI2_EXT_SENS_SYNC_FEAT_EN_MASK           UINT8_C(0x01)
#define BMI2_GYR_SELF_OFF_CORR_FEAT_EN_MASK       UINT8_C(0x02)
#define BMI2_WRIST_GEST_FEAT_EN_MASK              UINT8_C(0x20)
#define BMI2_WRIST_WEAR_WAKE_UP_FEAT_EN_MASK      UINT8_C(0x10)
#define BMI2_ACTIVITY_RECOG_EN_MASK               UINT8_C(0x01)
#define BMI2_ACC_SELF_TEST_FEAT_EN_MASK           UINT8_C(0x02)
#define BMI2_GYRO_SELF_TEST_CRT_EN_MASK           UINT8_C(0x01)
#define BMI2_ABORT_FEATURE_EN_MASK                UINT8_C(0x02)
#define BMI2_NVM_PREP_FEATURE_EN_MASK             UINT8_C(0x04)
#define BMI2_FREE_FALL_DET_FEAT_EN_MASK           UINT8_C(0x01)
#define BMI2_WRIST_GEST_WH_FEAT_EN_MASK           UINT8_C(0x02)

/*! @name Bit position definitions for BMI2 feature enable/disable */
#define BMI2_ANY_NO_MOT_EN_POS                    UINT8_C(0x07)
#define BMI2_STEP_DET_FEAT_EN_POS                 UINT8_C(0x03)
#define BMI2_STEP_COUNT_FEAT_EN_POS               UINT8_C(0x04)
#define BMI2_STEP_ACT_FEAT_EN_POS                 UINT8_C(0x05)
#define BMI2_GYR_USER_GAIN_FEAT_EN_POS            UINT8_C(0x03)
#define BMI2_HIGH_G_FEAT_EN_POS                   UINT8_C(0x07)
#define BMI2_LOW_G_FEAT_EN_POS                    UINT8_C(0x04)
#define BMI2_GYR_SELF_OFF_CORR_FEAT_EN_POS        UINT8_C(0x01)
#define BMI2_WRIST_GEST_FEAT_EN_POS               UINT8_C(0x05)
#define BMI2_WRIST_WEAR_WAKE_UP_FEAT_EN_POS       UINT8_C(0x04)
#define BMI2_ACC_SELF_TEST_FEAT_EN_POS            UINT8_C(0x01)
#define BMI2_ABORT_FEATURE_EN_POS                 UINT8_C(0x1)
#define BMI2_NVM_PREP_FEATURE_EN_POS              UINT8_C(0x02)
#define BMI2_WRIST_GEST_WH_FEAT_EN_POS            UINT8_C(0x01)

/*! Primary OIS low pass filter configuration position and mask */
#define BMI2_LP_FILTER_EN_MASK                    UINT8_C(0x01)

#define BMI2_LP_FILTER_CONFIG_POS                 UINT8_C(0x01)
#define BMI2_LP_FILTER_CONFIG_MASK                UINT8_C(0x06)

#define BMI2_PRIMARY_OIS_GYR_EN_POS               UINT8_C(0x06)
#define BMI2_PRIMARY_OIS_GYR_EN_MASK              UINT8_C(0x40)

#define BMI2_PRIMARY_OIS_ACC_EN_POS               UINT8_C(0x07)
#define BMI2_PRIMARY_OIS_ACC_EN_MASK              UINT8_C(0x80)

/*! @name Mask definitions for BMI2 any and no-motion feature configuration */
#define BMI2_ANY_NO_MOT_DUR_MASK                  UINT16_C(0x1FFF)
#define BMI2_ANY_NO_MOT_X_SEL_MASK                UINT16_C(0x2000)
#define BMI2_ANY_NO_MOT_Y_SEL_MASK                UINT16_C(0x4000)
#define BMI2_ANY_NO_MOT_Z_SEL_MASK                UINT16_C(0x8000)
#define BMI2_ANY_NO_MOT_THRES_MASK                UINT16_C(0x07FF)
#define BMI2_ANY_MOT_INT_MASK                     UINT8_C(0x40)

/*! @name Mask definitions for BMI2 no-motion interrupt mapping */
#define BMI2_NO_MOT_INT_MASK                      UINT8_C(0x20)

/*! @name Bit position definitions for BMI2 any and no-motion feature
 * configuration
 */
#define BMI2_ANY_NO_MOT_X_SEL_POS                 UINT8_C(0x0D)
#define BMI2_ANY_NO_MOT_Y_SEL_POS                 UINT8_C(0x0E)
#define BMI2_ANY_NO_MOT_Z_SEL_POS                 UINT8_C(0x0F)

/*! @name Mask definitions for BMI2 orientation feature configuration */
#define BMI2_ORIENT_UP_DOWN_MASK                  UINT16_C(0x0002)
#define BMI2_ORIENT_SYMM_MODE_MASK                UINT16_C(0x000C)
#define BMI2_ORIENT_BLOCK_MODE_MASK               UINT16_C(0x0030)
#define BMI2_ORIENT_THETA_MASK                    UINT16_C(0x0FC0)
#define BMI2_ORIENT_HYST_MASK                     UINT16_C(0x07FF)

/*! @name Bit position definitions for BMI2 orientation feature configuration */
#define BMI2_ORIENT_UP_DOWN_POS                   UINT8_C(0x01)
#define BMI2_ORIENT_SYMM_MODE_POS                 UINT8_C(0x02)
#define BMI2_ORIENT_BLOCK_MODE_POS                UINT8_C(0x04)
#define BMI2_ORIENT_THETA_POS                     UINT8_C(0x06)

/*! @name Mask definitions for BMI2 sig-motion feature configuration */
#define BMI2_SIG_MOT_PARAM_1_MASK                 UINT16_C(0xFFFF)
#define BMI2_SIG_MOT_PARAM_2_MASK                 UINT16_C(0xFFFF)
#define BMI2_SIG_MOT_PARAM_3_MASK                 UINT16_C(0xFFFF)
#define BMI2_SIG_MOT_PARAM_4_MASK                 UINT16_C(0xFFFF)
#define BMI2_SIG_MOT_PARAM_5_MASK                 UINT16_C(0xFFFF)

/*! @name Mask definitions for BMI2 parameter configurations */
#define BMI2_STEP_COUNT_PARAMS_MASK               UINT16_C(0xFFFF)

/*! @name Mask definitions for BMI2 step-counter/detector feature configuration */
#define BMI2_STEP_COUNT_WM_LEVEL_MASK             UINT16_C(0x03FF)
#define BMI2_STEP_COUNT_RST_CNT_MASK              UINT16_C(0x0400)
#define BMI2_STEP_BUFFER_SIZE_MASK                UINT16_C(0XFF00)
#define BMI2_STEP_COUNT_INT_MASK                  UINT8_C(0x02)
#define BMI2_STEP_ACT_INT_MASK                    UINT8_C(0x04)

/*! @name Bit position definitions for BMI2 step-counter/detector feature
 * configuration
 */
#define BMI2_STEP_COUNT_RST_CNT_POS               UINT8_C(0x0A)
#define BMI2_STEP_BUFFER_SIZE_POS                 UINT8_C(0X08)

/*! @name Mask definitions for BMI2 gyroscope user gain feature
 * configuration
 */
#define BMI2_GYR_USER_GAIN_RATIO_X_MASK           UINT16_C(0x07FF)
#define BMI2_GYR_USER_GAIN_RATIO_Y_MASK           UINT16_C(0x07FF)
#define BMI2_GYR_USER_GAIN_RATIO_Z_MASK           UINT16_C(0x07FF)

/*! @name Mask definitions for BMI2 gyroscope user gain saturation status */
#define BMI2_GYR_USER_GAIN_SAT_STAT_X_MASK        UINT8_C(0x01)
#define BMI2_GYR_USER_GAIN_SAT_STAT_Y_MASK        UINT8_C(0x02)
#define BMI2_GYR_USER_GAIN_SAT_STAT_Z_MASK        UINT8_C(0x04)
#define BMI2_G_TRIGGER_STAT_MASK                  UINT8_C(0x38)

/*! @name Bit position definitions for BMI2 gyroscope user gain saturation status */
#define BMI2_GYR_USER_GAIN_SAT_STAT_Y_POS         UINT8_C(0x01)
#define BMI2_GYR_USER_GAIN_SAT_STAT_Z_POS         UINT8_C(0x02)
#define BMI2_G_TRIGGER_STAT_POS                   UINT8_C(0x03)

/*! @name Mask definitions for MSB values of BMI2 gyroscope compensation */
#define BMI2_GYR_OFF_COMP_MSB_X_MASK              UINT8_C(0x03)
#define BMI2_GYR_OFF_COMP_MSB_Y_MASK              UINT8_C(0x0C)
#define BMI2_GYR_OFF_COMP_MSB_Z_MASK              UINT8_C(0x30)

/*! @name Bit positions for MSB values of BMI2 gyroscope compensation */
#define BMI2_GYR_OFF_COMP_MSB_Y_POS               UINT8_C(0x02)
#define BMI2_GYR_OFF_COMP_MSB_Z_POS               UINT8_C(0x04)

/*! @name Mask definitions for MSB values of BMI2 gyroscope compensation from user input */
#define BMI2_GYR_OFF_COMP_MSB_MASK                UINT16_C(0x0300)
#define BMI2_GYR_OFF_COMP_LSB_MASK                UINT16_C(0x00FF)

/*! @name Mask definitions for BMI2 orientation status */
#define BMI2_ORIENT_DETECT_MASK                   UINT8_C(0x03)
#define BMI2_ORIENT_FACE_UP_DWN_MASK              UINT8_C(0x04)

/*! @name Bit position definitions for BMI2 orientation status */
#define BMI2_ORIENT_FACE_UP_DWN_POS               UINT8_C(0x02)

/*! @name Mask definitions for NVM-VFRM error status */
#define BMI2_NVM_LOAD_ERR_STATUS_MASK             UINT8_C(0x01)
#define BMI2_NVM_PROG_ERR_STATUS_MASK             UINT8_C(0x02)
#define BMI2_NVM_ERASE_ERR_STATUS_MASK            UINT8_C(0x04)
#define BMI2_NVM_END_EXCEED_STATUS_MASK           UINT8_C(0x08)
#define BMI2_NVM_PRIV_ERR_STATUS_MASK             UINT8_C(0x10)
#define BMI2_VFRM_LOCK_ERR_STATUS_MASK            UINT8_C(0x20)
#define BMI2_VFRM_WRITE_ERR_STATUS_MASK           UINT8_C(0x40)
#define BMI2_VFRM_FATAL_ERR_STATUS_MASK           UINT8_C(0x80)

/*! @name Bit positions for NVM-VFRM error status */
#define BMI2_NVM_PROG_ERR_STATUS_POS              UINT8_C(0x01)
#define BMI2_NVM_ERASE_ERR_STATUS_POS             UINT8_C(0x02)
#define BMI2_NVM_END_EXCEED_STATUS_POS            UINT8_C(0x03)
#define BMI2_NVM_PRIV_ERR_STATUS_POS              UINT8_C(0x04)
#define BMI2_VFRM_LOCK_ERR_STATUS_POS             UINT8_C(0x05)
#define BMI2_VFRM_WRITE_ERR_STATUS_POS            UINT8_C(0x06)
#define BMI2_VFRM_FATAL_ERR_STATUS_POS            UINT8_C(0x07)

/*! @name Mask definitions for accelerometer self-test status */
#define BMI2_ACC_SELF_TEST_DONE_MASK              UINT8_C(0x01)
#define BMI2_ACC_X_OK_MASK                        UINT8_C(0x02)
#define BMI2_ACC_Y_OK_MASK                        UINT8_C(0x04)
#define BMI2_ACC_Z_OK_MASK                        UINT8_C(0x08)

/*! @name Bit Positions for accelerometer self-test status */
#define BMI2_ACC_X_OK_POS                         UINT8_C(0x01)
#define BMI2_ACC_Y_OK_POS                         UINT8_C(0x02)
#define BMI2_ACC_Z_OK_POS                         UINT8_C(0x03)

/*! @name Mask definitions for BMI2 high-g feature configuration */
#define BMI2_HIGH_G_THRES_MASK                    UINT16_C(0x7FFF)
#define BMI2_HIGH_G_HYST_MASK                     UINT16_C(0x0FFF)
#define BMI2_HIGH_G_X_SEL_MASK                    UINT16_C(0x1000)
#define BMI2_HIGH_G_Y_SEL_MASK                    UINT16_C(0x2000)
#define BMI2_HIGH_G_Z_SEL_MASK                    UINT16_C(0x4000)
#define BMI2_HIGH_G_DUR_MASK                      UINT16_C(0x0FFF)

/*! @name Bit position definitions for BMI2 high-g feature configuration */
#define BMI2_HIGH_G_X_SEL_POS                     UINT8_C(0x0C)
#define BMI2_HIGH_G_Y_SEL_POS                     UINT8_C(0x0D)
#define BMI2_HIGH_G_Z_SEL_POS                     UINT8_C(0x0E)

/*! @name Mask definitions for BMI2 low-g feature configuration */
#define BMI2_LOW_G_THRES_MASK                     UINT16_C(0x7FFF)
#define BMI2_LOW_G_HYST_MASK                      UINT16_C(0x0FFF)
#define BMI2_LOW_G_DUR_MASK                       UINT16_C(0x0FFF)

/*! @name Mask definitions for BMI2 free-fall detection feature configuration */
#define BMI2_FREE_FALL_ACCEL_SETT_MASK            UINT16_C(0xFFFF)

/*! @name Mask definitions for BMI2 flat feature configuration */
#define BMI2_FLAT_THETA_MASK                      UINT16_C(0x007E)
#define BMI2_FLAT_BLOCK_MASK                      UINT16_C(0x0180)
#define BMI2_FLAT_HYST_MASK                       UINT16_C(0x003F)
#define BMI2_FLAT_HOLD_TIME_MASK                  UINT16_C(0x3FC0)

/*! @name Bit position definitions for BMI2 flat feature configuration */
#define BMI2_FLAT_THETA_POS                       UINT8_C(0x01)
#define BMI2_FLAT_BLOCK_POS                       UINT8_C(0x07)
#define BMI2_FLAT_HOLD_TIME_POS                   UINT8_C(0x06)

/*! @name Mask definitions for BMI2 wrist gesture configuration */
#define BMI2_WRIST_GEST_WEAR_ARM_MASK             UINT16_C(0x0010)

/*! @name Bit position definitions for wrist gesture configuration */
#define BMI2_WRIST_GEST_WEAR_ARM_POS              UINT8_C(0x04)

/*! @name Mask definitions for BMI2 wrist gesture wh configuration */
#define BMI2_WRIST_GEST_WH_DEVICE_POS_MASK        UINT16_C(0x0001)
#define BMI2_WRIST_GEST_WH_INT                    UINT8_C(0x10)
#define BMI2_WRIST_GEST_WH_START_ADD              UINT8_C(0x08)

/*! @name Mask definitions for BMI2 wrist wear wake-up configuration */
#define BMI2_WRIST_WAKE_UP_WH_INT_MASK            UINT8_C(0x08)

/*! @name Mask definition for BMI2 wrist wear wake-up configuration for wearable variant */
#define BMI2_WRIST_WAKE_UP_ANGLE_LR_MASK          UINT16_C(0x00FF)
#define BMI2_WRIST_WAKE_UP_ANGLE_LL_MASK          UINT16_C(0xFF00)
#define BMI2_WRIST_WAKE_UP_ANGLE_PD_MASK          UINT16_C(0x00FF)
#define BMI2_WRIST_WAKE_UP_ANGLE_PU_MASK          UINT16_C(0xFF00)
#define BMI2_WRIST_WAKE_UP_MIN_DUR_MOVED_MASK     UINT16_C(0x00FF)
#define BMI2_WRIST_WAKE_UP_MIN_DUR_QUITE_MASK     UINT16_C(0xFF00)

/*! @name Bit position definition for BMI2 wrist wear wake-up configuration for wearable variant */
#define BMI2_WRIST_WAKE_UP_ANGLE_LL_POS           UINT16_C(0x0008)
#define BMI2_WRIST_WAKE_UP_ANGLE_PU_POS           UINT16_C(0x0008)
#define BMI2_WRIST_WAKE_UP_MIN_DUR_QUITE_POS      UINT16_C(0x0008)

/*! @name Macros to define values of BMI2 axis and its sign for re-map
 * settings
 */
#define BMI2_MAP_X_AXIS                           UINT8_C(0x00)
#define BMI2_MAP_Y_AXIS                           UINT8_C(0x01)
#define BMI2_MAP_Z_AXIS                           UINT8_C(0x02)
#define BMI2_MAP_POSITIVE                         UINT8_C(0x00)
#define BMI2_MAP_NEGATIVE                         UINT8_C(0x01)

/*! @name Mask definitions of BMI2 axis re-mapping */
#define BMI2_X_AXIS_MASK                          UINT8_C(0x03)
#define BMI2_X_AXIS_SIGN_MASK                     UINT8_C(0x04)
#define BMI2_Y_AXIS_MASK                          UINT8_C(0x18)
#define BMI2_Y_AXIS_SIGN_MASK                     UINT8_C(0x20)
#define BMI2_Z_AXIS_MASK                          UINT8_C(0xC0)
#define BMI2_Z_AXIS_SIGN_MASK                     UINT8_C(0x01)

/*! @name Bit position definitions of BMI2 axis re-mapping */
#define BMI2_X_AXIS_SIGN_POS                      UINT8_C(0x02)
#define BMI2_Y_AXIS_POS                           UINT8_C(0x03)
#define BMI2_Y_AXIS_SIGN_POS                      UINT8_C(0x05)
#define BMI2_Z_AXIS_POS                           UINT8_C(0x06)

/*! @name Macros to define polarity */
#define BMI2_NEG_SIGN                             UINT8_C(1)
#define BMI2_POS_SIGN                             UINT8_C(0)

/*! @name Macro to define related to CRT */
#define BMI2_CRT_READY_FOR_DOWNLOAD_US            UINT16_C(2000)
#define BMI2_CRT_READY_FOR_DOWNLOAD_RETRY         UINT8_C(100)

#define BMI2_CRT_WAIT_RUNNING_US                  UINT16_C(10000)
#define BMI2_CRT_WAIT_RUNNING_RETRY_EXECUTION     UINT8_C(200)

#define BMI2_CRT_MIN_BURST_WORD_LENGTH            UINT8_C(2)
#define BMI2_CRT_MAX_BURST_WORD_LENGTH            UINT16_C(255)

#define BMI2_ACC_FOC_2G_REF                       UINT16_C(16384)
#define BMI2_ACC_FOC_4G_REF                       UINT16_C(8192)
#define BMI2_ACC_FOC_8G_REF                       UINT16_C(4096)
#define BMI2_ACC_FOC_16G_REF                      UINT16_C(2048)

#define BMI2_GYRO_FOC_NOISE_LIMIT_NEGATIVE        INT8_C(-20)
#define BMI2_GYRO_FOC_NOISE_LIMIT_POSITIVE        INT8_C(20)

/* reference value with positive and negative noise range in lsb */
#define BMI2_ACC_2G_MAX_NOISE_LIMIT               (BMI2_ACC_FOC_2G_REF + UINT16_C(255))
#define BMI2_ACC_2G_MIN_NOISE_LIMIT               (BMI2_ACC_FOC_2G_REF - UINT16_C(255))
#define BMI2_ACC_4G_MAX_NOISE_LIMIT               (BMI2_ACC_FOC_4G_REF + UINT16_C(255))
#define BMI2_ACC_4G_MIN_NOISE_LIMIT               (BMI2_ACC_FOC_4G_REF - UINT16_C(255))
#define BMI2_ACC_8G_MAX_NOISE_LIMIT               (BMI2_ACC_FOC_8G_REF + UINT16_C(255))
#define BMI2_ACC_8G_MIN_NOISE_LIMIT               (BMI2_ACC_FOC_8G_REF - UINT16_C(255))
#define BMI2_ACC_16G_MAX_NOISE_LIMIT              (BMI2_ACC_FOC_16G_REF + UINT16_C(255))
#define BMI2_ACC_16G_MIN_NOISE_LIMIT              (BMI2_ACC_FOC_16G_REF - UINT16_C(255))

#define BMI2_FOC_SAMPLE_LIMIT                     UINT8_C(128)

/*! @name Bit wise selection of BMI2 sensors */
#define BMI2_MAIN_SENSORS \
    (BMI2_ACCEL_SENS_SEL | BMI2_GYRO_SENS_SEL \
     | BMI2_AUX_SENS_SEL | BMI2_TEMP_SENS_SEL)

/*!  @name Maximum number of BMI2 main sensors */
#define BMI2_MAIN_SENS_MAX_NUM                    UINT8_C(4)

/*! @name Macro to specify the number of step counter parameters */
#define BMI2_STEP_CNT_N_PARAMS                    UINT8_C(25)

/*! @name Macro to specify the number of free-fall accel setting parameters */
#define BMI2_FREE_FALL_ACCEL_SET_PARAMS           UINT8_C(7)

#define BMI2_SELECT_GYRO_SELF_TEST                UINT8_C(0)
#define BMI2_SELECT_CRT                           UINT8_C(1)

/*! @name Macro for NVM enable */
#define BMI2_NVM_UNLOCK_ENABLE                    UINT8_C(0x02)
#define BMI2_NVM_UNLOCK_DISABLE                   UINT8_C(0x00)

/*! @name macro to select between gyro self test and CRT */
#define BMI2_GYRO_SELF_TEST_SEL                   UINT8_C(0)
#define BMI2_CRT_SEL                              UINT8_C(1)

/******************************************************************************/
/*! @name       Accelerometer Macro Definitions               */
/******************************************************************************/
/*! @name Accelerometer Bandwidth parameters */
#define BMI2_ACC_OSR4_AVG1                        UINT8_C(0x00)
#define BMI2_ACC_OSR2_AVG2                        UINT8_C(0x01)
#define BMI2_ACC_NORMAL_AVG4                      UINT8_C(0x02)
#define BMI2_ACC_CIC_AVG8                         UINT8_C(0x03)
#define BMI2_ACC_RES_AVG16                        UINT8_C(0x04)
#define BMI2_ACC_RES_AVG32                        UINT8_C(0x05)
#define BMI2_ACC_RES_AVG64                        UINT8_C(0x06)
#define BMI2_ACC_RES_AVG128                       UINT8_C(0x07)

/*! @name Accelerometer Output Data Rate */
#define BMI2_ACC_ODR_0_78HZ                       UINT8_C(0x01)
#define BMI2_ACC_ODR_1_56HZ                       UINT8_C(0x02)
#define BMI2_ACC_ODR_3_12HZ                       UINT8_C(0x03)
#define BMI2_ACC_ODR_6_25HZ                       UINT8_C(0x04)
#define BMI2_ACC_ODR_12_5HZ                       UINT8_C(0x05)
#define BMI2_ACC_ODR_25HZ                         UINT8_C(0x06)
#define BMI2_ACC_ODR_50HZ                         UINT8_C(0x07)
#define BMI2_ACC_ODR_100HZ                        UINT8_C(0x08)
#define BMI2_ACC_ODR_200HZ                        UINT8_C(0x09)
#define BMI2_ACC_ODR_400HZ                        UINT8_C(0x0A)
#define BMI2_ACC_ODR_800HZ                        UINT8_C(0x0B)
#define BMI2_ACC_ODR_1600HZ                       UINT8_C(0x0C)

/*! @name Accelerometer G Range */
#define BMI2_ACC_RANGE_2G                         UINT8_C(0x00)
#define BMI2_ACC_RANGE_4G                         UINT8_C(0x01)
#define BMI2_ACC_RANGE_8G                         UINT8_C(0x02)
#define BMI2_ACC_RANGE_16G                        UINT8_C(0x03)

/*! @name Mask definitions for accelerometer configuration register */
#define BMI2_ACC_RANGE_MASK                       UINT8_C(0x03)
#define BMI2_ACC_ODR_MASK                         UINT8_C(0x0F)
#define BMI2_ACC_BW_PARAM_MASK                    UINT8_C(0x70)
#define BMI2_ACC_FILTER_PERF_MODE_MASK            UINT8_C(0x80)

/*! @name Bit position definitions for accelerometer configuration register */
#define BMI2_ACC_BW_PARAM_POS                     UINT8_C(0x04)
#define BMI2_ACC_FILTER_PERF_MODE_POS             UINT8_C(0x07)

/*! @name Self test macro to define range */
#define BMI2_ACC_SELF_TEST_RANGE                  UINT8_C(16)

/*! @name Self test macro to show resulting minimum and maximum difference
 * signal of the axes in mg
 */
#define BMI2_ST_ACC_X_SIG_MIN_DIFF                INT16_C(16000)
#define BMI2_ST_ACC_Y_SIG_MIN_DIFF                INT16_C(-15000)
#define BMI2_ST_ACC_Z_SIG_MIN_DIFF                INT16_C(10000)

/*! @name Mask definitions for accelerometer self-test */
#define BMI2_ACC_SELF_TEST_EN_MASK                UINT8_C(0x01)
#define BMI2_ACC_SELF_TEST_SIGN_MASK              UINT8_C(0x04)
#define BMI2_ACC_SELF_TEST_AMP_MASK               UINT8_C(0x08)

/*! @name Bit Positions for accelerometer self-test */
#define BMI2_ACC_SELF_TEST_SIGN_POS               UINT8_C(0x02)
#define BMI2_ACC_SELF_TEST_AMP_POS                UINT8_C(0x03)

/*! @name MASK definition for gyro self test status  */
#define BMI2_GYR_ST_AXES_DONE_MASK                UINT8_C(0X01)
#define BMI2_GYR_AXIS_X_OK_MASK                   UINT8_C(0x02)
#define BMI2_GYR_AXIS_Y_OK_MASK                   UINT8_C(0x04)
#define BMI2_GYR_AXIS_Z_OK_MASK                   UINT8_C(0x08)

/*! @name Bit position for gyro self test status  */
#define BMI2_GYR_AXIS_X_OK_POS                    UINT8_C(0x01)
#define BMI2_GYR_AXIS_Y_OK_POS                    UINT8_C(0x02)
#define BMI2_GYR_AXIS_Z_OK_POS                    UINT8_C(0x03)

/******************************************************************************/
/*! @name       Gyroscope Macro Definitions               */
/******************************************************************************/
/*! @name Gyroscope Bandwidth parameters */
#define BMI2_GYR_OSR4_MODE                        UINT8_C(0x00)
#define BMI2_GYR_OSR2_MODE                        UINT8_C(0x01)
#define BMI2_GYR_NORMAL_MODE                      UINT8_C(0x02)
#define BMI2_GYR_CIC_MODE                         UINT8_C(0x03)

/*! @name Gyroscope Output Data Rate */
#define BMI2_GYR_ODR_25HZ                         UINT8_C(0x06)
#define BMI2_GYR_ODR_50HZ                         UINT8_C(0x07)
#define BMI2_GYR_ODR_100HZ                        UINT8_C(0x08)
#define BMI2_GYR_ODR_200HZ                        UINT8_C(0x09)
#define BMI2_GYR_ODR_400HZ                        UINT8_C(0x0A)
#define BMI2_GYR_ODR_800HZ                        UINT8_C(0x0B)
#define BMI2_GYR_ODR_1600HZ                       UINT8_C(0x0C)
#define BMI2_GYR_ODR_3200HZ                       UINT8_C(0x0D)

/*! @name Gyroscope OIS Range */
#define BMI2_GYR_OIS_250                          UINT8_C(0x00)
#define BMI2_GYR_OIS_2000                         UINT8_C(0x01)

/*! @name Gyroscope Angular Rate Measurement Range */
#define BMI2_GYR_RANGE_2000                       UINT8_C(0x00)
#define BMI2_GYR_RANGE_1000                       UINT8_C(0x01)
#define BMI2_GYR_RANGE_500                        UINT8_C(0x02)
#define BMI2_GYR_RANGE_250                        UINT8_C(0x03)
#define BMI2_GYR_RANGE_125                        UINT8_C(0x04)

/*! @name Mask definitions for gyroscope configuration register */
#define BMI2_GYR_RANGE_MASK                       UINT8_C(0x07)
#define BMI2_GYR_OIS_RANGE_MASK                   UINT8_C(0x08)
#define BMI2_GYR_ODR_MASK                         UINT8_C(0x0F)
#define BMI2_GYR_BW_PARAM_MASK                    UINT8_C(0x30)
#define BMI2_GYR_NOISE_PERF_MODE_MASK             UINT8_C(0x40)
#define BMI2_GYR_FILTER_PERF_MODE_MASK            UINT8_C(0x80)

/*! @name Bit position definitions for gyroscope configuration register */
#define BMI2_GYR_OIS_RANGE_POS                    UINT8_C(0x03)
#define BMI2_GYR_BW_PARAM_POS                     UINT8_C(0x04)
#define BMI2_GYR_NOISE_PERF_MODE_POS              UINT8_C(0x06)
#define BMI2_GYR_FILTER_PERF_MODE_POS             UINT8_C(0x07)

/******************************************************************************/
/*! @name       Auxiliary Macro Definitions               */
/******************************************************************************/
/*! @name Auxiliary Output Data Rate */
#define BMI2_AUX_ODR_RESERVED                     UINT8_C(0x00)
#define BMI2_AUX_ODR_0_78HZ                       UINT8_C(0x01)
#define BMI2_AUX_ODR_1_56HZ                       UINT8_C(0x02)
#define BMI2_AUX_ODR_3_12HZ                       UINT8_C(0x03)
#define BMI2_AUX_ODR_6_25HZ                       UINT8_C(0x04)
#define BMI2_AUX_ODR_12_5HZ                       UINT8_C(0x05)
#define BMI2_AUX_ODR_25HZ                         UINT8_C(0x06)
#define BMI2_AUX_ODR_50HZ                         UINT8_C(0x07)
#define BMI2_AUX_ODR_100HZ                        UINT8_C(0x08)
#define BMI2_AUX_ODR_200HZ                        UINT8_C(0x09)
#define BMI2_AUX_ODR_400HZ                        UINT8_C(0x0A)
#define BMI2_AUX_ODR_800HZ                        UINT8_C(0x0B)

/*! @name Macro to define burst read lengths for both manual and auto modes */
#define BMI2_AUX_READ_LEN_0                       UINT8_C(0x00)
#define BMI2_AUX_READ_LEN_1                       UINT8_C(0x01)
#define BMI2_AUX_READ_LEN_2                       UINT8_C(0x02)
#define BMI2_AUX_READ_LEN_3                       UINT8_C(0x03)

/*! @name Mask definitions for auxiliary interface configuration register */
#define BMI2_AUX_SET_I2C_ADDR_MASK                UINT8_C(0xFE)
#define BMI2_AUX_MAN_MODE_EN_MASK                 UINT8_C(0x80)
#define BMI2_AUX_FCU_WR_EN_MASK                   UINT8_C(0x40)
#define BMI2_AUX_MAN_READ_BURST_MASK              UINT8_C(0x0C)
#define BMI2_AUX_READ_BURST_MASK                  UINT8_C(0x03)
#define BMI2_AUX_ODR_EN_MASK                      UINT8_C(0x0F)
#define BMI2_AUX_OFFSET_READ_OUT_MASK             UINT8_C(0xF0)

/*! @name Bit positions for auxiliary interface configuration register */
#define BMI2_AUX_SET_I2C_ADDR_POS                 UINT8_C(0x01)
#define BMI2_AUX_MAN_MODE_EN_POS                  UINT8_C(0x07)
#define BMI2_AUX_FCU_WR_EN_POS                    UINT8_C(0x06)
#define BMI2_AUX_MAN_READ_BURST_POS               UINT8_C(0x02)
#define BMI2_AUX_OFFSET_READ_OUT_POS              UINT8_C(0x04)

/******************************************************************************/
/*! @name       FIFO Macro Definitions                                        */
/******************************************************************************/
/*! @name Macros to define virtual FIFO frame mode */
#define BMI2_FIFO_VIRT_FRM_MODE                   UINT8_C(0x03)

/*! @name FIFO Header Mask definitions */
#define BMI2_FIFO_HEADER_ACC_FRM                  UINT8_C(0x84)
#define BMI2_FIFO_HEADER_AUX_FRM                  UINT8_C(0x90)
#define BMI2_FIFO_HEADER_GYR_FRM                  UINT8_C(0x88)
#define BMI2_FIFO_HEADER_GYR_ACC_FRM              UINT8_C(0x8C)
#define BMI2_FIFO_HEADER_AUX_ACC_FRM              UINT8_C(0x94)
#define BMI2_FIFO_HEADER_AUX_GYR_FRM              UINT8_C(0x98)
#define BMI2_FIFO_HEADER_ALL_FRM                  UINT8_C(0x9C)
#define BMI2_FIFO_HEADER_SENS_TIME_FRM            UINT8_C(0x44)
#define BMI2_FIFO_HEADER_SKIP_FRM                 UINT8_C(0x40)
#define BMI2_FIFO_HEADER_INPUT_CFG_FRM            UINT8_C(0x48)
#define BMI2_FIFO_HEAD_OVER_READ_MSB              UINT8_C(0x80)
#define BMI2_FIFO_VIRT_ACT_RECOG_FRM              UINT8_C(0xC8)

/*! @name BMI2 sensor selection for header-less frames  */
#define BMI2_FIFO_HEAD_LESS_ACC_FRM               UINT8_C(0x40)
#define BMI2_FIFO_HEAD_LESS_AUX_FRM               UINT8_C(0x20)
#define BMI2_FIFO_HEAD_LESS_GYR_FRM               UINT8_C(0x80)
#define BMI2_FIFO_HEAD_LESS_GYR_AUX_FRM           UINT8_C(0xA0)
#define BMI2_FIFO_HEAD_LESS_GYR_ACC_FRM           UINT8_C(0xC0)
#define BMI2_FIFO_HEAD_LESS_AUX_ACC_FRM           UINT8_C(0x60)
#define BMI2_FIFO_HEAD_LESS_ALL_FRM               UINT8_C(0xE0)

/*! @name Mask definitions for FIFO frame content configuration */
#define BMI2_FIFO_STOP_ON_FULL                    UINT16_C(0x0001)
#define BMI2_FIFO_TIME_EN                         UINT16_C(0x0002)
#define BMI2_FIFO_TAG_INT1                        UINT16_C(0x0300)
#define BMI2_FIFO_TAG_INT2                        UINT16_C(0x0C00)
#define BMI2_FIFO_HEADER_EN                       UINT16_C(0x1000)
#define BMI2_FIFO_AUX_EN                          UINT16_C(0x2000)
#define BMI2_FIFO_ACC_EN                          UINT16_C(0x4000)
#define BMI2_FIFO_GYR_EN                          UINT16_C(0x8000)
#define BMI2_FIFO_ALL_EN                          UINT16_C(0xE000)

/*! @name FIFO sensor data lengths */
#define BMI2_FIFO_ACC_LENGTH                      UINT8_C(6)
#define BMI2_FIFO_GYR_LENGTH                      UINT8_C(6)
#define BMI2_FIFO_AUX_LENGTH                      UINT8_C(8)
#define BMI2_FIFO_ACC_AUX_LENGTH                  UINT8_C(14)
#define BMI2_FIFO_GYR_AUX_LENGTH                  UINT8_C(14)
#define BMI2_FIFO_ACC_GYR_LENGTH                  UINT8_C(12)
#define BMI2_FIFO_ALL_LENGTH                      UINT8_C(20)
#define BMI2_SENSOR_TIME_LENGTH                   UINT8_C(3)
#define BMI2_FIFO_CONFIG_LENGTH                   UINT8_C(2)
#define BMI2_FIFO_WM_LENGTH                       UINT8_C(2)
#define BMI2_MAX_VALUE_FIFO_FILTER                UINT8_C(1)
#define BMI2_FIFO_DATA_LENGTH                     UINT8_C(2)
#define BMI2_FIFO_LENGTH_MSB_BYTE                 UINT8_C(1)
#define BMI2_FIFO_INPUT_CFG_LENGTH                UINT8_C(4)
#define BMI2_FIFO_SKIP_FRM_LENGTH                 UINT8_C(1)

/*! @name FIFO sensor virtual data lengths: sensor data plus sensor time */
#define BMI2_FIFO_VIRT_ACC_LENGTH                 UINT8_C(9)
#define BMI2_FIFO_VIRT_GYR_LENGTH                 UINT8_C(9)
#define BMI2_FIFO_VIRT_AUX_LENGTH                 UINT8_C(11)
#define BMI2_FIFO_VIRT_ACC_AUX_LENGTH             UINT8_C(17)
#define BMI2_FIFO_VIRT_GYR_AUX_LENGTH             UINT8_C(17)
#define BMI2_FIFO_VIRT_ACC_GYR_LENGTH             UINT8_C(15)
#define BMI2_FIFO_VIRT_ALL_LENGTH                 UINT8_C(23)

/*! @name FIFO sensor virtual data lengths: activity recognition */
#define BMI2_FIFO_VIRT_ACT_DATA_LENGTH            UINT8_C(6)
#define BMI2_FIFO_VIRT_ACT_TIME_LENGTH            UINT8_C(4)
#define BMI2_FIFO_VIRT_ACT_TYPE_LENGTH            UINT8_C(1)
#define BMI2_FIFO_VIRT_ACT_STAT_LENGTH            UINT8_C(1)

/*! @name BMI2 FIFO data filter modes */
#define BMI2_FIFO_UNFILTERED_DATA                 UINT8_C(0)
#define BMI2_FIFO_FILTERED_DATA                   UINT8_C(1)

/*! @name FIFO frame masks */
#define BMI2_FIFO_LSB_CONFIG_CHECK                UINT8_C(0x00)
#define BMI2_FIFO_MSB_CONFIG_CHECK                UINT8_C(0x80)
#define BMI2_FIFO_TAG_INTR_MASK                   UINT8_C(0xFF)

/*! @name BMI2 Mask definitions of FIFO configuration registers */
#define BMI2_FIFO_CONFIG_0_MASK                   UINT16_C(0x0003)
#define BMI2_FIFO_CONFIG_1_MASK                   UINT16_C(0xFF00)

/*! @name FIFO self wake-up mask definition */
#define BMI2_FIFO_SELF_WAKE_UP_MASK               UINT8_C(0x02)

/*! @name FIFO down sampling mask definition */
#define BMI2_ACC_FIFO_DOWNS_MASK                  UINT8_C(0x70)
#define BMI2_GYR_FIFO_DOWNS_MASK                  UINT8_C(0x07)

/*! @name FIFO down sampling bit positions */
#define BMI2_ACC_FIFO_DOWNS_POS                   UINT8_C(0x04)

/*! @name FIFO filter mask definition */
#define BMI2_ACC_FIFO_FILT_DATA_MASK              UINT8_C(0x80)
#define BMI2_GYR_FIFO_FILT_DATA_MASK              UINT8_C(0x08)

/*! @name FIFO filter bit positions */
#define BMI2_ACC_FIFO_FILT_DATA_POS               UINT8_C(0x07)
#define BMI2_GYR_FIFO_FILT_DATA_POS               UINT8_C(0x03)

/*! @name FIFO byte counter mask definition */
#define BMI2_FIFO_BYTE_COUNTER_MSB_MASK           UINT8_C(0x3F)

/*! @name FIFO self wake-up bit positions */
#define BMI2_FIFO_SELF_WAKE_UP_POS                UINT8_C(0x01)

/*! @name Mask Definitions for Virtual FIFO frames */
#define BMI2_FIFO_VIRT_FRM_MODE_MASK              UINT8_C(0xC0)
#define BMI2_FIFO_VIRT_PAYLOAD_MASK               UINT8_C(0x3C)

/*! @name Bit Positions for Virtual FIFO frames */
#define BMI2_FIFO_VIRT_FRM_MODE_POS               UINT8_C(0x06)
#define BMI2_FIFO_VIRT_PAYLOAD_POS                UINT8_C(0x02)

/******************************************************************************/
/*! @name        Interrupt Macro Definitions                  */
/******************************************************************************/
/*! @name BMI2 Interrupt Modes */
/* Non latched */
#define BMI2_INT_NON_LATCH                        UINT8_C(0)

/* Permanently latched */
#define BMI2_INT_LATCH                            UINT8_C(1)

/*! @name BMI2 Interrupt Pin Behavior */
#define BMI2_INT_PUSH_PULL                        UINT8_C(0)
#define BMI2_INT_OPEN_DRAIN                       UINT8_C(1)

/*! @name BMI2 Interrupt Pin Level */
#define BMI2_INT_ACTIVE_LOW                       UINT8_C(0)
#define BMI2_INT_ACTIVE_HIGH                      UINT8_C(1)

/*! @name BMI2 Interrupt Output Enable */
#define BMI2_INT_OUTPUT_DISABLE                   UINT8_C(0)
#define BMI2_INT_OUTPUT_ENABLE                    UINT8_C(1)

/*! @name BMI2 Interrupt Input Enable */
#define BMI2_INT_INPUT_DISABLE                    UINT8_C(0)
#define BMI2_INT_INPUT_ENABLE                     UINT8_C(1)

/*! @name Mask definitions for interrupt pin configuration */
#define BMI2_INT_LATCH_MASK                       UINT8_C(0x01)
#define BMI2_INT_LEVEL_MASK                       UINT8_C(0x02)
#define BMI2_INT_OPEN_DRAIN_MASK                  UINT8_C(0x04)
#define BMI2_INT_OUTPUT_EN_MASK                   UINT8_C(0x08)
#define BMI2_INT_INPUT_EN_MASK                    UINT8_C(0x10)

/*! @name Bit position definitions for interrupt pin configuration */
#define BMI2_INT_LEVEL_POS                        UINT8_C(0x01)
#define BMI2_INT_OPEN_DRAIN_POS                   UINT8_C(0x02)
#define BMI2_INT_OUTPUT_EN_POS                    UINT8_C(0x03)
#define BMI2_INT_INPUT_EN_POS                     UINT8_C(0x04)

/*! @name Mask definitions for data interrupt mapping */
#define BMI2_FFULL_INT                            UINT8_C(0x01)
#define BMI2_FWM_INT                              UINT8_C(0x02)
#define BMI2_DRDY_INT                             UINT8_C(0x04)
#define BMI2_ERR_INT                              UINT8_C(0x08)

/*! @name Mask definitions for data interrupt status bits */
#define BMI2_FFULL_INT_STATUS_MASK                UINT16_C(0x0100)
#define BMI2_FWM_INT_STATUS_MASK                  UINT16_C(0x0200)
#define BMI2_ERR_INT_STATUS_MASK                  UINT16_C(0x0400)
#define BMI2_AUX_DRDY_INT_MASK                    UINT16_C(0x2000)
#define BMI2_GYR_DRDY_INT_MASK                    UINT16_C(0x4000)
#define BMI2_ACC_DRDY_INT_MASK                    UINT16_C(0x8000)

/*!  @name Maximum number of interrupt pins */
#define BMI2_INT_PIN_MAX_NUM                      UINT8_C(2)

/*!  @name Macro for mapping feature interrupts */
#define BMI2_FEAT_BIT_DISABLE                     UINT8_C(0)
#define BMI2_FEAT_BIT0                            UINT8_C(1)
#define BMI2_FEAT_BIT1                            UINT8_C(2)
#define BMI2_FEAT_BIT2                            UINT8_C(3)
#define BMI2_FEAT_BIT3                            UINT8_C(4)
#define BMI2_FEAT_BIT4                            UINT8_C(5)
#define BMI2_FEAT_BIT5                            UINT8_C(6)
#define BMI2_FEAT_BIT6                            UINT8_C(7)
#define BMI2_FEAT_BIT7                            UINT8_C(8)
#define BMI2_FEAT_BIT_MAX                         UINT8_C(9)

/******************************************************************************/
/*! @name               OIS Interface Macro Definitions                       */
/******************************************************************************/
/*! @name Mask definitions for interface configuration register */
#define BMI2_OIS_IF_EN_MASK                       UINT8_C(0x10)
#define BMI2_AUX_IF_EN_MASK                       UINT8_C(0x20)

/*! @name Bit positions for OIS interface enable */
#define BMI2_OIS_IF_EN_POS                        UINT8_C(0x04)
#define BMI2_AUX_IF_EN_POS                        UINT8_C(0x05)

/******************************************************************************/
/*! @name       Macro Definitions for Axes re-mapping             */
/******************************************************************************/
/*! @name Macros for the user-defined values of axes and their polarities */
#define BMI2_X                                    UINT8_C(0x01)
#define BMI2_NEG_X                                UINT8_C(0x09)
#define BMI2_Y                                    UINT8_C(0x02)
#define BMI2_NEG_Y                                UINT8_C(0x0A)
#define BMI2_Z                                    UINT8_C(0x04)
#define BMI2_NEG_Z                                UINT8_C(0x0C)
#define BMI2_AXIS_MASK                            UINT8_C(0x07)
#define BMI2_AXIS_SIGN                            UINT8_C(0x08)

/******************************************************************************/
/*! @name         Macro Definitions for offset and gain compensation          */
/******************************************************************************/
/*! @name Mask definitions of gyroscope offset compensation registers */
#define BMI2_GYR_GAIN_EN_MASK                     UINT8_C(0x80)
#define BMI2_GYR_OFF_COMP_EN_MASK                 UINT8_C(0x40)

/*! @name Bit positions of gyroscope offset compensation registers */
#define BMI2_GYR_OFF_COMP_EN_POS                  UINT8_C(0x06)

/*! @name Mask definitions of gyroscope user-gain registers */
#define BMI2_GYR_USR_GAIN_X_MASK                  UINT8_C(0x7F)
#define BMI2_GYR_USR_GAIN_Y_MASK                  UINT8_C(0x7F)
#define BMI2_GYR_USR_GAIN_Z_MASK                  UINT8_C(0x7F)

/*! @name Bit positions of gyroscope offset compensation registers */
#define BMI2_GYR_GAIN_EN_POS                      UINT8_C(0x07)

/******************************************************************************/
/*! @name       Macro Definitions for internal status                 */
/******************************************************************************/
#define BMI2_NOT_INIT                             UINT8_C(0x00)
#define BMI2_INIT_OK                              UINT8_C(0x01)
#define BMI2_INIT_ERR                             UINT8_C(0x02)
#define BMI2_DRV_ERR                              UINT8_C(0x03)
#define BMI2_SNS_STOP                             UINT8_C(0x04)
#define BMI2_NVM_ERROR                            UINT8_C(0x05)
#define BMI2_START_UP_ERROR                       UINT8_C(0x06)
#define BMI2_COMPAT_ERROR                         UINT8_C(0x07)
#define BMI2_VFM_SKIPPED                          UINT8_C(0x10)
#define BMI2_AXES_MAP_ERROR                       UINT8_C(0x20)
#define BMI2_ODR_50_HZ_ERROR                      UINT8_C(0x40)
#define BMI2_ODR_HIGH_ERROR                       UINT8_C(0x80)

/******************************************************************************/
/*! @name        error status form gyro gain update status.               */
/******************************************************************************/
#define BMI2_G_TRIGGER_NO_ERROR                   UINT8_C(0x00)

#define BMI2_G_TRIGGER_PRECON_ERROR               UINT8_C(0x01)
#define BMI2_G_TRIGGER_DL_ERROR                   UINT8_C(0x02)
#define BMI2_G_TRIGGER_ABORT_ERROR                UINT8_C(0x03)

/******************************************************************************/
/*! @name       Variant specific features selection macros            */
/******************************************************************************/
#define BMI2_CRT_RTOSK_ENABLE                     UINT8_C(0x01)
#define BMI2_GYRO_CROSS_SENS_ENABLE               UINT8_C(0x02)
#define BMI2_GYRO_USER_GAIN_ENABLE                UINT8_C(0x08)
#define BMI2_NO_FEATURE_ENABLE                    UINT8_C(0x00)
#define BMI2_CRT_IN_FIFO_NOT_REQ                  UINT8_C(0x10)
#define BMI2_MAXIMUM_FIFO_VARIANT                 UINT8_C(0x20)

/*! Pull-up configuration for ASDA               */
#define BMI2_ASDA_PUPSEL_OFF                      UINT8_C(0x00)
#define BMI2_ASDA_PUPSEL_40K                      UINT8_C(0x01)
#define BMI2_ASDA_PUPSEL_10K                      UINT8_C(0x02)
#define BMI2_ASDA_PUPSEL_2K                       UINT8_C(0x03)

/******************************************************************************/
/*! @name           Function Pointers                             */
/******************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data     : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 *  retval =  BMA4_INTF_RET_SUCCESS -> Success
 *  retval != BMA4_INTF_RET_SUCCESS -> Failure
 *
 */
typedef BMI2_INTF_RETURN_TYPE (*bmi2_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * retval  = BMA4_INTF_RET_SUCCESS -> Success
 * retval != BMA4_INTF_RET_SUCCESS -> Failure
 *
 */
typedef BMI2_INTF_RETURN_TYPE (*bmi2_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                                                   void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bmi2_delay_fptr_t)(uint32_t period, void *intf_ptr);

/*!
 * @brief To get the configurations for wake_up feature, since wakeup feature is different for bmi260 and bmi261.
 *
 * @param[out]      wake_up    : Void pointer to store bmi2_wake_up_config structure.
 * @param[in, out]  bmi2_dev   : Void pointer to store bmi2_dev structure.
 *
 * @return Result of API execution status
 *
 * @retval BMI2_OK - Success.
 * @retval BMI2_E_COM_FAIL - Error: Communication fail
 * @retval BMI2_E_NULL_PTR - Error: Null pointer error
 * @retval BMI2_E_INVALID_PAGE - Error: Invalid Page
 *
 */
typedef int8_t (*bmi2_wake_up_fptr_t)(void *wake_up, void *bmi2_dev);

/*!
 * @brief To get the configurations for tap feature.
 *
 * @param[out]      tap        : Void pointer to store bmi2_tap_config structure.
 * @param[in, out]  bmi2_dev   : Void pointer to store bmi2_dev structure.
 *
 * @return Result of API execution status
 *
 * @retval BMI2_OK - Success.
 * @retval BMI2_E_COM_FAIL - Error: Communication fail
 * @retval BMI2_E_NULL_PTR - Error: Null pointer error
 * @retval BMI2_E_INVALID_PAGE - Error: Invalid Page
 *
 */
typedef int8_t (*bmi2_tap_fptr_t)(void *tap, void *bmi2_dev);

/******************************************************************************/
/*!  @name         Enum Declarations                                  */
/******************************************************************************/
/*!  @name Enum to define BMI2 sensor interfaces */
enum bmi2_intf {
    BMI2_SPI_INTF = 0,
    BMI2_I2C_INTF,
    BMI2_I3C_INTF
};

/*!  @name Enum to define BMI2 sensor configuration errors for accelerometer
 *   and gyroscope
 */
enum bmi2_sensor_config_error {
    BMI2_NO_ERROR,
    BMI2_ACC_ERROR,
    BMI2_GYR_ERROR,
    BMI2_ACC_GYR_ERROR
};

/*!  @name Enum to define interrupt lines */
enum bmi2_hw_int_pin {
    BMI2_INT_NONE,
    BMI2_INT1,
    BMI2_INT2,
    BMI2_INT_BOTH,
    BMI2_INT_PIN_MAX
};

/*!  @name Enum for the position of the wearable device */
enum bmi2_wear_arm_pos {
    BMI2_ARM_LEFT,
    BMI2_ARM_RIGHT
};

/*!  @name Enum to display type of activity recognition */
enum bmi2_act_recog_type {
    BMI2_ACT_UNKNOWN,
    BMI2_ACT_STILL,
    BMI2_ACT_WALK,
    BMI2_ACT_RUN,
    BMI2_ACT_BIKE,
    BMI2_ACT_VEHICLE,
    BMI2_ACT_TILTED
};

/*!  @name Enum to display activity recognition status */
enum bmi2_act_recog_stat {
    BMI2_ACT_START = 1,
    BMI2_ACT_END
};

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/
/*! @name Structure to store the compensated user-gain data of gyroscope */
struct bmi2_gyro_user_gain_data
{
    /*! x-axis */
    int8_t x;

    /*! y-axis */
    int8_t y;

    /*! z-axis */
    int8_t z;
};

/*! @name Structure to store the re-mapped axis */
struct bmi2_remap
{
    /*! Re-mapped x-axis */
    uint8_t x;

    /*! Re-mapped y-axis */
    uint8_t y;

    /*! Re-mapped z-axis */
    uint8_t z;
};

/*! @name Structure to store the value of re-mapped axis and its sign */
struct bmi2_axes_remap
{
    /*! Re-mapped x-axis */
    uint8_t x_axis;

    /*! Re-mapped y-axis */
    uint8_t y_axis;

    /*! Re-mapped z-axis */
    uint8_t z_axis;

    /*! Re-mapped x-axis sign */
    uint8_t x_axis_sign;

    /*! Re-mapped y-axis sign */
    uint8_t y_axis_sign;

    /*! Re-mapped z-axis sign */
    uint8_t z_axis_sign;
};

/*! @name Structure to define the type of sensor and its interrupt pin */
struct bmi2_sens_int_config
{
    /*! Defines the type of sensor */
    uint8_t type;

    /*! Type of interrupt pin */
    enum bmi2_hw_int_pin hw_int_pin;
};

/*! @name Structure to define output for activity recognition */
struct bmi2_act_recog_output
{
    /*! Time stamp */
    uint32_t time_stamp;

    /*! current activity */
    uint8_t curr_act;

    /*! previous activity */
    uint8_t prev_act;
};

/*! @name Structure to define FIFO frame configuration */
struct bmi2_fifo_frame
{
    /*! Pointer to FIFO data */
    uint8_t *data;

    /*! Number of user defined bytes of FIFO to be read */
    uint16_t length;

    /*! Defines header/header-less mode */
    uint8_t header_enable;

    /*! Enables type of data to be streamed - accelerometer, auxiliary or
     * gyroscope
     */
    uint16_t data_enable;

    /*! To index accelerometer bytes */
    uint16_t acc_byte_start_idx;

    /*! To index activity output bytes */
    uint16_t act_recog_byte_start_idx;

    /*! To index auxiliary bytes */
    uint16_t aux_byte_start_idx;

    /*! To index gyroscope bytes */
    uint16_t gyr_byte_start_idx;

    /*! FIFO sensor time */
    uint32_t sensor_time;

    /*! Skipped frame count */
    uint8_t skipped_frame_count;

    /*! Type of data interrupt to be mapped */
    uint8_t data_int_map;

    /*! Water-mark level for water-mark interrupt */
    uint16_t wm_lvl;

    /*! Accelerometer frame length */
    uint8_t acc_frm_len;

    /*! Gyroscope frame length */
    uint8_t gyr_frm_len;

    /*! Auxiliary frame length */
    uint8_t aux_frm_len;

    /*! Accelerometer and gyroscope frame length */
    uint8_t acc_gyr_frm_len;

    /*! Accelerometer and auxiliary frame length */
    uint8_t acc_aux_frm_len;

    /*! Gyroscope and auxiliary frame length */
    uint8_t aux_gyr_frm_len;

    /*! Accelerometer, Gyroscope and auxiliary frame length */
    uint8_t all_frm_len;
};

/*! @name Structure to define Interrupt pin configuration */
struct bmi2_int_pin_cfg
{
    /*! Configure level of interrupt pin */
    uint8_t lvl;

    /*! Configure behavior of interrupt pin */
    uint8_t od;

    /*! Output enable for interrupt pin */
    uint8_t output_en;

    /*! Input enable for interrupt pin */
    uint8_t input_en;
};

/*! @name Structure to define interrupt pin type, mode and configurations */
struct bmi2_int_pin_config
{
    /*! Interrupt pin type: INT1 or INT2 or BOTH */
    uint8_t pin_type;

    /*! Latched or non-latched mode*/
    uint8_t int_latch;

    /*! Structure to define Interrupt pin configuration */
    struct bmi2_int_pin_cfg pin_cfg[BMI2_INT_PIN_MAX_NUM];
};

/*! @name Structure to define an array of 8 auxiliary data bytes */
struct bmi2_aux_fifo_data
{
    /*! Auxiliary data */
    uint8_t data[8];

    /*! Sensor time for virtual frames */
    uint32_t virt_sens_time;
};

/*! @name Structure to define accelerometer and gyroscope sensor axes and
 * sensor time for virtual frames
 */
struct bmi2_sens_axes_data
{
    /*! Data in x-axis */
    int16_t x;

    /*! Data in y-axis */
    int16_t y;

    /*! Data in z-axis */
    int16_t z;

    /*! Sensor time for virtual frames */
    uint32_t virt_sens_time;
};

/*! @name Structure to define gyroscope saturation status of user gain */
struct bmi2_gyr_user_gain_status
{
    /*! Status in x-axis */
    uint8_t sat_x;

    /*! Status in y-axis */
    uint8_t sat_y;

    /*! Status in z-axis */
    uint8_t sat_z;

    /*! G trigger status */
    uint8_t g_trigger_status;
};

/*! @name Structure to store the status of gyro self test result */
struct bmi2_gyro_self_test_status
{
    /*! gyro self test axes done */
    uint8_t gyr_st_axes_done : 1;

    /*! status of gyro X-axis self test */
    uint8_t gyr_axis_x_ok : 1;

    /*! status of gyro Y-axis self test */
    uint8_t gyr_axis_y_ok : 1;

    /*! status of gyro Z-axis self test */
    uint8_t gyr_axis_z_ok : 1;
};

/*! @name Structure to define NVM error status */
struct bmi2_nvm_err_status
{
    /*! NVM load action error */
    uint8_t load_error;

    /*! NVM program action error */
    uint8_t prog_error;

    /*! NVM erase action error */
    uint8_t erase_error;

    /*! NVM program limit exceeded */
    uint8_t exceed_error;

    /*! NVM privilege error */
    uint8_t privil_error;
};

/*! @name Structure to define VFRM error status */
struct bmi2_vfrm_err_status
{
    /*! VFRM lock acquire error */
    uint8_t lock_error;

    /*! VFRM write error */
    uint8_t write_error;

    /*! VFRM fatal err */
    uint8_t fatal_error;
};

/*! @name Structure to define accelerometer self test feature status */
struct bmi2_acc_self_test_status
{
    /*! Accelerometer test completed */
    uint8_t acc_self_test_done;

    /*! Bit is set to 1 when accelerometer X-axis test passed */
    uint8_t acc_x_ok;

    /*! Bit is set to 1 when accelerometer y-axis test passed */
    uint8_t acc_y_ok;

    /*! Bit is set to 1 when accelerometer z-axis test passed */
    uint8_t acc_z_ok;
};

/*! @name Structure to define orientation output */
struct bmi2_orientation_output
{
    /*! Orientation portrait landscape */
    uint8_t portrait_landscape;

    /*! Orientation face-up down  */
    uint8_t faceup_down;
};

/*! @name Structure to define OIS output */
struct bmi2_ois_output
{
    /*! OIS accel x axis */
    int16_t ois_acc_x;

    /*! OIS accel y axis */
    int16_t ois_acc_y;

    /*! OIS accel z axis */
    int16_t ois_acc_z;

    /*! ois gyro x axis */
    int16_t ois_gyro_x;

    /*! OIS gyro y axis */
    int16_t ois_gyro_y;

    /*! OIS gyro z axis */
    int16_t ois_gyro_z;
};

/*! @name Union to define BMI2 sensor data */
union bmi2_sens_data
{
    /*! Accelerometer axes data */
    struct bmi2_sens_axes_data acc;

    /*! Gyroscope axes data */
    struct bmi2_sens_axes_data gyr;

    /*! Auxiliary sensor data */
    uint8_t aux_data[BMI2_AUX_NUM_BYTES];

    /*! Step counter output */
    uint32_t step_counter_output;

    /*! Step activity output */
    uint8_t activity_output;

    /*! Orientation output */
    struct bmi2_orientation_output orient_output;

    /*! High-g output */
    uint8_t high_g_output;

    /*! Gyroscope user gain saturation status */
    struct bmi2_gyr_user_gain_status gyro_user_gain_status;

    /*! NVM error status */
    struct bmi2_nvm_err_status nvm_status;

    /*! Virtual frame error status */
    struct bmi2_vfrm_err_status vfrm_status;

    /*! Wrist gesture output */
    uint8_t wrist_gesture_output;

    /*! Gyroscope cross sense value of z axis */
    int16_t correction_factor_zx;

    /*! Accelerometer self test feature status */
    struct bmi2_acc_self_test_status accel_self_test_output;

    /*! OIS output */
    struct bmi2_ois_output ois_output;
};

/*! @name Structure to define type of sensor and their respective data */
struct bmi2_sensor_data
{
    /*! Defines the type of sensor */
    uint8_t type;

    /*! Defines various sensor data */
    union bmi2_sens_data sens_data;
};

/*! @name Structure to define accelerometer configuration */
struct bmi2_accel_config
{
    /*! Output data rate in Hz */
    uint8_t odr;

    /*! Bandwidth parameter */
    uint8_t bwp;

    /*! Filter performance mode */
    uint8_t filter_perf;

    /*! g-range */
    uint8_t range;
};

/*! @name Structure to define gyroscope configuration */
struct bmi2_gyro_config
{
    /*! Output data rate in Hz */
    uint8_t odr;

    /*! Bandwidth parameter */
    uint8_t bwp;

    /*! Filter performance mode */
    uint8_t filter_perf;

    /*! OIS Range */
    uint8_t ois_range;

    /*! Gyroscope Range */
    uint8_t range;

    /*! Selects noise performance */
    uint8_t noise_perf;
};

/*! @name Structure to define auxiliary sensor configuration */
struct bmi2_aux_config
{
    /*! Enable/Disable auxiliary interface */
    uint8_t aux_en;

    /*! Manual or Auto mode*/
    uint8_t manual_en;

    /*! Enables FCU write command on auxiliary interface */
    uint8_t fcu_write_en;

    /*! Read burst length for manual mode */
    uint8_t man_rd_burst;

    /*! Read burst length for data mode */
    uint8_t aux_rd_burst;

    /*! Output data rate */
    uint8_t odr;

    /*! Read-out offset */
    uint8_t offset;

    /*! I2c address of auxiliary sensor */
    uint8_t i2c_device_addr;

    /*! Read address of auxiliary sensor */
    uint8_t read_addr;
};

/*! @name Structure to define any-motion configuration */
struct bmi2_any_motion_config
{
    /*! Duration in 50Hz samples(20msec) */
    uint16_t duration;

    /*! Acceleration slope threshold */
    uint16_t threshold;

    /*! To select per x-axis */
    uint16_t select_x;

    /*! To select per y-axis */
    uint16_t select_y;

    /*! To select per z-axis */
    uint16_t select_z;
};

/*! @name Structure to define no-motion configuration */
struct bmi2_no_motion_config
{
    /*! Duration in 50Hz samples(20msec) */
    uint16_t duration;

    /*! Acceleration slope threshold */
    uint16_t threshold;

    /*! To select per x-axis */
    uint16_t select_x;

    /*! To select per y-axis */
    uint16_t select_y;

    /*! To select per z-axis */
    uint16_t select_z;
};

/*! @name Structure to define sig-motion configuration */
struct bmi2_sig_motion_config
{
    /*! Block size */
    uint16_t block_size;

    /*! Parameter 2 */
    uint16_t param_2;

    /*! Parameter 3 */
    uint16_t param_3;

    /*! Parameter 4 */
    uint16_t param_4;

    /*! Parameter 5 */
    uint16_t param_5;
};

/*! @name Structure to define step counter/detector/activity configuration */
struct bmi2_step_config
{
    /*! Water-mark level */
    uint16_t watermark_level;

    /*! Reset counter */
    uint16_t reset_counter;

    /*! Step buffer size */
    uint8_t step_buffer_size;
};

/*! @name Structure to define gyroscope user gain configuration */
struct bmi2_gyro_user_gain_config
{
    /*! Gain update value for x-axis */
    uint16_t ratio_x;

    /*! Gain update value for y-axis */
    uint16_t ratio_y;

    /*! Gain update value for z-axis */
    uint16_t ratio_z;
};

/*! @name Structure to define wake-up configuration */
struct bmi2_wake_up_config
{
    /*! Wake-up sensitivity for bmi261 */
    uint16_t sensitivity;

    /*! Tap feature for BMI261
     * For Single tap, single_tap_en = 1
     * For Double tap, single_tap_en = 0
     */
    uint16_t single_tap_en;

    /*! Enable -> Filtered tap data, Disable -> Unfiltered data */
    uint16_t data_reg_en;

    /*! Scaling factor of threshold */
    uint16_t tap_sens_thres;

    /*! Maximum duration between each taps */
    uint16_t max_gest_dur;

    /*! Minimum quite time between the two gesture detection */
    uint16_t quite_time_after_gest;

    /*! Wait time */
    uint16_t wait_for_timeout;

    /*! Axis selection */
    uint16_t axis_sel;
};

/*! @name Structure to define tap configuration */
struct bmi2_tap_config
{
    /*! Tap sensitivity */
    uint16_t sensitivity;

    /*! Tap feature.
     * For Single tap, single_tap_en = 1
     * For Double tap, single_tap_en = 0
     */
    uint16_t single_tap_en;

    /*! Enable -> Filtered tap data, Disable -> Unfiltered data */
    uint16_t data_reg_en;

    /*! Scaling factor of threshold */
    uint16_t tap_sens_thres;

    /*! Maximum duration between each taps */
    uint16_t max_gest_dur;

    /*! Minimum quite time between the two gesture detection */
    uint16_t quite_time_after_gest;

    /*! Wait time */
    uint16_t wait_for_timeout;

    /*! Axis selection */
    uint16_t axis_sel;
};

/*! @name Structure to define orientation configuration */
struct bmi2_orient_config
{
    /*!  Upside/down detection */
    uint16_t ud_en;

    /*!  Symmetrical, high or low Symmetrical */
    uint16_t mode;

    /*!  Blocking mode */
    uint16_t blocking;

    /*!  Threshold angle */
    uint16_t theta;

    /*!  Acceleration hysteresis for orientation detection */
    uint16_t hysteresis;
};

/*! @name Structure to define high-g configuration */
struct bmi2_high_g_config
{
    /*!  Acceleration threshold */
    uint16_t threshold;

    /*!  Hysteresis */
    uint16_t hysteresis;

    /*! To select per x-axis */
    uint16_t select_x;

    /*! To select per y-axis */
    uint16_t select_y;

    /*! To select per z-axis */
    uint16_t select_z;

    /*!  Duration interval */
    uint16_t duration;
};

/*! @name Structure to define low-g configuration */
struct bmi2_low_g_config
{
    /*! Acceleration threshold */
    uint16_t threshold;

    /*! Hysteresis */
    uint16_t hysteresis;

    /*! Duration interval */
    uint16_t duration;
};

/*! @name Structure to define flat configuration */
struct bmi2_flat_config
{
    /*!  Theta angle for flat detection */
    uint16_t theta;

    /*!  Blocking mode */
    uint16_t blocking;

    /*!  Hysteresis for theta flat detection */
    uint16_t hysteresis;

    /*! Holds the duration in 50Hz samples(20msec) */
    uint16_t hold_time;
};

/*! @name Structure to define wrist gesture configuration */
struct bmi2_wrist_gest_config
{
    /*!  Wearable arm (left or right) */
    uint16_t wearable_arm;

    /*! Sine of the minimum tilt angle in portrait down direction of the device when wrist is rolled
     *  away from user. The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle).
     *  Range is 1448 to 1774. Default value is 1774.  */
    uint16_t min_flick_peak;

    /*! Value of minimum time difference between wrist roll-out and roll-in movement during flick gesture.
     *  Range is 3 to 5 samples at 50Hz. Default value is 4 (i.e. 0.08 seconds).  */
    uint16_t min_flick_samples;

    /*! Maximum time within which gesture movement has to be completed. Range is 150 to 250 samples at 50Hz.
     * Default value is 200 (i.e. 4 seconds).  */
    uint16_t max_duration;
};

/*! @name Structure to define wrist wear wake-up configuration */
struct bmi2_wrist_wear_wake_up_config
{
    /*! Cosine of min expected attitude change of the device within 1 second time window when
     *  moving within focus position.
     *  The parameter is scaled by 2048 i.e. 2048 * cos(angle). Range is 1024 to 1774.
     *  Default is 1448.  */
    uint16_t min_angle_focus;

    /*! Cosine of min expected attitude change of the device within 1 second time window when
     *  moving from non-focus to focus position.
     *  The parameter is scaled by 2048 i.e. 2048 * cos(angle). Range is 1448 to 1856.
     *  Default value is 1774.  */
    uint16_t min_angle_nonfocus;

    /*! Sine of the max allowed downward tilt angle in landscape right direction of the device,
     *  when it is in focus position
     *  (i.e. user is able to comfortably look at the dial of wear device).
     *  The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle). Range is 700 to 1024.
     *  Default value is 1024.  */
    uint16_t max_tilt_lr;

    /*! Sine of the max allowed downward tilt angle in landscape left direction of the device,
     * when it is in focus position
     *  (i.e. user is able to comfortably look at the dial of wear device).
     *   The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle). Range is 700 to
     * 1024. Default value is 700.  */
    uint16_t max_tilt_ll;

    /*! Sine of the max allowed backward tilt angle in portrait down direction of the device,
     *  when it is in focus position
     *  (i.e. user is able to comfortably look at the dial of wear device).
     *  The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle). Range is 0 to179.
     *  Default value is 179. */
    uint16_t max_tilt_pd;

    /*! Sine of the maximum allowed forward tilt angle in portrait up direction of the
     *  device, when it is in focus position
     *  (i.e. user is able to comfortably look at the dial of wear device).
     *  The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle). Range is 1774 to 1978.
     *  Default value is 1925. */
    uint16_t max_tilt_pu;
};

/*! @name Structure to define wrist gesture configuration for wearable variant */
struct bmi2_wrist_gest_w_config
{
    /*!  Wearable arm (left or right) */
    uint8_t device_position;

    /*! Minimum threshold for flick peak on y-axis */
    uint16_t min_flick_peak_y_threshold;

    /*! Minimum threshold for flick peak on z-axis */
    uint16_t min_flick_peak_z_threshold;

    /*! Maximum expected value of positive gravitational acceleration on x-axis
     * when arm is in focus pose */
    uint16_t gravity_bounds_x_pos;

    /*! Maximum expected value of negative gravitational acceleration on x-axis
     * when arm is in focus pose */
    uint16_t gravity_bounds_x_neg;

    /*! Maximum expected value of negative gravitational acceleration on y-axis
     * when arm is in focus pose */
    uint16_t gravity_bounds_y_neg;

    /*! Maximum expected value of negative gravitational acceleration on z-axis
     * when arm is in focus pose */
    uint16_t gravity_bounds_z_neg;

    /*! Exponential smoothing coefficient for adaptive peak threshold decay */
    uint16_t flick_peak_decay_coeff;

    /*! Exponential smoothing coefficient for acceleration mean estimation */
    uint16_t lp_mean_filter_coeff;

    /*! Maximum duration between 2 peaks of jiggle in samples @50Hz  */
    uint16_t max_duration_jiggle_peaks;
};

/*! @name Structure to define wrist wear wake-up configuration for wearable configuration */
struct bmi2_wrist_wear_wake_up_wh_config
{
    /*! Cosine of min expected attitude change of the device within 1 second time window when
     * moving within focus position.
     *  The parameter is scaled by 2048 i.e. 2048 * cos(angle). Range is 1024 to 1774.
     *  Default is 1448.  */
    uint16_t min_angle_focus;

    /*! Cosine of min expected attitude change of the device within 1 second time window when
     * moving from non-focus to focus position.
     *  The parameter is scaled by 2048 i.e. 2048 * cos(angle). Range is 1448 to 1856.
     *  Default value is 1774.  */
    uint16_t min_angle_nonfocus;

    /*! Sine of the max allowed downward tilt angle in landscape right direction of the device,
     * when it is in focus position  (i.e. user is able to comfortably look at the dial of wear device).
     *  The configuration parameter is scaled by 256 i.e. 256 * sin(angle). Range is 88 to 128.
     * Default value is 128.  */
    uint8_t angle_lr;

    /*! Sine of the max allowed downward tilt angle in landscape left direction of the device,
     * when it is in focus position (i.e. user is able to comfortably look at the dial of wear device).
     *   The configuration parameter is scaled by 256 i.e. 256 * sin(angle). Range is 88 to 128.
     * Default value is 128.  */
    uint8_t angle_ll;

    /*! Sine of the max allowed backward tilt angle in portrait down direction of the device,
     * when it is in focus position (i.e. user is able to comfortably look at the dial of wear device).
     *   The configuration parameter is scaled by 256 i.e. 256 * sin(angle). Range is 0 to 179.
     * Default value is 22. */
    uint8_t angle_pd;

    /*! Sine of the maximum allowed forward tilt angle in portrait up direction of the device,
     * when it is in focus position (i.e. user is able to comfortably look at the dial of wear device).
     * The configuration parameter is scaled by 256 i.e. 256 * sin(angle). Range is 222 to 247.
     * Default value is 241. */
    uint8_t angle_pu;

    /*! Minimum duration the arm should be moved while performing gesture. Range: 1 to 10,
     * resolution = 20 ms.
     * Default 2(40 ms)*/
    uint8_t min_dur_mov;

    /*! Minimum duration the arm should be static between two consecutive gestures. Range: 1 to
     * 10, resolution = 20 ms
     * Default 2(40 ms)*/
    uint8_t min_dur_quite;
};

/*! @name Structure to define primary OIS configuration */
struct bmi2_primary_ois_config
{
    uint8_t lp_filter_enable;

    uint8_t lp_filter_config;

    uint8_t primary_ois_reserved;

    uint8_t primary_ois_gyro_en;

    uint8_t primary_ois_accel_en;
};

/*! @name Structure to configure free-fall detection settings */
struct bmi2_free_fall_det_config
{
    /*! free-fall accel settings */
    uint16_t freefall_accel_settings[BMI2_FREE_FALL_ACCEL_SET_PARAMS];
};

/*!  @name Union to define the sensor configurations */
union bmi2_sens_config_types
{
    /*! Accelerometer configuration */
    struct bmi2_accel_config acc;

    /*! Gyroscope configuration */
    struct bmi2_gyro_config gyr;

    /*! Auxiliary configuration */
    struct bmi2_aux_config aux;

    /*! Any-motion configuration */
    struct bmi2_any_motion_config any_motion;

    /*! No-motion configuration */
    struct bmi2_no_motion_config no_motion;

    /*! Sig_motion configuration */
    struct bmi2_sig_motion_config sig_motion;

    /*! Step counter parameter configuration */
    uint16_t step_counter_params[BMI2_STEP_CNT_N_PARAMS];

    /*! Step counter/detector/activity configuration */
    struct bmi2_step_config step_counter;

    /*! Gyroscope user gain configuration */
    struct bmi2_gyro_user_gain_config gyro_gain_update;

    /*! Wake-up configuration */
    struct bmi2_wake_up_config tap;

    /*! Tap configuration */
    struct bmi2_tap_config tap_conf;

    /*! Orientation configuration */
    struct bmi2_orient_config orientation;

    /*! High-g configuration */
    struct bmi2_high_g_config high_g;

    /*! Low-g configuration */
    struct bmi2_low_g_config low_g;

    /*! Flat configuration */
    struct bmi2_flat_config flat;

    /*! Wrist gesture configuration */
    struct bmi2_wrist_gest_config wrist_gest;

    /*! Wrist wear wake-up configuration */
    struct bmi2_wrist_wear_wake_up_config wrist_wear_wake_up;

    /*! Wrist gesture configuration for wearable variant */
    struct bmi2_wrist_gest_w_config wrist_gest_w;

    /*! Wrist wear wake-up configuration for wearable variant */
    struct bmi2_wrist_wear_wake_up_wh_config wrist_wear_wake_up_wh;

    /*! Primary OIS configuration */
    struct bmi2_primary_ois_config primary_ois;

    /* Free-fall detection configurations */
    struct bmi2_free_fall_det_config free_fall_det;
};

/*!  @name Structure to define the type of the sensor and its configurations */
struct bmi2_sens_config
{
    /*! Defines the type of sensor */
    uint8_t type;

    /*! Defines various sensor configurations */
    union bmi2_sens_config_types cfg;
};

/*!  @name Structure to define the feature configuration */
struct bmi2_feature_config
{
    /*! Defines the type of sensor */
    uint8_t type;

    /*! Page to where the feature is mapped */
    uint8_t page;

    /*! Address of the feature */
    uint8_t start_addr;
};

/*!  @name Structure to define the feature interrupt configurations */
struct bmi2_map_int
{
    /*! Defines the type of sensor */
    uint8_t type;

    /*! Defines the feature interrupt */
    uint8_t sens_map_int;
};

/*!  @name Structure to define BMI2 sensor configurations */
struct bmi2_dev
{
    /*! Chip id of BMI2 */
    uint8_t chip_id;

    /*! The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *intf_ptr;

    /*! To store warnings */
    uint8_t info;

    /*! Type of Interface  */
    enum bmi2_intf intf;

    /*! To store interface pointer error */
    BMI2_INTF_RETURN_TYPE intf_rslt;

    /*! For switching from I2C to SPI */
    uint8_t dummy_byte;

    /*! Resolution for FOC */
    uint8_t resolution;

    /*! User set read/write length */
    uint16_t read_write_len;

    /*! Pointer to the configuration data buffer address */
    const uint8_t *config_file_ptr;

    /*! To define maximum page number */
    uint8_t page_max;

    /*! To define maximum number of input sensors/features */
    uint8_t input_sens;

    /*! To define maximum number of output sensors/features */
    uint8_t out_sens;

    /*! Indicate manual enable for auxiliary communication */
    uint8_t aux_man_en;

    /*! Defines manual read burst length for auxiliary communication */
    uint8_t aux_man_rd_burst_len;

    /*! Array of feature input configuration structure */
    const struct bmi2_feature_config *feat_config;

    /*! Array of feature output configuration structure */
    const struct bmi2_feature_config *feat_output;

    /*! Structure to maintain a copy of the re-mapped axis */
    struct bmi2_axes_remap remap;

    /*! Flag to hold enable status of sensors */
    uint64_t sens_en_stat;

    /*! Read function pointer */
    bmi2_read_fptr_t read;

    /*! Write function pointer */
    bmi2_write_fptr_t write;

    /*!  Delay function pointer */
    bmi2_delay_fptr_t delay_us;

    /*! To store the gyroscope cross sensitivity value */
    int16_t gyr_cross_sens_zx;

    /* gyro enable status, used as a flag in CRT enabling and aborting */
    uint8_t gyro_en : 1;

    /* advance power saving mode status, used as a flag in CRT enabling and aborting */
    uint8_t aps_status;

    /* used as a flag to enable variant specific features like crt */
    uint16_t variant_feature;

    /* To store hold the size of config file */
    uint16_t config_size;

    /*! Function pointer to get wakeup configurations */
    bmi2_wake_up_fptr_t get_wakeup_config;

    /*! Function pointer to set wakeup configurations */
    bmi2_wake_up_fptr_t set_wakeup_config;

    /*! Function pointer to get tap configurations */
    bmi2_tap_fptr_t get_tap_config;

    /*! Function pointer to set tap configurations */
    bmi2_tap_fptr_t set_tap_config;

    /*! Array of feature interrupts configuration structure */
    struct bmi2_map_int *map_int;

    /*! To define maximum number of interrupts */
    uint8_t sens_int_map;
};

/*!  @name Structure to enable an accel axis for foc */
struct bmi2_accel_foc_g_value
{
    /*! '0' to disable x axis and '1' to enable x axis */
    uint8_t x;

    /*! '0' to disable y axis and '1' to enable y axis */
    uint8_t y;

    /*! '0' to disable z axis and '1' to enable z axis */
    uint8_t z;

    /*! '0' for positive input and '1' for negative input */
    uint8_t sign;
};

/*! @name Structure to configure activity recognition settings */
struct bmi2_act_recg_sett
{
    /*! Activity recognition register 1 */
    uint8_t act_rec_1 : 1;

    /*! Activity recognition register 2 */
    uint16_t act_rec_2;

    /*! Activity recognition register 3 */
    uint16_t act_rec_3;

    /*! Activity recognition register 4 */
    uint8_t act_rec_4 : 4;

    /*! Activity recognition register 5 */
    uint8_t act_rec_5 : 4;
};

/*! @name Structure to configure activity recognition settings for bmi270hc */
struct bmi2_hc_act_recg_sett
{
    /*! Static segment size for activity classification. */
    uint8_t segment_size;

    /*! Enable/Disable post processing of the activity detected */
    uint8_t pp_en;

    /*! Minimum threshold of the Gini's diversity index (GDI) */
    uint16_t min_gdi_thres;

    /*! Maximum threshold of the Gini's diversity index (GDI) */
    uint16_t max_gdi_thres;

    /*! Buffer size for post processing of the activity detected */
    uint16_t buf_size;

    /*! Minimum segments belonging to a certain activity type */
    uint16_t min_seg_conf;
};

#endif /* BMI2_DEFS_H_ */
