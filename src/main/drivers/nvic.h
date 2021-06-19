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

#pragma once


// can't use 0
#define NVIC_PRIO_MAX                      NVIC_BUILD_PRIORITY(0, 1)
#define NVIC_PRIO_TIMER                    NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_BARO_EXTI                NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_SONAR_EXTI               NVIC_BUILD_PRIORITY(2, 0)  // maybe increase slightly
#define NVIC_PRIO_DSHOT_DMA                NVIC_BUILD_PRIORITY(2, 1)
#define NVIC_PRIO_TRANSPONDER_DMA          NVIC_BUILD_PRIORITY(3, 0)
#define NVIC_PRIO_MPU_INT_EXTI             NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_MAG_INT_EXTI             NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_WS2811_DMA               NVIC_BUILD_PRIORITY(1, 2)  // TODO - is there some reason to use high priority? (or to use DMA IRQ at all?)
#define NVIC_PRIO_SERIALUART_TXDMA         NVIC_BUILD_PRIORITY(1, 1)  // Highest of all SERIALUARTx_TXDMA
#define NVIC_PRIO_SERIALUART1_TXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART1_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART1              NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART2_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART2_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART2              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART3_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART3_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART3              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART4_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART4_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART4              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART5_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART5_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART5              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART6_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART6_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART6              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART7_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART7_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART7              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART8_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART8_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART8              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART9_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART9_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART9              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART10_TXDMA       NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART10_RXDMA       NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART10             NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALLPUART1_TXDMA      NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALLPUART1_RXDMA      NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALLPUART1            NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_I2C_ER                   NVIC_BUILD_PRIORITY(0, 0)
#define NVIC_PRIO_I2C_EV                   NVIC_BUILD_PRIORITY(0, 0)
#define NVIC_PRIO_USB                      NVIC_BUILD_PRIORITY(2, 0)
#define NVIC_PRIO_USB_WUP                  NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SONAR_ECHO               NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_MPU_DATA_READY           NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_MAG_DATA_READY           NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_CALLBACK                 NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_MAX7456_DMA              NVIC_BUILD_PRIORITY(3, 0)
#define NVIC_PRIO_SDIO_DMA                 NVIC_BUILD_PRIORITY(0, 0)

#ifdef USE_HAL_DRIVER
// utility macros to join/split priority
#define NVIC_PRIORITY_GROUPING NVIC_PRIORITYGROUP_2
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING))))>>4)
#else
// utility macros to join/split priority
#define NVIC_PRIORITY_GROUPING NVIC_PriorityGroup_2
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
#endif
