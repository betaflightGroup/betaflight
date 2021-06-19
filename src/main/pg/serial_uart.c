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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_UART

#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/serial_uart.h"

#include "drivers/io_types.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(serialUartConfig_t, UARTDEV_CONFIG_MAX, serialUartConfig, PG_SERIAL_UART_CONFIG, 0);

typedef struct uartDmaopt_s {
    UARTDevice_e device;
    int8_t txDmaopt;
    int8_t rxDmaopt;
} uartDmaopt_t;

static uartDmaopt_t uartDmaopt[] = {
#ifdef USE_UART1
    { UARTDEV_1, UART1_TX_DMA_OPT, UART1_RX_DMA_OPT },
#endif
#ifdef USE_UART2
    { UARTDEV_2, UART2_TX_DMA_OPT, UART2_RX_DMA_OPT },
#endif
#ifdef USE_UART3
    { UARTDEV_3, UART3_TX_DMA_OPT, UART3_RX_DMA_OPT },
#endif
#ifdef USE_UART4
    { UARTDEV_4, UART4_TX_DMA_OPT, UART4_RX_DMA_OPT },
#endif
#ifdef USE_UART5
    { UARTDEV_5, UART5_TX_DMA_OPT, UART5_RX_DMA_OPT },
#endif
#ifdef USE_UART6
    { UARTDEV_6, UART6_TX_DMA_OPT, UART6_RX_DMA_OPT },
#endif
#ifdef USE_UART7
    { UARTDEV_7, UART7_TX_DMA_OPT, UART7_RX_DMA_OPT },
#endif
#ifdef USE_UART8
    { UARTDEV_8, UART8_TX_DMA_OPT, UART8_RX_DMA_OPT },
#endif
#ifdef USE_UART9
    { UARTDEV_9, UART9_TX_DMA_OPT, UART9_RX_DMA_OPT },
#endif
#ifdef USE_UART10
    { UARTDEV_10, UART10_TX_DMA_OPT, UART10_RX_DMA_OPT },
#endif
};

void pgResetFn_serialUartConfig(serialUartConfig_t *config)
{
    for (unsigned i = 0; i < UARTDEV_CONFIG_MAX; i++) {
        config[i].txDmaopt = -1;
        config[i].rxDmaopt = -1;
    }

    for (unsigned i = 0; i < ARRAYLEN(uartDmaopt); i++) {
        UARTDevice_e device = uartDmaopt[i].device;
        config[device].txDmaopt = uartDmaopt[i].txDmaopt;
        config[device].rxDmaopt = uartDmaopt[i].rxDmaopt;
    }
}
#endif
