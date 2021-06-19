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

#ifdef USE_UART

#include "build/build_config.h"

#include "io/serial.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "pg/pg_ids.h"

// Backward compatibility for exisiting targets

// F1 targets don't explicitly define pins.

#ifdef STM32F1
#ifdef USE_UART1
#ifndef UART1_RX_PIN
#define UART1_RX_PIN        PA10
#endif
#ifndef UART1_TX_PIN
#define UART1_TX_PIN        PA9
#endif
#endif // USE_UART1

#ifdef USE_UART2
#ifndef UART2_RX_PIN
#define UART2_RX_PIN        PA3
#endif
#ifndef UART2_TX_PIN
#define UART2_TX_PIN        PA2
#endif
#endif // USE_UART2
#endif // STM32F1

// XXX Is there an F3 target that does not define UART pins?

#ifdef STM32F3
#ifdef USE_UART1
#ifndef UART1_TX_PIN
#define UART1_TX_PIN        PA9
#endif
#ifndef UART1_RX_PIN
#define UART1_RX_PIN        PA10
#endif
#endif

#ifdef USE_UART2
#ifndef UART2_TX_PIN
#define UART2_TX_PIN        PD5
#endif
#ifndef UART2_RX_PIN
#define UART2_RX_PIN        PD6
#endif
#endif

#ifdef USE_UART3
#ifndef UART3_TX_PIN
#define UART3_TX_PIN        PB10
#endif
#ifndef UART3_RX_PIN
#define UART3_RX_PIN        PB11
#endif
#endif

#ifdef USE_UART4
#ifndef UART4_TX_PIN
#define UART4_TX_PIN        PC10
#endif
#ifndef UART4_RX_PIN
#define UART4_RX_PIN        PC11
#endif
#endif

#ifdef USE_UART5
#ifndef UART5_TX_PIN
#define UART5_TX_PIN        PC12
#endif
#ifndef UART5_RX_PIN
#define UART5_RX_PIN        PD2
#endif
#endif
#endif // STM32F3

// Default pin (NONE).

#ifdef USE_UART1
# if !defined(UART1_RX_PIN)
#  define UART1_RX_PIN NONE
# endif
# if !defined(UART1_TX_PIN)
#  define UART1_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART1)
#  define INVERTER_PIN_UART1 NONE
# endif
#endif

#ifdef USE_UART2
# if !defined(UART2_RX_PIN)
#  define UART2_RX_PIN NONE
# endif
# if !defined(UART2_TX_PIN)
#  define UART2_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART2)
#  define INVERTER_PIN_UART2 NONE
# endif
#endif

#ifdef USE_UART3
# if !defined(UART3_RX_PIN)
#  define UART3_RX_PIN NONE
# endif
# if !defined(UART3_TX_PIN)
#  define UART3_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART3)
#  define INVERTER_PIN_UART3 NONE
# endif
#endif

#ifdef USE_UART4
# if !defined(UART4_RX_PIN)
#  define UART4_RX_PIN NONE
# endif
# if !defined(UART4_TX_PIN)
#  define UART4_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART4)
#  define INVERTER_PIN_UART4 NONE
# endif
#endif

#ifdef USE_UART5
# if !defined(UART5_RX_PIN)
#  define UART5_RX_PIN NONE
# endif
# if !defined(UART5_TX_PIN)
#  define UART5_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART5)
#  define INVERTER_PIN_UART5 NONE
# endif
#endif

#ifdef USE_UART6
# if !defined(UART6_RX_PIN)
#  define UART6_RX_PIN NONE
# endif
# if !defined(UART6_TX_PIN)
#  define UART6_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART6)
#  define INVERTER_PIN_UART6 NONE
# endif
#endif

#ifdef USE_UART7
# if !defined(UART7_RX_PIN)
#  define UART7_RX_PIN NONE
# endif
# if !defined(UART7_TX_PIN)
#  define UART7_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART7)
#  define INVERTER_PIN_UART7 NONE
# endif
#endif

#ifdef USE_UART8
# if !defined(UART8_RX_PIN)
#  define UART8_RX_PIN NONE
# endif
# if !defined(UART8_TX_PIN)
#  define UART8_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART8)
#  define INVERTER_PIN_UART8 NONE
# endif
#endif

#ifdef USE_UART9
# if !defined(UART9_RX_PIN)
#  define UART9_RX_PIN NONE
# endif
# if !defined(UART9_TX_PIN)
#  define UART9_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART9)
#  define INVERTER_PIN_UART9 NONE
# endif
#endif

#ifdef USE_UART10
# if !defined(UART10_RX_PIN)
#  define UART10_RX_PIN NONE
# endif
# if !defined(UART10_TX_PIN)
#  define UART10_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART10)
#  define INVERTER_PIN_UART10 NONE
# endif
#endif

#ifdef USE_LPUART1
# if !defined(LPUART1_RX_PIN)
#  define LPUART1_RX_PIN NONE
# endif
# if !defined(LPUART1_TX_PIN)
#  define LPUART1_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_LPUART1)
#  define INVERTER_PIN_LPUART1 NONE
# endif
#endif

#ifdef USE_SOFTSERIAL1
# if !defined(SOFTSERIAL1_RX_PIN)
#  define SOFTSERIAL1_RX_PIN NONE
# endif
# if !defined(SOFTSERIAL1_TX_PIN)
#  define SOFTSERIAL1_TX_PIN NONE
# endif
#endif

#ifdef USE_SOFTSERIAL2
# if !defined(SOFTSERIAL2_RX_PIN)
#  define SOFTSERIAL2_RX_PIN NONE
# endif
# if !defined(SOFTSERIAL2_TX_PIN)
#  define SOFTSERIAL2_TX_PIN NONE
# endif
#endif

#if defined(USE_UART) || defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL1)
typedef struct serialDefaultPin_s {
    serialPortIdentifier_e ident;
    ioTag_t rxIO, txIO, inverterIO;
} serialDefaultPin_t;

static const serialDefaultPin_t serialDefaultPin[] = {
#ifdef USE_UART1
    { SERIAL_PORT_USART1, IO_TAG(UART1_RX_PIN), IO_TAG(UART1_TX_PIN), IO_TAG(INVERTER_PIN_UART1) },
#endif
#ifdef USE_UART2
    { SERIAL_PORT_USART2, IO_TAG(UART2_RX_PIN), IO_TAG(UART2_TX_PIN), IO_TAG(INVERTER_PIN_UART2) },
#endif
#ifdef USE_UART3
    { SERIAL_PORT_USART3, IO_TAG(UART3_RX_PIN), IO_TAG(UART3_TX_PIN), IO_TAG(INVERTER_PIN_UART3) },
#endif
#ifdef USE_UART4
    { SERIAL_PORT_UART4, IO_TAG(UART4_RX_PIN), IO_TAG(UART4_TX_PIN), IO_TAG(INVERTER_PIN_UART4) },
#endif
#ifdef USE_UART5
    { SERIAL_PORT_UART5, IO_TAG(UART5_RX_PIN), IO_TAG(UART5_TX_PIN), IO_TAG(INVERTER_PIN_UART5) },
#endif
#ifdef USE_UART6
    { SERIAL_PORT_USART6, IO_TAG(UART6_RX_PIN), IO_TAG(UART6_TX_PIN), IO_TAG(INVERTER_PIN_UART6) },
#endif
#ifdef USE_UART7
    { SERIAL_PORT_USART7, IO_TAG(UART7_RX_PIN), IO_TAG(UART7_TX_PIN), IO_TAG(INVERTER_PIN_UART7) },
#endif
#ifdef USE_UART8
    { SERIAL_PORT_USART8, IO_TAG(UART8_RX_PIN), IO_TAG(UART8_TX_PIN), IO_TAG(INVERTER_PIN_UART8) },
#endif
#ifdef USE_UART9
    { SERIAL_PORT_UART9, IO_TAG(UART9_RX_PIN), IO_TAG(UART9_TX_PIN), IO_TAG(INVERTER_PIN_UART9) },
#endif
#ifdef USE_UART10
    { SERIAL_PORT_USART10, IO_TAG(UART10_RX_PIN), IO_TAG(UART10_TX_PIN), IO_TAG(INVERTER_PIN_UART10) },
#endif
#ifdef USE_LPUART1
    { SERIAL_PORT_LPUART1, IO_TAG(LPUART1_RX_PIN), IO_TAG(LPUART1_TX_PIN), IO_TAG(INVERTER_PIN_LPUART1) },
#endif
#ifdef USE_SOFTSERIAL1
    { SERIAL_PORT_SOFTSERIAL1, IO_TAG(SOFTSERIAL1_RX_PIN), IO_TAG(SOFTSERIAL1_TX_PIN), IO_TAG(NONE) },
#endif
#ifdef USE_SOFTSERIAL2
    { SERIAL_PORT_SOFTSERIAL2, IO_TAG(SOFTSERIAL2_RX_PIN), IO_TAG(SOFTSERIAL2_TX_PIN), IO_TAG(NONE) },
#endif
};

PG_REGISTER_WITH_RESET_FN(serialPinConfig_t, serialPinConfig, PG_SERIAL_PIN_CONFIG, 0);

void pgResetFn_serialPinConfig(serialPinConfig_t *serialPinConfig)
{
    for (size_t index = 0 ; index < ARRAYLEN(serialDefaultPin) ; index++) {
        const serialDefaultPin_t *defpin = &serialDefaultPin[index];
        serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_INDEX(defpin->ident)] = defpin->rxIO;
        serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_INDEX(defpin->ident)] = defpin->txIO;
        serialPinConfig->ioTagInverter[SERIAL_PORT_IDENTIFIER_TO_INDEX(defpin->ident)] = defpin->inverterIO;
    }
}
#endif
#endif
