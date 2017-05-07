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

#pragma once

#if defined(INVERTER_PIN_UART1) || defined(INVERTER_PIN_UART2) || defined(INVERTER_PIN_UART3) || defined(INVERTER_PIN_UART4) || defined(INVERTER_PIN_UART5) || defined(INVERTER_PIN_UART6)
#define USE_INVERTER
#endif

#include "config/parameter_group.h"
#include "drivers/io.h"
#include "drivers/serial.h"

typedef struct inverterConfig_s {
    ioTag_t ioTag[SERIAL_PORT_MAX_INDEX]; // XXX Many waisted bytes here; should be hard uart count
} inverterConfig_t;

PG_DECLARE(inverterConfig_t, inverterConfig);

void initInverters(const inverterConfig_t *inverterConfigToUse);

void enableInverter(USART_TypeDef *USARTx, bool on);
