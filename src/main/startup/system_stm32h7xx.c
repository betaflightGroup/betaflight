/**
  ******************************************************************************
  * @file    system_stm32h7xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-Mx Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32h7xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32h7xx_system
  * @{
  */

/** @addtogroup STM32H7xx_System_Private_Includes
  * @{
  */

#include "stm32h7xx.h"
#include "drivers/system.h"
#include "platform.h"
#include "string.h"
#include "common/utils.h"

#include "build/debug.h"

void forcedSystemResetWithoutDisablingCaches(void);

#if !defined  (HSE_VALUE)
#define HSE_VALUE    ((uint32_t)25000000) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (CSI_VALUE)
#define CSI_VALUE    ((uint32_t)4000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* CSI_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    ((uint32_t)64000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */


/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM mounted
     on EVAL board as data memory  */
/*#define DATA_IN_ExtSRAM */
/*#define DATA_IN_ExtSDRAM*/

#if defined(DATA_IN_ExtSRAM) && defined(DATA_IN_ExtSDRAM)
#error "Please select DATA_IN_ExtSRAM or DATA_IN_ExtSDRAM "
#endif /* DATA_IN_ExtSRAM && DATA_IN_ExtSDRAM */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00       /*!< Vector Table base offset field.
                                      This value must be a multiple of 0x200. */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
  uint32_t SystemCoreClock = 64000000;
  uint32_t SystemD2Clock = 64000000;
  const  uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_FunctionPrototypes
  * @{
  */
#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
static void SystemInit_ExtMemCtl(void);
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Private_Functions
  * @{
  */

static void ErrorHandler(void)
{
    while (1);
}

void HandleStuckSysTick(void)
{
    uint32_t tickStart = HAL_GetTick();
    uint32_t tickEnd = 0;

    // H7 at 480Mhz requires a loop count of 160000. Double this for the timeout to be safe.
    int attemptsRemaining = 320000;
    while (((tickEnd = HAL_GetTick()) == tickStart) && --attemptsRemaining) {
    }

    if (tickStart == tickEnd) {
        forcedSystemResetWithoutDisablingCaches();
    }
}

typedef struct pllConfig_s {
    uint16_t clockMhz;
    uint8_t m;
    uint16_t n;
    uint8_t p;
    uint8_t q;
    uint8_t r;
    uint32_t vos;
    uint32_t vciRange;
} pllConfig_t;

#if defined(STM32H743xx) || defined(STM32H750xx)
/*
   PLL1 configuration for different silicon revisions of H743 and H750.

   Note for future overclocking support.

   - Rev.Y (and Rev.X), nominal max at 400MHz, runs stably overclocked to 480MHz.
   - Rev.V, nominal max at 480MHz, runs stably at 540MHz, but not to 600MHz (VCO probably out of operating range)

   - A possible frequency table would look something like this, and a revision
     check logic would place a cap for Rev.Y and V.

        400 420 440 460 (Rev.Y & V ends here) 480 500 520 540
 */

// 400MHz for Rev.Y (and Rev.X)
pllConfig_t pll1ConfigRevY = {
    .clockMhz = 400,
    .m = 4,
    .n = 400,
    .p = 2,
    .q = 8,
    .r = 5,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE1,
    .vciRange = RCC_PLL1VCIRANGE_2,
};

// 480MHz for Rev.V
pllConfig_t pll1ConfigRevV = {
    .clockMhz = 480,
    .m = 4,
    .n = 480,
    .p = 2,
    .q = 8,
    .r = 5,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE0,
    .vciRange = RCC_PLL1VCIRANGE_2,
};

#define MCU_HCLK_DIVIDER RCC_HCLK_DIV2

// H743 and H750
// For HCLK=200MHz with VOS1 range, ST recommended flash latency is 2WS.
// RM0433 (Rev.5) Table 12. FLASH recommended number of wait states and programming delay
//
// For higher HCLK frequency, VOS0 is available on RevV silicons, with FLASH wait states 4WS
// AN5312 (Rev.1) Section 1.2.1 Voltage scaling Table.1
//
// XXX Check if Rev.V requires a different value

#define MCU_FLASH_LATENCY FLASH_LATENCY_2

// Source for CRS input
#define MCU_RCC_CRS_SYNC_SOURCE RCC_CRS_SYNC_SOURCE_USB2

// Workaround for weird HSE behaviors
// (Observed only on Rev.V H750, but may also apply to H743 and Rev.V.)
#define USE_H7_HSERDY_SLOW_WORKAROUND
#define USE_H7_HSE_TIMEOUT_WORKAROUND

#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)

// Nominal max 280MHz with 8MHz HSE
// (340 is okay, 360 doesn't work.)
// 

pllConfig_t pll1Config7A3 = {
    .clockMhz = 280,
    .m = 4,
    .n = 280,
    .p = 2,
    .q = 8,
    .r = 5,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE0,
    .vciRange = RCC_PLL1VCIRANGE_1,
};

// Unlike H743/H750, HCLK can be directly fed with SYSCLK.
#define MCU_HCLK_DIVIDER RCC_HCLK_DIV1

// RM0455 (Rev.6) Table 15. FLASH recommended number of wait states and programming delay
// 280MHz at VOS0 is 6WS

#define MCU_FLASH_LATENCY FLASH_LATENCY_6

// Source for CRS input
#define MCU_RCC_CRS_SYNC_SOURCE RCC_CRS_SYNC_SOURCE_USB1

#elif defined(STM32H723xx) || defined(STM32H725xx)

// Nominal max 550MHz

pllConfig_t pll1Config72x = {
    .clockMhz = 550,
    .m = 4,
    .n = 275,
    .p = 1,
    .q = 2,
    .r = 2,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE0,
    .vciRange = RCC_PLL1VCIRANGE_1,
};

#define MCU_HCLK_DIVIDER RCC_HCLK_DIV2

// RM0468 (Rev.2) Table 16. 
// 550MHz (AXI Interface clock) at VOS0 is 3WS
#define MCU_FLASH_LATENCY FLASH_LATENCY_3

#define MCU_RCC_CRS_SYNC_SOURCE RCC_CRS_SYNC_SOURCE_USB1

#else
#error Unknown MCU type
#endif

// HSE clock configuration, originally taken from
// STM32Cube_FW_H7_V1.3.0/Projects/STM32H743ZI-Nucleo/Examples/RCC/RCC_ClockConfig/Src/main.c
static void SystemClockHSE_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

#ifdef notdef
    // CSI has been disabled at SystemInit().
    // HAL_RCC_ClockConfig() will fail because CSIRDY is off.

    /* -1- Select CSI as system clock source to allow modification of the PLL configuration */

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_CSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        /* Initialization Error */
        ErrorHandler();
    }
#endif

    pllConfig_t *pll1Config;

#if defined(STM32H743xx) || defined(STM32H750xx)
    pll1Config = (HAL_GetREVID() == REV_ID_V) ? &pll1ConfigRevV : &pll1ConfigRevY;
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
    pll1Config = &pll1Config7A3;
#elif defined(STM32H723xx) || defined(STM32H725xx)
    pll1Config = &pll1Config72x;
#else
#error Unknown MCU type
#endif

    // Configure voltage scale.
    // It has been pre-configured at PWR_REGULATOR_VOLTAGE_SCALE1,
    // and it may stay or overridden by PWR_REGULATOR_VOLTAGE_SCALE0 depending on the clock config.

    __HAL_PWR_VOLTAGESCALING_CONFIG(pll1Config->vos);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
        // Empty
    }

    /* -2- Enable HSE  Oscillator, select it as PLL source and finally activate the PLL */

#ifdef USE_H7_HSERDY_SLOW_WORKAROUND
    // With reference to 2.3.22 in the ES0250 Errata for the L476.
    // Applying the same workaround here in the vain hopes that it improves startup times.
    // Randomly the HSERDY bit takes AGES, over 10 seconds, to be set.

    __HAL_RCC_GPIOH_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    GPIO_InitTypeDef  gpio_initstruct;
    gpio_initstruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initstruct.Pull = GPIO_NOPULL;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(GPIOH, &gpio_initstruct);
#endif

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Even Nucleo-H473ZI and Nucleo-H7A3ZI work without RCC_HSE_BYPASS

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = pll1Config->m;
    RCC_OscInitStruct.PLL.PLLN = pll1Config->n;
    RCC_OscInitStruct.PLL.PLLP = pll1Config->p;
    RCC_OscInitStruct.PLL.PLLQ = pll1Config->q;
    RCC_OscInitStruct.PLL.PLLR = pll1Config->r;

    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLRGE = pll1Config->vciRange;
    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&RCC_OscInitStruct);

#ifdef USE_H7_HSE_TIMEOUT_WORKAROUND
    if (status == HAL_TIMEOUT) {
        forcedSystemResetWithoutDisablingCaches(); // DC - sometimes HSERDY gets stuck, waiting longer doesn't help.
    }
#endif

    if (status != HAL_OK) {
        /* Initialization Error */
        ErrorHandler();
    }

    // Configure PLL2 and PLL3
    // Use of PLL2 and PLL3 are not determined yet.
    // A review of total system wide clock requirements is necessary.


    // Configure SCGU (System Clock Generation Unit)
    // Select PLL as system clock source and configure bus clock dividers.
    //
    // Clock type and divider member names do not have direct visual correspondence.
    // Here is how these correspond:
    //   RCC_CLOCKTYPE_SYSCLK           sys_ck
    //   RCC_CLOCKTYPE_HCLK             AHBx (rcc_hclk1,rcc_hclk2,rcc_hclk3,rcc_hclk4)
    //   RCC_CLOCKTYPE_D1PCLK1          APB3 (rcc_pclk3)
    //   RCC_CLOCKTYPE_PCLK1            APB1 (rcc_pclk1)
    //   RCC_CLOCKTYPE_PCLK2            APB2 (rcc_pclk2)
    //   RCC_CLOCKTYPE_D3PCLK1          APB4 (rcc_pclk4)

    RCC_ClkInitStruct.ClockType = ( \
        RCC_CLOCKTYPE_SYSCLK | \
        RCC_CLOCKTYPE_HCLK | \
        RCC_CLOCKTYPE_D1PCLK1 | \
        RCC_CLOCKTYPE_PCLK1 | \
        RCC_CLOCKTYPE_PCLK2  | \
        RCC_CLOCKTYPE_D3PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.AHBCLKDivider = MCU_HCLK_DIVIDER;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, MCU_FLASH_LATENCY) != HAL_OK) {
        /* Initialization Error */
        ErrorHandler();
    }

    /* -4- Optional: Disable CSI Oscillator (if the HSI is no more needed by the application)*/
    RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_CSI;
    RCC_OscInitStruct.CSIState        = RCC_CSI_OFF;
    RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        /* Initialization Error */
        ErrorHandler();
    }
}

void SystemClock_Config(void)
{
    // Configure power supply

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H7A3xx)
    // Legacy H7 devices (H743, H750) and newer but SMPS-less devices(H7A3, H723)

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    // Pre-configure voltage scale to PWR_REGULATOR_VOLTAGE_SCALE1.
    // SystemClockHSE_Config may configure PWR_REGULATOR_VOLTAGE_SCALE0.

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#elif defined(STM32H7A3xxQ) || defined(STM32H725xx)

    // We assume all SMPS equipped devices use this mode (Direct SMPS).
    // - All STM32H7A3xxQ devices.
    // - All STM32H725xx devices (Note STM32H725RG is Direct SMPS only - no LDO).
    //
    // Note that:
    // - Nucleo-H7A3ZI-Q is preconfigured for power supply configuration 2 (Direct SMPS).
    // - Nucleo-H723ZI-Q transplanted with STM32H725ZG is the same as above.

    HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

#else
#error Unknown MCU
#endif

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
        // Empty
    }

    SystemClockHSE_Config();

    /*activate CSI clock mondatory for I/O Compensation Cell*/

    __HAL_RCC_CSI_ENABLE() ;

    /* Enable SYSCFG clock mondatory for I/O Compensation Cell */

    __HAL_RCC_SYSCFG_CLK_ENABLE() ;

    /* Enables the I/O Compensation Cell */

    HAL_EnableCompensationCell();

    HandleStuckSysTick();

    HAL_Delay(10);

    // Configure peripheral clocks

    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

    // Configure HSI48 as peripheral clock for USB

    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    RCC_PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    // Configure CRS for dynamic calibration of HSI48
    // While ES0392 Rev 5 "STM32H742xI/G and STM32H743xI/G device limitations" states CRS not working for REV.Y,
    // it is always turned on as it seems that it has no negative effect on clock accuracy.

    RCC_CRSInitTypeDef crsInit = {
        .Prescaler = RCC_CRS_SYNC_DIV1,
        .Source = MCU_RCC_CRS_SYNC_SOURCE,
        .Polarity = RCC_CRS_SYNC_POLARITY_RISING,
        .ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT,
        .ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT,
        .HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT,
    };

    __HAL_RCC_CRS_CLK_ENABLE();
    HAL_RCCEx_CRSConfig(&crsInit);

#ifdef USE_CRS_INTERRUPTS
    // Turn on USE_CRS_INTERRUPTS to see CRS in action
    HAL_NVIC_SetPriority(CRS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CRS_IRQn);
    __HAL_RCC_CRS_ENABLE_IT(RCC_CRS_IT_SYNCOK|RCC_CRS_IT_SYNCWARN|RCC_CRS_IT_ESYNC|RCC_CRS_IT_ERR);
#endif

    // Configure UART peripheral clock sources
    //
    // Possible sources:
    //   D2PCLK1 (pclk1 for APB1 = USART234578)
    //   D2PCLK2 (pclk2 for APB2 = USART16)
    //   PLL2 (pll2_q_ck)
    //   PLL3 (pll3_q_ck),
    //   HSI (hsi_ck),
    //   CSI (csi_ck),LSE(lse_ck);

    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART16|RCC_PERIPHCLK_USART234578;
    RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    RCC_PeriphClkInit.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    // Configure SPI peripheral clock sources
    //
    // Possible sources for SPI123:
    //   PLL (pll1_q_ck)
    //   PLL2 (pll2_p_ck)
    //   PLL3 (pll3_p_ck)
    //   PIN (I2S_CKIN)
    //   CLKP (per_ck)
    // Possible sources for SPI45:
    //   D2PCLK1 (rcc_pclk2 = APB1) 100MHz
    //   PLL2 (pll2_q_ck)
    //   PLL3 (pll3_q_ck)
    //   HSI (hsi_ker_ck)
    //   CSI (csi_ker_ck)
    //   HSE (hse_ck)
    // Possible sources for SPI6:
    //   D3PCLK1 (rcc_pclk4 = APB4) 100MHz
    //   PLL2 (pll2_q_ck)
    //   PLL3 (pll3_q_ck)
    //   HSI (hsi_ker_ck)
    //   CSI (csi_ker_ck)
    //   HSE (hse_ck)

    // We use 100MHz for Rev.Y and 120MHz for Rev.V from various sources

    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI123|RCC_PERIPHCLK_SPI45|RCC_PERIPHCLK_SPI6;
    RCC_PeriphClkInit.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    RCC_PeriphClkInit.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
    RCC_PeriphClkInit.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    // Configure I2C peripheral clock sources
    //
    // Current source for I2C123:
    //   D2PCLK1 (rcc_pclk1 = APB1 peripheral clock)
    //
    // Current source for I2C4:
    //   D3PCLK1 (rcc_pclk4 = APB4 peripheral clock)
    //
    // Note that peripheral clock determination in bus_i2c_hal_init.c must be modified when the sources are modified.

    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C123|RCC_PERIPHCLK_I2C4;
    RCC_PeriphClkInit.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    RCC_PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

#ifdef USE_SDCARD_SDIO
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
    RCC_PeriphClkInit.PLL2.PLL2M = 5;
    RCC_PeriphClkInit.PLL2.PLL2N = 500;
    RCC_PeriphClkInit.PLL2.PLL2P = 2; // 500Mhz
    RCC_PeriphClkInit.PLL2.PLL2Q = 3; // 266Mhz - 133Mhz can be derived from this for for QSPI if flash chip supports the speed.
    RCC_PeriphClkInit.PLL2.PLL2R = 4; // 200Mhz HAL LIBS REQUIRE 200MHZ SDMMC CLOCK, see HAL_SD_ConfigWideBusOperation, SDMMC_HSpeed_CLK_DIV, SDMMC_NSpeed_CLK_DIV
    RCC_PeriphClkInit.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
    RCC_PeriphClkInit.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    RCC_PeriphClkInit.PLL2.PLL2FRACN = 0;
    RCC_PeriphClkInit.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
#endif

    // Configure MCO clocks for clock test/verification

    // Possible sources for MCO1:
    //   RCC_MCO1SOURCE_HSI (hsi_ck)
    //   RCC_MCO1SOURCE_LSE (?)
    //   RCC_MCO1SOURCE_HSE (hse_ck)
    //   RCC_MCO1SOURCE_PLL1QCLK (pll1_q_ck)
    //   RCC_MCO1SOURCE_HSI48 (hsi48_ck)

    //  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);     // HSE(8M) / 1 = 1M
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);     // HSI48(48M) / 4 = 12M

    // Possible sources for MCO2:
    //   RCC_MCO2SOURCE_SYSCLK  (sys_ck)
    //   RCC_MCO2SOURCE_PLL2PCLK (pll2_p_ck)
    //   RCC_MCO2SOURCE_HSE (hse_ck)
    //   RCC_MCO2SOURCE_PLLCLK (pll1_p_ck)
    //   RCC_MCO2SOURCE_CSICLK (csi_ck)
    //   RCC_MCO2SOURCE_LSICLK (lsi_ck)

    HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLCLK, RCC_MCODIV_15); // PLL1P(400M) / 15 = 26.67M
}

#ifdef USE_CRS_INTERRUPTS
static uint32_t crs_syncok = 0;
static uint32_t crs_syncwarn = 0;
static uint32_t crs_expectedsync = 0;
static uint32_t crs_error = 0;

void HAL_RCCEx_CRS_SyncOkCallback(void)
{
    ++crs_syncok;
}

void HAL_RCCEx_CRS_SyncWarnCallback(void)
{
    ++crs_syncwarn;
}

void HAL_RCCEx_CRS_ExpectedSyncCallback(void)
{
    ++crs_expectedsync;
}

void HAL_RCCEx_CRS_ErrorCallback(uint32_t Error)
{
    ++crs_error;
}

void CRS_IRQHandler(void)
{
    HAL_RCCEx_CRS_IRQHandler();
}
#endif

#include "build/debug.h"

void systemCheckResetReason(void);

#include "drivers/memprot.h"

void SystemInit (void)
{
    memProtReset();

    initialiseMemorySections();

#if !defined(USE_EXST)
    // only stand-alone and bootloader firmware needs to do this.
    // if it's done in the EXST firmware as well as the BOOTLOADER firmware you get a reset loop.
    systemCheckResetReason();
#endif

    // FPU settings
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // Set CP10 and CP11 Full Access
#endif

    // Reset the RCC clock configuration to the default reset state
    // Set HSION bit
    RCC->CR = RCC_CR_HSION;

    // Reset CFGR register
    RCC->CFGR = 0x00000000;

    // Reset HSEON, CSSON , CSION,RC48ON, CSIKERON PLL1ON, PLL2ON and PLL3ON bits

    // XXX Don't do this until we are established with clock handling
    // RCC->CR &= (uint32_t)0xEAF6ED7F;

    // Instead, we explicitly turn those on
    RCC->CR |= RCC_CR_CSION;
    RCC->CR |= RCC_CR_HSION;
    RCC->CR |= RCC_CR_HSEON;
    RCC->CR |= RCC_CR_HSI48ON;

#if defined(STM32H743xx) || defined(STM32H750xx)
    /* Reset D1CFGR register */
    RCC->D1CFGR = 0x00000000;

    /* Reset D2CFGR register */
    RCC->D2CFGR = 0x00000000;

    /* Reset D3CFGR register */
    RCC->D3CFGR = 0x00000000;
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
  /* Reset CDCFGR1 register */
  RCC->CDCFGR1 = 0x00000000;

  /* Reset CDCFGR2 register */
  RCC->CDCFGR2 = 0x00000000;

  /* Reset SRDCFGR register */
  RCC->SRDCFGR = 0x00000000;
#endif

    /* Reset PLLCKSELR register */
    RCC->PLLCKSELR = 0x00000000;

    /* Reset PLLCFGR register */
    RCC->PLLCFGR = 0x00000000;
    /* Reset PLL1DIVR register */
    RCC->PLL1DIVR = 0x00000000;
    /* Reset PLL1FRACR register */
    RCC->PLL1FRACR = 0x00000000;

    /* Reset PLL2DIVR register */
    RCC->PLL2DIVR = 0x00000000;

    /* Reset PLL2FRACR register */

    RCC->PLL2FRACR = 0x00000000;
    /* Reset PLL3DIVR register */
    RCC->PLL3DIVR = 0x00000000;

    /* Reset PLL3FRACR register */
    RCC->PLL3FRACR = 0x00000000;

    /* Reset HSEBYP bit */
    RCC->CR &= (uint32_t)0xFFFBFFFF;

    /* Disable all interrupts */
    RCC->CIER = 0x00000000;

    /* Change  the switch matrix read issuing capability to 1 for the AXI SRAM target (Target 7) */
  *((__IO uint32_t*)0x51008108) = 0x00000001;

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
    SystemInit_ExtMemCtl();
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

    /* Configure the Vector Table location add offset address ------------------*/
#if defined(VECT_TAB_SRAM)
#if defined(STM32H743xx) || defined(STM32H750xx)
    SCB->VTOR = D1_AXISRAM_BASE  | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal ITCMSRAM */
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
    SCB->VTOR = CD_AXISRAM_BASE  | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal ITCMSRAM */
#else
#error Unknown MCU type
#endif
#elif defined(USE_EXST)
    extern void *isr_vector_table_base;

    SCB->VTOR = (uint32_t)&isr_vector_table_base;
#else
    SCB->VTOR = FLASH_BANK1_BASE | VECT_TAB_OFFSET;       /* Vector Table Relocation in Internal FLASH */
#endif

#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

    SystemClock_Config();
    SystemCoreClockUpdate();

#ifdef STM32H7
    initialiseD2MemorySections();
#endif

    // Configure MPU

    memProtConfigure(mpuRegions, mpuRegionCount);

    // Enable CPU L1-Cache
    SCB_EnableICache();
    SCB_EnableDCache();
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock , it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is CSI, SystemCoreClock will contain the CSI_VALUE(*)
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(**)
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(***)
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the CSI_VALUE(*),
  *             HSI_VALUE(**) or HSE_VALUE(***) multiplied/divided by the PLL factors.
  *
  *         (*) CSI_VALUE is a constant defined in stm32h7xx_hal.h file (default value
  *             4 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *         (**) HSI_VALUE is a constant defined in stm32h7xx_hal.h file (default value
  *             64 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (***)HSE_VALUE is a constant defined in stm32h7xx_hal.h file (default value
  *              25 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  * @param  None
  * @retval None
  */

void SystemCoreClockUpdate (void)
{
    SystemCoreClock = HAL_RCC_GetSysClockFreq();
}

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32h7xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
#if defined (DATA_IN_ExtSDRAM)
    register uint32_t tmpreg = 0, timeout = 0xFFFF;
    register __IO uint32_t index;

    /* Enable GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface clock */
    RCC->AHB4ENR |= 0x000001F8;
    /* Connect PDx pins to FMC Alternate function */
    GPIOD->AFR[0]  = 0x000000CC;
    GPIOD->AFR[1]  = 0xCC000CCC;
    /* Configure PDx pins in Alternate function mode */
    GPIOD->MODER   = 0xAFEAFFFA;
    /* Configure PDx pins speed to 50 MHz */
    GPIOD->OSPEEDR = 0xA02A000A;
    /* Configure PDx pins Output type to push-pull */
    GPIOD->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PDx pins */
     GPIOD->PUPDR   = 0x55555505;
    /* Connect PEx pins to FMC Alternate function */
    GPIOE->AFR[0]  = 0xC00000CC;
    GPIOE->AFR[1]  = 0xCCCCCCCC;
      /* Configure PEx pins in Alternate function mode */
    GPIOE->MODER   = 0xAAAABFFA;
    /* Configure PEx pins speed to 50 MHz */
    GPIOE->OSPEEDR = 0xAAAA800A;
    /* Configure PEx pins Output type to push-pull */
    GPIOE->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PEx pins */
    GPIOE->PUPDR   = 0x55554005;
    /* Connect PFx pins to FMC Alternate function */
    GPIOF->AFR[0]  = 0x00CCCCCC;
    GPIOF->AFR[1]  = 0xCCCCC000;
    /* Configure PFx pins in Alternate function mode */
    GPIOF->MODER   = 0xAABFFAAA;
    /* Configure PFx pins speed to 50 MHz */
    GPIOF->OSPEEDR = 0xAA800AAA;
    /* Configure PFx pins Output type to push-pull */
    GPIOF->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PFx pins */
    GPIOF->PUPDR   = 0x55400555;
    /* Connect PGx pins to FMC Alternate function */
    GPIOG->AFR[0]  = 0x00CCCCCC;
    GPIOG->AFR[1]  = 0xC000000C;
    /* Configure PGx pins in Alternate function mode */
    GPIOG->MODER   = 0xBFFEFAAA;
 /* Configure PGx pins speed to 50 MHz */
    GPIOG->OSPEEDR = 0x80020AAA;
    /* Configure PGx pins Output type to push-pull */
    GPIOG->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PGx pins */
    GPIOG->PUPDR   = 0x40010515;
    /* Connect PHx pins to FMC Alternate function */
    GPIOH->AFR[0]  = 0xCCC00000;
    GPIOH->AFR[1]  = 0xCCCCCCCC;
    /* Configure PHx pins in Alternate function mode */
    GPIOH->MODER   = 0xAAAAABFF;
    /* Configure PHx pins speed to 50 MHz */
    GPIOH->OSPEEDR = 0xAAAAA800;
    /* Configure PHx pins Output type to push-pull */
    GPIOH->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PHx pins */
    GPIOH->PUPDR   = 0x55555400;
    /* Connect PIx pins to FMC Alternate function */
    GPIOI->AFR[0]  = 0xCCCCCCCC;
    GPIOI->AFR[1]  = 0x00000CC0;
    /* Configure PIx pins in Alternate function mode */
    GPIOI->MODER   = 0xFFEBAAAA;
    /* Configure PIx pins speed to 50 MHz */
    GPIOI->OSPEEDR = 0x0028AAAA;
    /* Configure PIx pins Output type to push-pull */
    GPIOI->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PIx pins */
    GPIOI->PUPDR   = 0x00145555;
/*-- FMC Configuration ------------------------------------------------------*/
    /* Enable the FMC interface clock */
    (RCC->AHB3ENR |= (RCC_AHB3ENR_FMCEN));
    /*SDRAM Timing and access interface configuration*/
    /*LoadToActiveDelay  = 2
      ExitSelfRefreshDelay = 6
      SelfRefreshTime      = 4
      RowCycleDelay        = 6
      WriteRecoveryTime    = 2
      RPDelay              = 2
      RCDDelay             = 2
      SDBank             = FMC_SDRAM_BANK2
      ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_9
      RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12
      MemoryDataWidth    = FMC_SDRAM_MEM_BUS_WIDTH_32
      InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4
      CASLatency         = FMC_SDRAM_CAS_LATENCY_2
      WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE
      SDClockPeriod      = FMC_SDRAM_CLOCK_PERIOD_2
      ReadBurst          = FMC_SDRAM_RBURST_ENABLE
      ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0*/

    FMC_Bank5_6->SDCR[0] = 0x00001800;
    FMC_Bank5_6->SDCR[1] = 0x00000165;
    FMC_Bank5_6->SDTR[0] = 0x00105000;
    FMC_Bank5_6->SDTR[1] = 0x01010351;

    /* SDRAM initialization sequence */
    /* Clock enable command */
    FMC_Bank5_6->SDCMR = 0x00000009;
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
    while((tmpreg != 0) && (timeout-- > 0))
    {
      tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
    }

    /* Delay */
    for (index = 0; index<1000; index++);

    /* PALL command */
      FMC_Bank5_6->SDCMR = 0x0000000A;
    timeout = 0xFFFF;
    while((tmpreg != 0) && (timeout-- > 0))
    {
      tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
    }

    FMC_Bank5_6->SDCMR = 0x000000EB;
    timeout = 0xFFFF;
    while((tmpreg != 0) && (timeout-- > 0))
    {
      tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
    }

    FMC_Bank5_6->SDCMR = 0x0004400C;
    timeout = 0xFFFF;
    while((tmpreg != 0) && (timeout-- > 0))
    {
      tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
    }
    /* Set refresh count */
    tmpreg = FMC_Bank5_6->SDRTR;
    FMC_Bank5_6->SDRTR = (tmpreg | (0x00000603<<1));

    /* Disable write protection */
    tmpreg = FMC_Bank5_6->SDCR[1];
    FMC_Bank5_6->SDCR[1] = (tmpreg & 0xFFFFFDFF);

    /*FMC controller Enable*/
    FMC_Bank1->BTCR[0]  |= 0x80000000;


#endif /* DATA_IN_ExtSDRAM */

#if defined(DATA_IN_ExtSRAM)
/*-- GPIOs Configuration -----------------------------------------------------*/
     /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
    RCC->AHB4ENR   |= 0x00000078;

    /* Connect PDx pins to FMC Alternate function */
    GPIOD->AFR[0]  = 0x00CCC0CC;
    GPIOD->AFR[1]  = 0xCCCCCCCC;
    /* Configure PDx pins in Alternate function mode */
    GPIOD->MODER   = 0xAAAA0A8A;
    /* Configure PDx pins speed to 100 MHz */
    GPIOD->OSPEEDR = 0xFFFF0FCF;
    /* Configure PDx pins Output type to push-pull */
    GPIOD->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PDx pins */
    GPIOD->PUPDR   = 0x55550545;

    /* Connect PEx pins to FMC Alternate function */
    GPIOE->AFR[0]  = 0xC00CC0CC;
    GPIOE->AFR[1]  = 0xCCCCCCCC;
    /* Configure PEx pins in Alternate function mode */
    GPIOE->MODER   = 0xAAAA828A;
    /* Configure PEx pins speed to 100 MHz */
    GPIOE->OSPEEDR = 0xFFFFC3CF;
    /* Configure PEx pins Output type to push-pull */
    GPIOE->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PEx pins */
    GPIOE->PUPDR   = 0x55554145;

    /* Connect PFx pins to FMC Alternate function */
    GPIOF->AFR[0]  = 0x00CCCCCC;
    GPIOF->AFR[1]  = 0xCCCC0000;
    /* Configure PFx pins in Alternate function mode */
    GPIOF->MODER   = 0xAA000AAA;
    /* Configure PFx pins speed to 100 MHz */
    GPIOF->OSPEEDR = 0xFF000FFF;
    /* Configure PFx pins Output type to push-pull */
    GPIOF->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PFx pins */
    GPIOF->PUPDR   = 0x55000555;

    /* Connect PGx pins to FMC Alternate function */
    GPIOG->AFR[0]  = 0x00CCCCCC;
    GPIOG->AFR[1]  = 0x000000C0;
    /* Configure PGx pins in Alternate function mode */
    GPIOG->MODER   = 0x00200AAA;
    /* Configure PGx pins speed to 100 MHz */
    GPIOG->OSPEEDR = 0x00300FFF;
    /* Configure PGx pins Output type to push-pull */
    GPIOG->OTYPER  = 0x00000000;
    /* No pull-up, pull-down for PGx pins */
    GPIOG->PUPDR   = 0x00100555;

/*-- FMC/FSMC Configuration --------------------------------------------------*/
    /* Enable the FMC/FSMC interface clock */
    (RCC->AHB3ENR |= (RCC_AHB3ENR_FMCEN));

    /* Configure and enable Bank1_SRAM2 */
    FMC_Bank1->BTCR[4]  = 0x00001091;
    FMC_Bank1->BTCR[5]  = 0x00110212;
    FMC_Bank1E->BWTR[4] = 0x0FFFFFFF;

    /*FMC controller Enable*/
    FMC_Bank1->BTCR[0]  |= 0x80000000;

#endif /* DATA_IN_ExtSRAM */
}
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
