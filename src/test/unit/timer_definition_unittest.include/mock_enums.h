#pragma once

enum TestTimerEnum {
    TIM0, TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9,
    TIM10, TIM11, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM18, TIM19, TIM20,
    TEST_TIMER_SIZE,
};

enum TestChannelEnum {
    CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH9, CH10, CH1N, CH2N, CH3N,
    TEST_CHANNEL_SIZE,
};

// Keep this in sync with TEST_PIN_NAMES below.
enum TestPinEnum {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9,
    PA10, PA11, PA12, PA13, PA14, PA15,
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9,
    PB10, PB11, PB12, PB13, PB14, PB15,
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9,
    PC10, PC11, PC12, PC13, PC14, PC15,
    PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9,
    PD10, PD11, PD12, PD13, PD14, PD15,
    PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9,
    PE10, PE11, PE12, PE13, PE14, PE15,
    PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9,
    PF10, PF11, PF12, PF13, PF14, PF15,
    PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9,
    PG10, PG11, PG12, PG13, PG14, PG15,
    PH0, PH1, PH2, PH3, PH4, PH5, PH6, PH7, PH8, PH9,
    PH10, PH11, PH12, PH13, PH14, PH15, TEST_PIN_SIZE,
};

// Keep this in sync with TestPinEnum above.
const char *const TEST_PIN_NAMES[TEST_PIN_SIZE] = {
    "PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA9",
    "PA10", "PA11", "PA12", "PA13", "PA14", "PA15",
    "PB0", "PB1", "PB2", "PB3", "PB4", "PB5", "PB6", "PB7", "PB8", "PB9",
    "PB10", "PB11", "PB12", "PB13", "PB14", "PB15",
    "PC0", "PC1", "PC2", "PC3", "PC4", "PC5", "PC6", "PC7", "PC8", "PC9",
    "PC10", "PC11", "PC12", "PC13", "PC14", "PC15",
    "PD0", "PD1", "PD2", "PD3", "PD4", "PD5", "PD6", "PD7", "PD8", "PD9",
    "PD10", "PD11", "PD12", "PD13", "PD14", "PD15",
    "PE0", "PE1", "PE2", "PE3", "PE4", "PE5", "PE6", "PE7", "PE8", "PE9",
    "PE10", "PE11", "PE12", "PE13", "PE14", "PE15",
    "PF0", "PF1", "PF2", "PF3", "PF4", "PF5", "PF6", "PF7", "PF8", "PF9",
    "PF10", "PF11", "PF12", "PF13", "PF14", "PF15",
    "PG0", "PG1", "PG2", "PG3", "PG4", "PG5", "PG6", "PG7", "PG8", "PG9",
    "PG10", "PG11", "PG12", "PG13", "PG14", "PG15",
    "PH0", "PH1", "PH2", "PH3", "PH4", "PH5", "PH6", "PH7", "PH8", "PH9",
    "PH10", "PH11", "PH12", "PH13", "PH14", "PH15",
};

enum TestTimUseEnum {
    TIM_USE_ANY,
    TIM_USE_BEEPER,
    TIM_USE_CAMERA_CONTROL,
    TIM_USE_LED,
    TIM_USE_MOTOR,
    TIM_USE_NONE,
    TIM_USE_PPM,
    TIM_USE_PWM,
    TIM_USE_SERVO,
    TIM_USE_TRANSPONDER,
    TIM_USE_VIDEO_SYNC,
    TIM_USE_VIDEO_PIXEL,
    TEST_TIM_USE_SIZE,
};
