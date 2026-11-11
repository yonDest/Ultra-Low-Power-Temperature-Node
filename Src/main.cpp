/**
 ******************************************************************************
 * @file    main.cpp
 * @author  Yonathan Desta
 * @brief   Ultra-low-power temperature node for the STM32L432KC (bare-metal)
 *
 * Wake cycle (every SAMPLE_PERIOD_S seconds):
 *
 *   RTC wakeup -> power ADC -> measure VREFINT + die temperature
 *              -> power ADC down -> report over LPUART1 -> Stop 2
 *
 * Design notes
 *   - CMSIS register access only: no HAL, no LL, no heap, integer math only.
 *   - Core runs from the 4 MHz MSI reset default at voltage range 2.
 *   - RTC (LSE) is the only wake source; LPUART1 is clocked from LSE so the
 *     baud rate is independent of the system clock.
 *   - Everything not needed during the active window is powered down before
 *     sleeping: ADC (deep power-down), temp sensor/VREFINT, LPUART, GPIO clocks.
 *   - Temperature uses the factory TS_CAL1/TS_CAL2 points, compensated for the
 *     actual VDDA measured through the internal VREFINT channel.
 *
 * Copyright (c) 2026 Yonathan Desta
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "stm32l432xx.h"

#include <stdint.h>

// ── Configuration ────────────────────────────────────────────────────────────
static constexpr uint32_t SAMPLE_PERIOD_S = 10;   // RTC wake period (1..65536 s)
static constexpr uint32_t ADC_AVERAGES    = 8;    // conversions averaged per channel
static constexpr uint32_t LPUART_BRR      = 874;  // 256 * 32768 / 9600 baud (LSE clock)

// ── Factory calibration data (DS11451, "Temperature sensor calibration values")
static const volatile uint16_t *const TS_CAL1_ADDR     = reinterpret_cast<const volatile uint16_t *>(0x1FFF75A8UL);
static const volatile uint16_t *const VREFINT_CAL_ADDR = reinterpret_cast<const volatile uint16_t *>(0x1FFF75AAUL);
static const volatile uint16_t *const TS_CAL2_ADDR     = reinterpret_cast<const volatile uint16_t *>(0x1FFF75CAUL);
static constexpr int32_t  TS_CAL1_TEMP_C = 30;
static constexpr int32_t  TS_CAL2_TEMP_C = 130;
static constexpr uint32_t CAL_VDDA_MV    = 3000;   // VDDA at which the factory values were taken

// ADC1 input channels
static constexpr uint32_t ADC_CH_VREFINT = 0;
static constexpr uint32_t ADC_CH_TEMP    = 17;

// ── ISR flag ─────────────────────────────────────────────────────────────────
static volatile uint8_t g_wake = 0;

// ── Small helpers ────────────────────────────────────────────────────────────

// Crude busy-wait: roughly 1 us per count at 4 MHz (conservative if slower).
static void delay_us(uint32_t us)
{
    while (us--) {
        __NOP();
    }
}

// Spin until (*reg & mask) matches `set`, or give up after `limit` polls.
static bool wait_flag(const volatile uint32_t *reg, uint32_t mask, bool set, uint32_t limit)
{
    while (limit--) {
        if (((*reg & mask) != 0U) == set) {
            return true;
        }
    }
    return false;
}

[[noreturn]] static void fatal(void)
{
    __disable_irq();
    for (;;) {
        __WFI();
    }
}

// ── Power / clocks ───────────────────────────────────────────────────────────
static void power_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;                       // allow the clock to reach PWR

    // MSI = 4 MHz is the reset default: lowest active current, nothing to configure.
    // Voltage scaling range 2 (<= 26 MHz) lowers run current vs. range 1.
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_1;
    if (!wait_flag(&PWR->SR2, PWR_SR2_VOSF, false, 100000)) {
        fatal();
    }

    // 0 wait states is valid at 4 MHz in range 2 (and is the reset default).
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0WS;
}

static void gpio_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;

    // PA2 = LPUART1_TX (AF8). Held as a push-pull output, idle-high, between reports.
    GPIOA->BSRR  = GPIO_BSRR_BS2;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE2) | GPIO_MODER_MODE2_0;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFUL << 8)) | (8UL << 8);

    // The only pins that are not analog at reset are the JTAG ones.
    // Park the unused ones; PA13/PA14 (SWD) are deliberately left alone.
    GPIOA->MODER |= GPIO_MODER_MODE15;                          // PA15 (JTDI)
    GPIOB->MODER |= GPIO_MODER_MODE3 | GPIO_MODER_MODE4;        // PB3 (JTDO), PB4 (NJTRST)

    RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;      // port config is retained with the clock off
}

// ── RTC wakeup (LSE, 1 Hz ck_spre, wakeup timer on EXTI line 20) ─────────────
static void rtc_wakeup_init(uint32_t period_s)
{
    PWR->CR1 |= PWR_CR1_DBP;                   // backup-domain write access
    (void)PWR->CR1;

    // RTCSEL is write-once until a backup-domain reset: recover from a stale source.
    const uint32_t sel = RCC->BDCR & RCC_BDCR_RTCSEL;
    if (sel != 0U && sel != RCC_BDCR_RTCSEL_0) {
        RCC->BDCR |= RCC_BDCR_BDRST;
        RCC->BDCR &= ~RCC_BDCR_BDRST;
    }

    RCC->BDCR |= RCC_BDCR_LSEON;
    if (!wait_flag(&RCC->BDCR, RCC_BDCR_LSERDY, true, 4000000)) {
        fatal();                               // no 32.768 kHz crystal running
    }
    RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCSEL_0;   // LSE
    RCC->BDCR |= RCC_BDCR_RTCEN;

    RTC->WPR = 0xCA;                           // unlock write protection
    RTC->WPR = 0x53;

    RTC->CR &= ~RTC_CR_WUTE;
    if (!wait_flag(&RTC->ISR, RTC_ISR_WUTWF, true, 100000)) {
        fatal();
    }
    RTC->WUTR = period_s - 1U;                 // counts at 1 Hz (default prescalers)
    RTC->CR   = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUCKSEL_2;       // ck_spre, 16-bit
    RTC->ISR  = ~(RTC_ISR_WUTF | RTC_ISR_INIT) | (RTC->ISR & RTC_ISR_INIT);
    RTC->CR  |= RTC_CR_WUTIE | RTC_CR_WUTE;

    RTC->WPR = 0xFF;                           // relock

    EXTI->RTSR1 |= EXTI_RTSR1_RT20;            // wakeup timer = EXTI line 20, rising edge
    EXTI->IMR1  |= EXTI_IMR1_IM20;
    NVIC_SetPriority(RTC_WKUP_IRQn, 3);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

extern "C" void RTC_WKUP_IRQHandler(void)
{
    if (RTC->ISR & RTC_ISR_WUTF) {
        RTC->ISR = ~(RTC_ISR_WUTF | RTC_ISR_INIT) | (RTC->ISR & RTC_ISR_INIT);   // rc_w0
        g_wake = 1;
    }
    EXTI->PR1 = EXTI_PR1_PIF20;                // write 1 to clear
}

// ── ADC: powered only for the duration of a measurement ──────────────────────
static void adc_power_on(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    (void)RCC->AHB2ENR;
    RCC->CCIPR |= RCC_CCIPR_ADCSEL;            // 0b11: SYSCLK as ADC kernel clock

    // Start the temperature sensor / VREFINT first so they settle during regulator startup.
    ADC1_COMMON->CCR |= ADC_CCR_TSEN | ADC_CCR_VREFEN;

    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    delay_us(150);                             // regulator startup (20 us) + sensor settling

    ADC1->CR &= ~ADC_CR_ADCALDIF;              // single-ended calibration
    ADC1->CR |= ADC_CR_ADCAL;
    if (!wait_flag(&ADC1->CR, ADC_CR_ADCAL, false, 100000)) {
        fatal();
    }

    ADC1->SMPR1 = (6UL << ADC_SMPR1_SMP0_Pos);     // VREFINT: 247.5 cycles (~62 us @ 4 MHz)
    ADC1->SMPR2 = (6UL << ADC_SMPR2_SMP17_Pos);    // temp sensor: 247.5 cycles (min 5 us)

    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    if (!wait_flag(&ADC1->ISR, ADC_ISR_ADRDY, true, 100000)) {
        fatal();
    }
}

static void adc_power_off(void)
{
    ADC1->CR |= ADC_CR_ADDIS;
    wait_flag(&ADC1->CR, ADC_CR_ADEN, false, 100000);
    ADC1_COMMON->CCR &= ~(ADC_CCR_TSEN | ADC_CCR_VREFEN);
    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_DEEPPWD;                // deep power-down (calibration is lost)
    RCC->AHB2ENR &= ~RCC_AHB2ENR_ADCEN;
}

static uint32_t adc_sample(uint32_t channel, uint32_t count)
{
    ADC1->SQR1 = (channel << ADC_SQR1_SQ1_Pos);    // sequence length 1
    uint32_t acc = 0;
    for (uint32_t i = 0; i < count; i++) {
        ADC1->ISR = ADC_ISR_EOC;
        ADC1->CR |= ADC_CR_ADSTART;
        if (!wait_flag(&ADC1->ISR, ADC_ISR_EOC, true, 100000)) {
            fatal();
        }
        acc += ADC1->DR;
    }
    return acc / count;
}

struct Reading {
    int32_t  centi_c;    // temperature, 0.01 degC
    uint32_t vdda_mv;    // supply, mV
};

static Reading measure(void)
{
    adc_power_on();
    const uint32_t vref_raw = adc_sample(ADC_CH_VREFINT, ADC_AVERAGES);
    const uint32_t ts_raw   = adc_sample(ADC_CH_TEMP, ADC_AVERAGES);
    adc_power_off();

    // VDDA from the internal reference, then scale the temp reading to the 3.0 V calibration basis.
    const uint32_t vdda_mv = (CAL_VDDA_MV * static_cast<uint32_t>(*VREFINT_CAL_ADDR)) / vref_raw;
    const int32_t  ts_3v   = static_cast<int32_t>((ts_raw * vdda_mv) / CAL_VDDA_MV);

    const int32_t cal1 = *TS_CAL1_ADDR;
    const int32_t cal2 = *TS_CAL2_ADDR;
    const int32_t span = (TS_CAL2_TEMP_C - TS_CAL1_TEMP_C) * 100;

    Reading r;
    r.centi_c = TS_CAL1_TEMP_C * 100 + (span * (ts_3v - cal1)) / (cal2 - cal1);
    r.vdda_mv = vdda_mv;
    return r;
}

// ── LPUART1 TX (9600 8N1, clocked from LSE, enabled only while reporting) ────
static void uart_init(void)
{
    RCC->CCIPR |= RCC_CCIPR_LPUART1SEL;        // 0b11: LSE kernel clock
}

static void uart_enable(void)
{
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    (void)RCC->APB1ENR2;
    LPUART1->BRR = LPUART_BRR;
    LPUART1->CR1 = USART_CR1_TE | USART_CR1_UE;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE2) | GPIO_MODER_MODE2_1;   // AF
}

static void uart_disable(void)
{
    wait_flag(&LPUART1->ISR, USART_ISR_TC, true, 1000000);
    LPUART1->CR1 = 0;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE2) | GPIO_MODER_MODE2_0;   // idle-high output
    RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN;
}

static void uart_putc(char c)
{
    wait_flag(&LPUART1->ISR, USART_ISR_TXE, true, 1000000);
    LPUART1->TDR = static_cast<uint8_t>(c);
}

static void uart_puts(const char *s)
{
    while (*s) {
        uart_putc(*s++);
    }
}

static void uart_put_u32(uint32_t v)
{
    char buf[10];
    int n = 0;
    do {
        buf[n++] = static_cast<char>('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);
    while (n--) {
        uart_putc(buf[n]);
    }
}

// Prints e.g. "T=24.31C VDDA=3297mV\r\n"
static void report(const Reading &r)
{
    uart_enable();
    uart_puts("T=");
    int32_t t = r.centi_c;
    if (t < 0) {
        uart_putc('-');
        t = -t;
    }
    uart_put_u32(static_cast<uint32_t>(t / 100));
    uart_putc('.');
    const uint32_t frac = static_cast<uint32_t>(t % 100);
    if (frac < 10U) {
        uart_putc('0');
    }
    uart_put_u32(frac);
    uart_puts("C VDDA=");
    uart_put_u32(r.vdda_mv);
    uart_puts("mV\r\n");
    uart_disable();
}

// ── Stop 2 ───────────────────────────────────────────────────────────────────
static void enter_stop2(void)
{
    PWR->SCR = PWR_SCR_CWUF;                   // clear stale wake-up flags
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS) | PWR_CR1_LPMS_STOP2;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();
    __ISB();
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    // Core resumes on MSI (STOPWUCK default): same 4 MHz clock, nothing to re-init.
}

// ── Application ──────────────────────────────────────────────────────────────
int main(void)
{
    SCB->CPACR |= (0xFUL << 20);               // FPU access (harmless if unused)
    __DSB();
    __ISB();

    power_init();
    gpio_init();
    uart_init();
    rtc_wakeup_init(SAMPLE_PERIOD_S);

    report(measure());                         // first sample at boot

    for (;;) {
        g_wake = 0;
        while (!g_wake) {                      // spurious wakes simply go back to sleep
            enter_stop2();
        }
        report(measure());
    }
}
