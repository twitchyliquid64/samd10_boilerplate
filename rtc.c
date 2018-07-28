#include "samd10.h"

volatile uint32_t millisecondcount = 0;

void setup_rtc(void) {
  // first, set the global clock divider to 1.
  GCLK->GENDIV.reg = ((GCLK_GENCTRL_ID(2)) | (GCLK_GENDIV_DIV(1)));
  // connect generic clock generator 2 to OSC32K and enable it.
  GCLK->GENCTRL.reg = ((GCLK_GENCTRL_ID(2)) | (GCLK_GENCTRL_SRC_OSC32K) | (GCLK_GENCTRL_GENEN));
  // use generic clock generator 2 + enable it + connect it to the RTC module.
  GCLK->CLKCTRL.reg = ((GCLK_CLKCTRL_GEN_GCLK2) | (GCLK_CLKCTRL_CLKEN) | (GCLK_CLKCTRL_ID(RTC_GCLK_ID)));

  // reset the RTC peripheral.
  RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_SWRST;
  // wait for reset complete.
  while (RTC->MODE0.CTRL.bit.SWRST);

  // clear settings, set to MODE0.
  RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_MODE_COUNT32;
  // enable clear-counter-when-match-comp0, prescaler set to 1.
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_MATCHCLR | RTC_MODE0_CTRL_PRESCALER_DIV1;
  // set comp0 to 32.
  RTC->MODE0.COMP[0].reg = 32;
  // synchronise on the counter register.
  RTC->MODE0.READREQ.reg |= RTC_READREQ_RCONT;
  // enable compare-match interrupts.
  RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
  // enable.
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
  // clear interrupts (properly enable).
  RTC->MODE0.INTFLAG.reg = 0xFF;

  NVIC_SetPriority (RTC_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
  NVIC_EnableIRQ(RTC_IRQn);
}

uint32_t RTC_get_count()
{
  RTC->MODE0.READREQ.reg |= RTC_READREQ_RREQ;
  return RTC->MODE0.COUNT.reg;
}

uint32_t millis()
{
  return millisecondcount;
}

void irq_handler_rtc(void)
{
  millisecondcount++;
  RTC->MODE0.INTFLAG.reg = 0xFF;  // Clear all possible interrupts (all modes, too!)
}

void delay_ms_rtc(uint32_t wait)
{
  uint32_t end = millis() + wait;

  while (millis() < end);
}
