#include "samd10.h"

void setup_tc1() {
  // enable the clock gate to the peripheral.
  PM->APBCMASK.reg |= PM_APBCMASK_TC1;

  // setup_rtc() should have connected 32k internal osc to GCLK 2.
  // connect GCLK 2 to TC1.
  // use generic clock generator 2 + enable it + connect it to the TC1 module.
  GCLK->CLKCTRL.reg = ((GCLK_CLKCTRL_GEN_GCLK2) | (GCLK_CLKCTRL_CLKEN) | (GCLK_CLKCTRL_ID(TC1_GCLK_ID)));

  // set 16bit mode, no prescaling.
  // WAVEGEN_MFRQ is selected, meaning the counter will count till the value in CC0 before overflowing.
  TC1->COUNT16.CTRLA.reg = ((TC_CTRLA_MODE_COUNT16) | (TC_CTRLA_WAVEGEN_MFRQ) | (TC_CTRLA_PRESCALER_DIV1));
  // wait for synchronization.
  while (TC1->COUNT16.STATUS.bit.SYNCBUSY);

  // set current count value to 0.
  TC1->COUNT16.COUNT.reg = 0;
  // wait for synchronization.
  while (TC1->COUNT16.STATUS.bit.SYNCBUSY);

  // set capture compare value to 13999.
  TC1->COUNT16.CC[0].reg = 13999;
  // wait for synchronization.
  while (TC1->COUNT16.STATUS.bit.SYNCBUSY);

  // enable interrupts on overflow.
  TC1->COUNT16.INTENSET.reg = ((TC_INTENCLR_OVF));

  NVIC_SetPriority(TC1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
  NVIC_EnableIRQ(TC1_IRQn);

  // enable TC1.
  TC1->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC1->COUNT16.STATUS.bit.SYNCBUSY);
}

void irq_handler_tc1() {
  if (TC1->COUNT16.INTFLAG.bit.OVF) {
    //PORT->Group[0].OUTTGL.reg |= PORT_PA24;
    TC1->COUNT16.INTFLAG.reg = TC_INTENCLR_OVF; // clear overflow
  }
}
