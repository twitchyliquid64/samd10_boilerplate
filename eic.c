#include "samd10.h"
#include "uart.h"

void setup_eic(){
  // setup_rtc() should have connected 32k internal osc to GCLK 2.
  // connect GCLK 2 to EIC.
  // use generic clock generator 2 + enable it + connect it to the EIC peripheral.
  GCLK->CLKCTRL.reg = ((GCLK_CLKCTRL_GEN_GCLK2) | (GCLK_CLKCTRL_CLKEN) | (GCLK_CLKCTRL_ID(EIC_GCLK_ID)));

  // PA_25 is EXTINT5. Configure filtering + rising edge.
  EIC->CONFIG[0].reg = ((EIC_CONFIG_FILTEN5) | (EIC_CONFIG_SENSE5_RISE));
  // enable interrupts for EXTINT5.
  EIC->INTENSET.reg = ((EIC_INTENSET_EXTINT5));
  // Synchronize.
  while (EIC->STATUS.bit.SYNCBUSY);

  NVIC_SetPriority(EIC_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
  NVIC_EnableIRQ(EIC_IRQn);

  // Enable the peripheral.
  EIC->CTRL.reg = ((EIC_CTRL_ENABLE));
  // Synchronize.
  while (EIC->STATUS.bit.SYNCBUSY);
}


void irq_handler_eic(void) {
  if (EIC->INTFLAG.bit.EXTINT5) {
    EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT5;
    PORT->Group[0].OUTSET.reg |= PORT_PA24;
    //uart_puts("HELLO!\r\n");
  }
}
