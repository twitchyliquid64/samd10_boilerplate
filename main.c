#include "samd10.h"
#include "busyloop.h"
#include "rtc.h"
#include "tc.h"
#include "eic.h"
#include "adc.h"
#include "uart.h"
#include "i2c.h"
#include "circ_buffer.h"

extern circBuf_t uartRecvBuff;

void configure_system_clock() {
  // enable external oscilator, crystal mode, gain set sanely for just under 30Mhz, enable-on-standby, AGC enabled
  SYSCTRL->XOSC.reg = ((SYSCTRL_XOSC_ENABLE) | (SYSCTRL_XOSC_RUNSTDBY) | (SYSCTRL_XOSC_XTALEN) | SYSCTRL_XOSC_STARTUP(0xA) | SYSCTRL_XOSC_GAIN(0x3) | (SYSCTRL_XOSC_AMPGC));
  while (!SYSCTRL->PCLKSR.bit.XOSCRDY);

  // first, set the clock divider to 1.
  GCLK->GENDIV.reg = ((GCLK_GENCTRL_ID(0)) | (GCLK_GENDIV_DIV(1)));
  // connect generic clock generator 0 to XOSC and enable it.
  GCLK->GENCTRL.reg = ((GCLK_GENCTRL_ID(0)) | (GCLK_GENCTRL_SRC_XOSC) | (GCLK_GENCTRL_GENEN));
}

int main(void) {
  configure_system_clock();
  setup_rtc();
  setup_tc1();
  setup_eic();
  setup_adc();
  setup_uart();
  setup_i2c();

  // Setup LED on PA24 for blinky-boi.
  PORT->Group[0].DIRSET.reg |= PORT_PA24;
  PORT->Group[0].OUTSET.reg |= PORT_PA24;

  // Setup pin on PA25 as an input. PULLEN is not set (no pullups).
  PORT->Group[0].PINCFG[25].reg = ((PORT_PINCFG_INEN) | (PORT_PINCFG_PMUXEN));
  // Setup continous sampling on the group of pins to shave 2 clock cycles off a read.
  PORT->Group[0].CTRL.reg |= PORT_PA25;
  // Setup pin mux function for pin 25 (2*12 + 1, AKA odd) to function A, which corresponds to external interrupt 5.
  PORT->Group[0].PMUX[12].reg = ((PORT_PMUX_PMUXO_A));

  uart_puts("Booted.\r\n");

  while(1) {
    delay_ms_rtc(1);

    UART_RX_ISR_DISABLE();
    if (circBufLen(&uartRecvBuff) > 2) {
      uint8_t buff[4];
      int copied = circBufCopy(&uartRecvBuff,(uint8_t *) &buff, 3);
      buff[3] = '\0';

      // Yes I know this is a wierd way to do it.
      if (copied == 3 && buff[0] == 'r' && buff[1] == 'e' && buff[2] == 'd')
        PORT->Group[0].OUTSET.reg |= PORT_PA24;
      if (copied == 3 && buff[0] == 'o' && buff[1] == 'f' && buff[2] == 'f')
        PORT->Group[0].OUTCLR.reg |= PORT_PA24;
    }
    UART_RX_ISR_ENABLE();


    //if ((PORT->Group[0].IN.reg & PORT_PA25) != 0) {
    //  PORT->Group[0].OUTSET.reg |= PORT_PA24;
    //}
  }
}
