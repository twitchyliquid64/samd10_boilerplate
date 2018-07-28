#include "samd10.h"
#include "busyloop.h"
#include "rtc.h"

void configure_system_clock() {
  // enable external oscilator, crystal mode, gain set sanely for just under 30Mhz, enable-on-standby, AGC enabled
  SYSCTRL->XOSC.reg = ((SYSCTRL_XOSC_ENABLE) | (SYSCTRL_XOSC_RUNSTDBY) | (SYSCTRL_XOSC_XTALEN) | SYSCTRL_XOSC_GAIN(0xB) | (SYSCTRL_XOSC_AMPGC));
  while (!SYSCTRL->PCLKSR.bit.XOSCRDY);

  // first, set the clock divider to 1.
  GCLK->GENDIV.reg = ((GCLK_GENCTRL_ID(0)) | (GCLK_GENDIV_DIV(1)));
  // connect generic clock generator 0 to XOSC and enable it.
  GCLK->GENCTRL.reg = ((GCLK_GENCTRL_ID(0)) | (GCLK_GENCTRL_SRC_XOSC) | (GCLK_GENCTRL_GENEN));
}

int main(void) {
  configure_system_clock();
  setup_rtc();

  // Setup LED on PA24 for blinky-boi.
  PORT->Group[0].DIRSET.reg |= PORT_PA24;
  PORT->Group[0].OUTSET.reg |= PORT_PA24;
  while(1) {
    delay_ms_busyloop(1000);
    PORT->Group[0].OUTTGL.reg |= PORT_PA24;
  }
}
