#include "samd10.h"
#include "busyloop.h"
#include "rtc.h"

int main(void) {
  setup_rtc();

  PORT->Group[0].DIRSET.reg |= PORT_PA24;
  PORT->Group[0].OUTSET.reg |= PORT_PA24;
  while(1) {
    delay_ms_rtc(1000);
    PORT->Group[0].OUTTGL.reg |= PORT_PA24;
  }
}
