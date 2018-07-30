#include "samd10.h"

void setup_adc(){
  // enable ADC clocking (enabled by default IIRC).
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;
  // enable temperature sensor reference.
  SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN;

  // first, set the clock divider to 1.
  GCLK->GENDIV.reg = ((GCLK_GENCTRL_ID(4)) | (GCLK_GENDIV_DIV(1)));
  // connect generic clock generator 4 to OSC8M and enable it.
  // NOTE: By default SYSCTRL->0SC8m.PRESC == 0x3, which means /8 prescaler --> 1Mhz.
  GCLK->GENCTRL.reg = ((GCLK_GENCTRL_ID(4)) | (GCLK_GENCTRL_SRC_OSC8M) | (GCLK_GENCTRL_GENEN));
  // use generic clock generator 4 + enable it + connect it to the ADC module.
  GCLK->CLKCTRL.reg = ((GCLK_CLKCTRL_GEN_GCLK4) | (GCLK_CLKCTRL_CLKEN) | (GCLK_CLKCTRL_ID(ADC_GCLK_ID)));

  // configure ADC to have no gain, reference temperature, relative to internal ground.
  ADC->INPUTCTRL.reg = ((ADC_INPUTCTRL_GAIN_1X) | (ADC_INPUTCTRL_MUXNEG_GND) | (ADC_INPUTCTRL_MUXPOS_TEMP));
  while(ADC->STATUS.bit.SYNCBUSY);
  // prescale the clock down by 128, use 12bit conversions.
  ADC->CTRLB.reg = ((ADC_CTRLB_PRESCALER_DIV128_Val) | (ADC_CTRLB_RESSEL_12BIT));
  while(ADC->STATUS.bit.SYNCBUSY);

  // set max sampling time.
  ADC->SAMPCTRL.reg = 4;
  while(ADC->STATUS.bit.SYNCBUSY);

  // take 4 samples at a time, adjust result by 0.
  ADC->AVGCTRL.reg = ((ADC_AVGCTRL_SAMPLENUM_4) | ADC_AVGCTRL_ADJRES(2));
  while(ADC->STATUS.bit.SYNCBUSY);
  // reference the ADC for temperature to 1V.
  ADC->REFCTRL.reg = ((ADC_REFCTRL_REFSEL_INT1V));
  while(ADC->STATUS.bit.SYNCBUSY);

  ADC->CTRLA.reg = ((ADC_CTRLA_ENABLE));
  while(ADC->STATUS.bit.SYNCBUSY);

  // trigger a conversion (first conversion after changing the sources needs to be thrown away).
  ADC->SWTRIG.reg = ((ADC_SWTRIG_START));
  while(ADC->STATUS.bit.SYNCBUSY);
  while(!ADC->INTFLAG.bit.RESRDY);
}



// TODO read and process calibration data. source: https://github.com/adafruit/circuitpython/issues/177
