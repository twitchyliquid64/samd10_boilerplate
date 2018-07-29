#include "samd10.h"

#define ADC_TEMP_SAMPLE_LENGTH            4

// Calibration rows.
float tempR;        /* Production Room Temperature value read from NVM memory - tempR */
float tempH;        /* Production Hot Temperature value read from NVM memory - tempH */
uint16_t ADCR;      /* Production Room Temperature ADC Value read from NVM memory - ADCR */
uint16_t ADCH;      /* Production Hot Temperature ADC Value read from NVM memory - ADCH */

void load_calibration_data(void);

void setup_adc(){
  load_calibration_data();

  // enable ADC clocking (enabled by default IIRC).
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;
  // enable temperature sensor reference.
  SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN;

  // first, set the clock divider to 1.
  GCLK->GENDIV.reg = ((GCLK_GENCTRL_ID(4)) | (GCLK_GENDIV_DIV(1)));
  // connect generic clock generator 4 to OSC8M and enable it.
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
  ADC->SAMPCTRL.reg = ADC_TEMP_SAMPLE_LENGTH;
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




/**
* \brief Decimal to Fraction Conversation.
* This function converts the decimal value into fractional
* and return the fractional value for temperature calculation.
* Source: https://github.com/adafruit/circuitpython/issues/177
*/
float convert_dec_to_frac(uint8_t val)
{
  if (val < 10)
  {
    return ((float)val/10.0);
  }
  else if (val <100)
  {
    return ((float)val/100.0);
  }
  else
  {
    return ((float)val/1000.0);
  }
}

/**
* \brief Calibration Data.
* This function extract the production calibration data information from
* Temperature log row content and store it variables for temperature calculation
* Source: https://github.com/adafruit/circuitpython/issues/177
*/
void load_calibration_data(void)
{
  volatile uint32_t val1;       /* Temperature Log Row Content first 32 bits */
  volatile uint32_t val2;       /* Temperature Log Row Content another 32 bits */
  uint8_t room_temp_val_int;      /* Integer part of room temperature in °C */
  uint8_t room_temp_val_dec;      /* Decimal part of room temperature in °C */
  uint8_t hot_temp_val_int;     /* Integer part of hot temperature in °C */
  uint8_t hot_temp_val_dec;     /* Decimal part of hot temperature in °C */
  //int8_t room_int1v_val;        /* internal 1V reference drift at room temperature */
  //int8_t hot_int1v_val;       /* internal 1V reference drift at hot temperature*/

  uint32_t *temp_log_row_ptr = (uint32_t *)NVMCTRL_TEMP_LOG;

  val1 = *temp_log_row_ptr;
  temp_log_row_ptr++;
  val2 = *temp_log_row_ptr;

  room_temp_val_int = (uint8_t)((val1 & FUSES_ROOM_TEMP_VAL_INT_Msk) >> FUSES_ROOM_TEMP_VAL_INT_Pos);
  room_temp_val_dec = (uint8_t)((val1 & FUSES_ROOM_TEMP_VAL_DEC_Msk) >> FUSES_ROOM_TEMP_VAL_DEC_Pos);
  hot_temp_val_int = (uint8_t)((val1 & FUSES_HOT_TEMP_VAL_INT_Msk) >> FUSES_HOT_TEMP_VAL_INT_Pos);
  hot_temp_val_dec = (uint8_t)((val1 & FUSES_HOT_TEMP_VAL_DEC_Msk) >> FUSES_HOT_TEMP_VAL_DEC_Pos);
  //room_int1v_val = (int8_t)((val1 & FUSES_ROOM_INT1V_VAL_Msk) >> FUSES_ROOM_INT1V_VAL_Pos);
  //hot_int1v_val = (int8_t)((val2 & FUSES_HOT_INT1V_VAL_Msk) >> FUSES_HOT_INT1V_VAL_Pos);

  ADCR = (uint16_t)((val2 & FUSES_ROOM_ADC_VAL_Msk) >> FUSES_ROOM_ADC_VAL_Pos);
  ADCH = (uint16_t)((val2 & FUSES_HOT_ADC_VAL_Msk) >> FUSES_HOT_ADC_VAL_Pos);

  tempR = room_temp_val_int + convert_dec_to_frac(room_temp_val_dec);
  tempH = hot_temp_val_int + convert_dec_to_frac(hot_temp_val_dec);
}
