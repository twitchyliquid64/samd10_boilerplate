#include "samd10.h"

// For some reason its not included?
#include "instance/sercom2.h"
#define SERCOM2           ((Sercom   *)0x42001000UL) /**< \brief (SERCOM2) APB Base Address */

void setup_i2c(void)
{
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;

  // use generic clock generator 4 + enable it + connect it to the SERCOM 2 module.
  // CLKGEN 4 should have already been connected to OSC8M with a divider of 1.
  // NOTE: By default SYSCTRL->0SC8m.PRESC == 0x3, which means /8 prescaler --> 1Mhz.
  GCLK->CLKCTRL.reg = ((GCLK_CLKCTRL_GEN_GCLK4) | (GCLK_CLKCTRL_CLKEN) | (GCLK_CLKCTRL_ID(SERCOM2_GCLK_ID_CORE)));
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Set PA15 to use an alternative function.
  PORT->Group[0].PINCFG[15].reg = ((PORT_PINCFG_PMUXEN));
  // Set PA14 to use an alternative function.
  PORT->Group[0].PINCFG[14].reg = ((PORT_PINCFG_PMUXEN));
  // Setup PA15 to use peripheral function D (SERCOM2 PAD1). (7*2 + 1, AKA odd).
  // Setup PA14 to use peripheral function D (SERCOM2 PAD0). (7*2, AKA even).
  PORT->Group[0].PMUX[7].reg |= ((PORT_PMUX_PMUXO_D) | (PORT_PMUX_PMUXE_D));

  // Enable smart mode (ack send when DATA read).
  SERCOM2->I2CM.CTRLB.reg = ((SERCOM_I2CM_CTRLB_SMEN));
  while (SERCOM2->I2CM.SYNCBUSY.reg);

  // Set the Master Baud Rate. TODO: Document what this means.
  SERCOM2->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(48);
  while (SERCOM2->I2CM.SYNCBUSY.reg);

  // Enable I2C, set I2C Master mode, set 400-800ns hold time on SDA.
  SERCOM2->I2CM.CTRLA.reg = ((SERCOM_I2CM_CTRLA_ENABLE) | (SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) | (SERCOM_I2CM_CTRLA_SDAHOLD(3)));
  while (SERCOM2->I2CM.SYNCBUSY.reg);

  // Reset bus state --> idle.
  SERCOM2->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(1);
  while (SERCOM2->I2CM.SYNCBUSY.reg);
}

// NOTE: Does not handle error bits for multi-master setups.
int i2c_write(uint8_t addr, uint8_t *data, int size)
{
  SERCOM2->I2CM.ADDR.reg = ((SERCOM_I2CM_ADDR_ADDR(addr))); // bit 0 == 0 mean WRITE.
  while (0 == (SERCOM2->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB));

  if (SERCOM2->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK)
  {
    // Acknowledge, issuing a STOP.
    SERCOM2->I2CM.CTRLB.reg |= ((SERCOM_I2CM_CTRLB_CMD(3)));
    return -2;
  }

  for (int i = 0; i < size; i++)
  {
    // Trigger data transaction by writing the first byte.
    SERCOM2->I2CM.DATA.reg = data[i];
    while (0 == (SERCOM2->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB));

    if (SERCOM2->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK)
    {
      // Acknowledge, issuing a STOP.
      SERCOM2->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
      return -1;
    }
  }

  // Acknowledge, issuing a STOP.
  SERCOM2->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
  return 0;
}
