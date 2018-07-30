#include "samd10.h"


void setup_uart(void) {
  // enable power to the SERCOM peripheral.
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
  // use generic clock generator 4 + enable it + connect it to the SERCOM module.
  // CLKGEN 4 should have already been connected to OSC8M with a divider of 1.
  GCLK->CLKCTRL.reg = ((GCLK_CLKCTRL_GEN_GCLK4) | (GCLK_CLKCTRL_CLKEN) | (GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE)));

  // Set PA05 to use an alternative function.
  PORT->Group[0].PINCFG[5].reg = ((PORT_PINCFG_PMUXEN));
  // Setup PA05 to use peripheral function C (SERCOM0 PAD3). (2*2 + 1, AKA odd).
  PORT->Group[0].PMUX[2].reg = ((PORT_PMUX_PMUXO_C));
  // Set PA04 to use an alternative function.
  PORT->Group[0].PINCFG[4].reg = ((PORT_PINCFG_PMUXEN));
  // Setup PA04 to use peripheral function C (SERCOM0 PAD1). (2*2, AKA even).
  PORT->Group[0].PMUX[2].reg = ((PORT_PMUX_PMUXE_C));

  // Setup the UART with:
  // - LSB first (little endian - default)
  // - Asynchronous comms (CMODE=0)
  // - 16x oversampling on 7-8-9 (SAMPA + SAMPR), arithmetic baud-rate generation.
  // - Internal clocking
  // - RX on PAD3 (PA05), TX on PAD2 (PA04)
  SERCOM0->USART.CTRLA.reg =
	   SERCOM_USART_CTRLA_DORD |
	   SERCOM_USART_CTRLA_MODE_USART_INT_CLK | SERCOM_USART_CTRLA_SAMPA(0) |
     SERCOM_USART_CTRLA_SAMPR(0) |
	   SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1);

  // TX enable, character size set to 8 bits.
  SERCOM0->USART.CTRLB.reg = ((SERCOM_USART_CTRLB_TXEN) | (SERCOM_USART_CTRLB_CHSIZE(0))); // SERCOM_USART_CTRLB_RXEN

  // Aynchronous mode + arithmetic baud-rate generation selected in CTRLA.
  // Datasheet says Register = 65,536 * ( 1 - S(Fbaud/Fref))
  //                Register = 65,536 * ( 1 - S(9600/8,000,000))
  //                Register = 64277.7088
  SERCOM0->USART.BAUD.reg = 64277;

  // Enable peripheral.
  SERCOM0->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}

void uart_putc(char c)
{
	while (!(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE)); // While not Data Register Empty
	SERCOM0->USART.DATA.reg = c;
}

void uart_puts(const char *s)
{
	while (*s)
	uart_putc(*s++);
}
