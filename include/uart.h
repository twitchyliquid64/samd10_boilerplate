void setup_uart();

void uart_puts(const char *s);

#define UART_RX_ISR_DISABLE() \
  SERCOM0->USART.INTENCLR.reg = (SERCOM_USART_INTENCLR_RXC);

#define UART_RX_ISR_ENABLE() \
  SERCOM0->USART.INTENSET.reg = (SERCOM_USART_INTENSET_RXC);
