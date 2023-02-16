#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__)
#else
  -EXTI_IRQ_PRIO=1 -UART_IRQ_PRIO=3
#endif