#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__)
#else
  //#define HAL_ADC_MODULE_DISABLED
  //#define HAL_I2C_MODULE_DISABLED
  //#define HAL_RTC_MODULE_DISABLED
  //#define HAL_SPI_MODULE_DISABLED
  #define HAL_DAC_MODULE_DISABLED
  #define HAL_CAN_MODULE_DISABLED  
  //#define HAL_EXTI_MODULE_DISABLED
  //#define HAL_TIM_MODULE_DISABLED
#endif