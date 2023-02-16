// Remote node for RFM69, with voltage divider(A6), charge state (D8)
// PCB v.1.0
//
// ATMEL ATMEGA328 / ARDUINO
// -------------------------
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9)  PWM
//                  +----+
//
//  STM32F103C
//  ----------
//  RFM69HCW attached to SPI_1 
//  CS    <-->  PA4 <-->  BOARD_SPI1_NSS_PIN
//  SCK   <-->  PA5 <-->  BOARD_SPI1_SCK_PIN
//  MISO  <-->  PA6 <-->  BOARD_SPI1_MISO_PIN
//  MOSI  <-->  PA7 <-->  BOARD_SPI1_MOSI_PIN
//  DIO0  <-->  PA3 <-->  Interrupt on receiving a packet


// Do we have temperature/humidity sensor HTU2XD_SHT2X_SI70XX
// Comment out if not.
#define HTU2XD_SHT2X_SI70XX

// What is the Hardware Serial port to use for debug messages.
// Comment out if none.
#define SERIAL_PORT Serial1

// Includes
#include <SPI.h>
#include <RFM69.h>
#include <RFM69_ATC.h>
#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__)
  #define ATMEGA
  #include "LowPower.h"
  #include <avr/eeprom.h> // Global configuration for in chip EEPROM
#else
  #include "STM32LowPower.h"
  #ifdef SERIAL_PORT
    #define FLASH_DEBUG_OUTPUT SERIAL_PORT
  #endif
  #include "FlashStorage_STM32F1.h"
#endif

#ifdef HTU2XD_SHT2X_SI70XX
  #include <Wire.h>
  #include <HTU2xD_SHT2x_Si70xx.h>
  #define ELEMENTS 5
  HTU2xD_SHT2x_SI70xx ht2x(HTU2xD_SENSOR, HUMD_12BIT_TEMP_14BIT); //sensor type, resolution
#else
  #define ELEMENTS 3
#endif

// This node settings
#define VERSION         101    // Version of EEPROM struct
#define SENSOR_DELAY    6000 // In milliseconds, 10 minutes

// Constants
#define REG_LEN         21   // Size of one conf. element
#define NODE_NAME_SIZE  16   // As defined on gateway

// Pins
#ifdef ATMEGA
  #define CHARGE_STS    8
  #define BATT_VOLTAGE  A6
#else
  #define CHARGE_STS    PA8
  #define BATT_VOLTAGE  PB1
#endif

// Radio
#define NODEID          11   // This is our address 
#define NETWORKID       100  // Do not change, defined on gateway
#define GATEWAYID       1    // Do not change, gateway address
#define RADIO_REPEAT    5    // Repeat sending
#define FREQUENCY       RF69_868MHZ // Match this with the version of your gateway (others: RF69_433MHZ, RF69_915MHZ)
#define KEY             "ABCDABCDABCDABCD" // Has to be same 16 characters/bytes on all nodes, not more not less!
#define ENABLE_ATC      // Comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI        -75
// Radio ATC
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

// Initialize 
#ifdef ATMEGA
  // Notning on ATmega
#else // STM32
  // RTC
  STM32RTC& rtc = STM32RTC::getInstance();
  // HW SERIAL
  #ifdef SERIAL_PORT
    // USB or standard Serial1
    #if (SERIAL_PORT==Serial1)
      //                         RX    TX
      HardwareSerial SERIAL_PORT(PA10, PA9);
    #endif
  #endif
#endif

// Global variables
uint16_t sleepCount = 0;
uint8_t pingCount = 0;
uint8_t msg[31]; // size of REG_LEN, or larger for sensor msg if longer than REG_LEN size

// Configuration struct
struct config_t {
  uint16_t version;
  uint8_t  reg[REG_LEN * ELEMENTS]; // REG_LEN * #, number of elements on this node
} conf; 

// Float conversion 
union u_tag {
  byte  b[4]; 
  float fval;
} u;

/* 
 * Registration 
 */
void sendConf(){
  int8_t result;
  uint8_t count = 0;

  // Wait some time to avoid contention
  delay(NODEID * 1000);

  #ifdef SERIAL_PORT
    SERIAL_PORT.print(F("Conf:"));
  #endif

  while (count < sizeof(conf.reg)) {
    msg[0] = 'R'; // Registration flag
    memcpy(&msg[1], &conf.reg[count], REG_LEN);    
    result = radio.sendWithRetry(GATEWAYID, msg, REG_LEN + 1, RADIO_REPEAT);
    #ifdef SERIAL_PORT
      SERIAL_PORT.print(F(" ")); SERIAL_PORT.print(result);
    #endif
    count += REG_LEN;
  }  
  
  #ifdef SERIAL_PORT
    SERIAL_PORT.println(F("."));
  #endif
}
/* 
 * Set defaults on first time 
 */
void setDefault(){
  conf.version = VERSION;   // Change VERSION to force EEPROM re-load
  conf.reg[0+(REG_LEN*0)] = 'S';       // Sensor
  conf.reg[1+(REG_LEN*0)] = 'V';       // Voltage
  conf.reg[2+(REG_LEN*0)] = 0;         // Local address
  conf.reg[3+(REG_LEN*0)] = B00000000; // Default setting
  conf.reg[4+(REG_LEN*0)] = B00011111; // Default setting, group=16, disabled
  memset(&conf.reg[5+(REG_LEN*0)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*1)] = 'S';       // Sensor
  conf.reg[1+(REG_LEN*1)] = 'X';       // TX power level
  conf.reg[2+(REG_LEN*1)] = 0;         // Local address
  conf.reg[3+(REG_LEN*1)] = B00000000; // Default setting
  conf.reg[4+(REG_LEN*1)] = B00011111; // Default setting, group=16, disabled
  memset(&conf.reg[5+(REG_LEN*1)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*2)] = 'S';       // Sensor
  conf.reg[1+(REG_LEN*2)] = 'D';       // Digital pin, 1 = charging
  conf.reg[2+(REG_LEN*2)] = 0;         // Local address
  conf.reg[3+(REG_LEN*2)] = B00000000; // Default setting
  conf.reg[4+(REG_LEN*2)] = B00011111; // Default setting, group=16, disabled
  memset(&conf.reg[5+(REG_LEN*2)], 0, NODE_NAME_SIZE);
  #ifdef HTU2XD_SHT2X_SI70XX
    conf.reg[0+(REG_LEN*3)] = 'S';       // Sensor
    conf.reg[1+(REG_LEN*3)] = 'T';       // Temperature
    conf.reg[2+(REG_LEN*3)] = 0;         // Local address
    conf.reg[3+(REG_LEN*3)] = B00000000; // Default setting
    conf.reg[4+(REG_LEN*3)] = B00011111; // Default setting, group=16, disabled
    memset(&conf.reg[5+(REG_LEN*3)], 0, NODE_NAME_SIZE);
    conf.reg[0+(REG_LEN*4)] = 'S';       // Sensor
    conf.reg[1+(REG_LEN*4)] = 'H';       // Humidity
    conf.reg[2+(REG_LEN*4)] = 0;         // Local address
    conf.reg[3+(REG_LEN*4)] = B00000000; // Default setting
    conf.reg[4+(REG_LEN*4)] = B00011111; // Default setting, group=16, disabled
    memset(&conf.reg[5+(REG_LEN*4)], 0, NODE_NAME_SIZE);
  #endif
}
/*
 * Send ping command to gateway 
 */
void sendPing(void){
  msg[0] = 'C'; // Command
  msg[1] = 2;   // PING = 2
  // Send to GW 
  radio.sendWithRetry(GATEWAYID, msg, 2);
}
/*
 * Process incoming radio data
 */
void checkRadio(){
  uint8_t pos;

  // Look for incomming transmissions
  if (radio.receiveDone()) {
    
    // ACK handeling
    if (radio.ACKRequested()) { 
      delay(5); // wait after receive, we need this delay or gateway will not see ACK!!!
      radio.sendACK();
      #ifdef SERIAL_PORT
        SERIAL_PORT.print("ACK;");
      #endif
    }

    // Commands
    if ((char)radio.DATA[0] == 'C') {
      #ifdef SERIAL_PORT
        SERIAL_PORT.print("C:"); SERIAL_PORT.println(radio.DATA[1]);
      #endif
      // Commands from gateway
      switch (radio.DATA[1]) {
        case 1: // Request for registration
          sendConf(); 
          break;
        default: break;
      }
    }

    // Registration
    if ((char)radio.DATA[0] == 'R') { 
      #ifdef SERIAL_PORT
        SERIAL_PORT.print("R:");
      #endif
      // Replace part of conf string with new paramters.
      pos = 0; 
      while (((conf.reg[pos] != radio.DATA[1]) || (conf.reg[pos+1] != radio.DATA[2]) ||
              (conf.reg[pos+2] != radio.DATA[3])) && (pos < sizeof(conf.reg))) {
        pos += REG_LEN; // size of one conf. element
      }
      if (pos < sizeof(conf.reg)) {
        #ifdef SERIAL_PORT
          SERIAL_PORT.println(pos/REG_LEN); // Show # of updated element       
        #endif
        // Replace data
        memcpy(&conf.reg[pos], (uint8_t*)&radio.DATA[1], REG_LEN);
        // Save it to EEPROM
        conf.version = VERSION;
        // Update EEPROM        
        #ifdef ATMEGA
          eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration              
        #else
          EEPROM.put(0, conf);
        #endif        
      }
    }
  }
}
/*
 * Send float value of one element to gateway 
 */
void sendValue(uint8_t element, float value){
  u.fval = value; 
  msg[0] = conf.reg[(REG_LEN*element)];
  msg[1] = conf.reg[1+(REG_LEN*element)];
  msg[2] = conf.reg[2+(REG_LEN*element)];
  memcpy(&msg[3], &u.b[0], 4);
  // Send to GW 
  radio.sendWithRetry(GATEWAYID, msg, 7);
}
/* 
 * Add float value to message 
 */
void addFloatVal(uint8_t element, uint8_t *out, float value){
  out[0] = conf.reg[1+(REG_LEN*element)];
  out[1] = conf.reg[2+(REG_LEN*element)];
  u.fval = value; 
  memcpy(&out[2], &u.b[0], 4);
}
/*  
 * Send message to gateway
 */
void sendMessage() {
  // Build message
  msg[0] = 'S'; // Sensor
  // BATT Voltage, Voltage divider 2:1 
  addFloatVal(0, &msg[1], 0.0064453125*(float)analogRead(BATT_VOLTAGE));
  // TX power level, 0~100%; 0-31 level; HighPower has only 0-16
  addFloatVal(1, &msg[7], ((float)(radio.getPowerLevel()+1))*(radio.getHighPower() ? 6.25 : 3.125));
  // Charging, 0 = charging, 1 = not  
  addFloatVal(2, &msg[13], (float)!digitalRead(CHARGE_STS));
  // Optional sensors
  #ifdef HTU2XD_SHT2X_SI70XX
    // HTU21D Temperature 
    addFloatVal(3, &msg[19], ht2x.readTemperature());  
    // HTU21D Humidity       
    addFloatVal(4, &msg[25], ht2x.readHumidity());  
    // Send to GW 
    radio.sendWithRetry(GATEWAYID, msg, 31);
  #else
    // Send to GW 
    radio.sendWithRetry(GATEWAYID, msg, 19);
  #endif
}
/*
 * Setup
 */
void setup() {
  #ifdef SERIAL_PORT
    SERIAL_PORT.begin(115200); 
    while (!SERIAL_PORT);
    SERIAL_PORT.print("Start ");
  #endif

  #ifdef ATMEGA
    #ifdef SERIAL_PORT
      SERIAL_PORT.println(F("ATmega"));
    #endif
    // Set pins
    pinMode(CHARGE_STS, INPUT);     // set pin to input
    digitalWrite(CHARGE_STS, HIGH); // turn on pullup resistors
  #else
    #ifdef SERIAL_PORT
      SERIAL_PORT.println(F("STM32"));
    #endif
    // set the slave select (CS) pin (PA4 is default for SPI_1)
    radio.setSlaveSelectPin(PA4);
    // get the IRQ pin (which is connected to RFM69's DIO0)
    radio.setInterruptPin(PA3);
    // Set the IRQ pin number (at the STM32 must be same as interrupt pin)
    radio.setInterruptNumber(radio.getInterruptPin());
    // Set RTC to external quartz
    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin();
    // LowPower sleep library
    LowPower.begin();
    // Set pins
    pinMode(PB1, INPUT_ANALOG);     // Battery analog in 
    pinMode(CHARGE_STS, INPUT);     // Set pin to input
    digitalWrite(CHARGE_STS, HIGH); // Turn on pullup resistors
    // Disable peripherlas and pins to lower conumption
    STM32DisablePeripherals();
  #endif

  #ifdef HTU2XD_SHT2X_SI70XX
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    // Check connected     
    while (ht2x.begin() != true) //reset sensor, set heater off, set resolution, check power (sensor doesn't operate correctly if VDD < +2.25v)
    {
      #ifdef SERIAL_PORT
        SERIAL_PORT.println(F("HTU2xD/SHT2x failed"));
      #endif
      delay(5000);
    }
    #ifdef SERIAL_PORT
      SERIAL_PORT.println(F("HTU2xD/SHT2x OK"));
    #endif
  #endif

  // RFM69
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();  // uncomment only for RFM69HW!
  // radio.encrypt(KEY); // uncomment if you use encryption
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif
 
  // Node configuration 
  #ifdef ATMEGA
    eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
  #else
    #ifdef SERIAL_PORT
      SERIAL_PORT.print(F("EEPROM length: ")); SERIAL_PORT.println(EEPROM.length());
    #endif
    EEPROM.init();
    EEPROM.get(0, conf);          
  #endif    
  if (conf.version != VERSION) {
    setDefault();
    // Update EEPROM        
    #ifdef ATMEGA
      eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration              
    #else
      EEPROM.put(0, conf);
    #endif 
    #ifdef SERIAL_PORT
      SERIAL_PORT.println(F("Defaults set!"));
    #endif
  }
  // Send it to master
  sendConf();

  // Send our sensors values
  delay(200);
  sendMessage();
  
  // Let's start, go to sleep :)
  radio.sleep();
}
/*
 * Main loop
 */
void loop() {

  #ifdef ATMEGA
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // Sleep MCU for ~ 8 seconds
    sleepCount++;
  #else
    STM32BeforeSleep();
    LowPower.deepSleep(SENSOR_DELAY);
    STM32AfterSleep();    
  #endif  
  
  #ifdef ATMEGA  
    // SLEEP_8S = 8000ms
    if (sleepCount) >= (SENSOR_DELAY/8000) {
      sleepCount = 0;
  #endif         
  
  // Send our sensors values
  sendMessage();
  // Some debug
  #ifdef SERIAL_PORT
    SERIAL_PORT.println(radio.getPowerLevel());
  #endif
  // Check for queued messages after sleep
  for (uint8_t i=0; i < 8; i++){      
    delay(10); 
    checkRadio();
  }
  // Every hour or so, send alive ping
  pingCount++;        
  if (pingCount >= 6) {
    sendPing();
    pingCount = 0;
  }
  // Put radio to sleep
  radio.sleep();

  #ifdef ATMEGA
    }
  #endif    
}
/*
 * STM32 after sleep actions
 */
void STM32AfterSleep(){
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();    
  __HAL_RCC_GPIOB_CLK_ENABLE();
  // Peripherals
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
}
/*
 * STM32 before sleep actions
 */
void STM32BeforeSleep(){
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();    
  __HAL_RCC_GPIOB_CLK_DISABLE();
  // Peripherals
  __HAL_RCC_SPI1_CLK_DISABLE();    
  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_USART1_CLK_DISABLE();
}
/*
 * STM32 disable unused peripherals pins
 */
 void STM32DisablePeripherals() {
    // Diable not used peripherals
    __HAL_RCC_SPI2_CLK_DISABLE();
    #if (SERIAL_PORT!=Serial1)
      __HAL_RCC_USART1_CLK_DISABLE();
    #endif
    __HAL_RCC_USART2_CLK_DISABLE();
    __HAL_RCC_USART3_CLK_DISABLE();
    __HAL_RCC_CAN1_CLK_DISABLE();
    __HAL_RCC_DMA1_CLK_DISABLE();
    __HAL_RCC_TIM2_CLK_DISABLE();
    __HAL_RCC_TIM3_CLK_DISABLE();
    __HAL_RCC_TIM4_CLK_DISABLE();
    // Disable unused pins
    pinMode(PA0, INPUT_ANALOG);
    pinMode(PA1, INPUT_ANALOG);
    pinMode(PA2, INPUT_ANALOG);
    // IRQ pinMode(PA3, INPUT_ANALOG);
    // SPI pinMode(PA4, INPUT_ANALOG);
    // SPI pinMode(PA5, INPUT_ANALOG);
    // SPI pinMode(PA6, INPUT_ANALOG);
    // SPI pinMode(PA7, INPUT_ANALOG);
    
    // pinMode(PA8, INPUT_ANALOG); // CHARGE_STS
    #if (SERIAL_PORT!=Serial1)
      pinMode(PA9, INPUT_ANALOG);  // UART
      pinMode(PA10, INPUT_ANALOG); // UART
    #endif    
    pinMode(PA11, INPUT_ANALOG); // USB
    pinMode(PA12, INPUT_ANALOG); // USB
    pinMode(PA13, INPUT_ANALOG);
    pinMode(PA14, INPUT_ANALOG);
    pinMode(PA15, INPUT_ANALOG);

    pinMode(PB0, INPUT_ANALOG);
    // BATT pinMode(PB1, INPUT_ANALOG); // Already INPUT_ANALOG
    pinMode(PB2, INPUT_ANALOG);
    pinMode(PB3, INPUT_ANALOG);
    pinMode(PB4, INPUT_ANALOG);
    pinMode(PB5, INPUT_ANALOG);
    pinMode(PB6, INPUT_ANALOG);
    pinMode(PB7, INPUT_ANALOG);
    #ifndef HTU2XD_SHT2X_SI70XX
      pinMode(PB8, INPUT_ANALOG); // I2C
      pinMode(PB9, INPUT_ANALOG); // I2C
    #endif
    pinMode(PB10, INPUT_ANALOG);
    pinMode(PB11, INPUT_ANALOG);
    pinMode(PB12, INPUT_ANALOG);
    pinMode(PB13, INPUT_ANALOG);
    pinMode(PB14, INPUT_ANALOG);
    pinMode(PB15, INPUT_ANALOG);

    pinMode(PC13, INPUT_ANALOG);
 }