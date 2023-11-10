/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI Corporation
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example transmits data on hardcoded channel and receives data
 * when not transmitting. Running this sketch on two nodes should allow
 * them to communicate.
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// we formerly would check this configuration; but now there is a flag,
// in the LMIC, LMIC.noRXIQinversion;
// if we set that during init, we get the same effect.  If
// DISABLE_INVERT_IQ_ON_RX is defined, it means that LMIC.noRXIQinversion is
// treated as always set.
//
// #if !defined(DISABLE_INVERT_IQ_ON_RX)
// #error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
//        lmic_project_config.h in arduino-lmic/project_config to set it.
// #endif

// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc

#define TX_INTERVAL 4000

uint8_t motor_data;
char sensor_data[10];
const int led_pin = 3;  // 8
const int moisture_sensor_pin = A5;

//bool RXflag = false;
int txrx_loop = 15; //15 is giving good output

unsigned long previousMillis = 0;
const unsigned long interval = 6000;

volatile uint8_t InterruptFlag= 1;

uint8_t Node_addr = 0xAA;
uint8_t chksm;
int min_to_sleep = 2;
bool ack_flag = false;

// union packet {
//   uint8_t sensor_data;
//       typedef struct 
//         {
//           uint8_t addr    :15;   //12 bit data
//           uint8_t data  :15;    //2 bit data          || |||||||||||| |||| || ||||||||||||                
//         }sensor;
// };

#pragma pack(1)
typedef struct 
  {
      uint8_t addr  ;   //12 bit data
      char data[8] ;      //2 bit data          || |||||||||||| |||| || ||||||||||||                
  }packet;

packet sensor;

typedef struct
{
  uint8_t node_addr ;
  uint8_t data;
  uint8_t ack ;
  uint8_t chksm ;

}dwnlnk;

dwnlnk sensor_d;

/*
// Calculate the XOR checksum for a vector of bytes
uint8_t calculateChecksum(char sensor_data[], int n) {
    uint8_t checksum = 0;
    Serial.print(n);
    for (int i = 0; i < n; i++)
    {
        checksum ^= byte;
    }
    return checksum;
}
*/

/*
uint8_t calculateChecksum2(packet s) {
    uint8_t checksum = 0;
    //Serial.print();
    for (int i = 0; i < sizeof(packet); i++)
    {
        checksum += ((unsigned char*)&s)[i];
    }

    return ~checksum;
}
*/

/*
void struc_to_array(dwnlnk data_d, uint8_t data[])
{
    for (int i = 0; i < sizeof(data_d); i++)
    {
       data[i] = ((unsigned char *)&data_d)[i];
    }
    Serial.print((char*)data));
}

void array_to_struc(dwnlnk data_d, uint8_t data[])
{
    for (int i = 0; i < sizeof(data_d); i++)
    {
      ((unsigned char *)&data_d)[i] = data[i];
    }
     Serial.print((unsigned char *)data_d.node_addr);
     Serial.print((char *)data_d.ack);
     Serial.print((char *)data_d.data);
     Serial.print((unsigned char *)data_d.chksm);
}
*/

uint8_t calc_checksum(void *ptr,int type) {
    uint8_t checksum = 0;
    int size = 0;
    //packet* p1; dwnlnk* p2;
    if(type == 0)
    {
        packet* p1 = (packet*)ptr;
        size = sizeof(packet);
        
        for (int i = 0; i < size; i++)
        {
          checksum += ((unsigned char*)p1)[i];
        }
    }
    else
    {
        dwnlnk* p2 = (dwnlnk*)ptr;
        size = sizeof(dwnlnk)-1;
        
        for (int i = 0; i < size; i++)
        {
          checksum += ((unsigned char*)p2)[i];
        }
    }

    return ~checksum;
}

void sleep_init()
{
  ADCSRA &= ~(1 << ADEN);  //disabling ADC

  //Power Reduction Register
  PRR = (1 << PRTWI)    |
        (1 << PRTIM2)   | 
        (1 << PRTIM1)   |
        (1 << PRADC); 
//        (1 << PRSPI)    |     // keep on LoRa
//        (1 << PRUSART0) |     // ''
         
}

void sleep_dis()
{
  //Power Reduction Register
  PRR &= ~(1 << PRTWI)    |
         ~(1 << PRTIM2)   | 
         ~(1 << PRTIM1)   |
         ~(1 << PRADC); 
//       ~(1 << PRSPI)    |     // keep on LoRa
//       ~(1 << PRUSART0) |     // ''
         
  // Enable ADC
  ADCSRA |= (1 << ADEN);
  
  // // Enable ADC auto-triggering
  // ADCSRA |= (1 << ADATE);
  
  // // Set ADC auto-trigger source to Free Running Mode
  // ADCSRB &= ~(1 << ADTS2) & ~(1 << ADTS1) & ~(1 << ADTS0);
  
  // // Set ADC reference voltage to AVCC
  // ADMUX |= (1 << REFS0);
  
  // // Set ADC input channel to A0
  // ADMUX |= 0x00;


// Reconfigure ADC settings
   //ADCSRA |= (1 << ADEN);  // Enable ADC
   delay(100);             // Wait for ADC stabilization
}

void reset_wdt()
{
    //disable watchdog timer
    asm("wdr");                       //reset WDT
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0x00;
}

// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect)
{
  // execute scheduled jobs and events
  InterruptFlag = 1;
  reset_wdt();
  //sleep_dis();
 // delay(10000);
}

// Arduino Sleep Mode : Power Down
void goToSleep(int n)
{
  for(int i = 0; i < n; i++)
  {  
  //disable interrupts while configuring sleep 
  asm("cli");

  uint8_t wdt_timeout = (1 << WDP3) | (1 << WDP0); //8s  //for 4s remove | (1 << WDP0) or 0x21
  asm("wdr");   //reset WDT
  WDTCSR |= (1 << WDCE) | (1 << WDE); //special operation to change WDT config
  WDTCSR = (1 << WDIE) | wdt_timeout;  //enable WDT interrupts MODE, 

  
  //sleep_init();

  //keep_ADCSRA = ADCSRA;
  //ADCSRA = 0;  //disabling ADC
  //Sleep sequence (call right before sleeping)
  SMCR |= (1 << SM1);  // Power down sleep mode
  SMCR |= (1 << SE);   // Enable Sleep

  //turn off Brown out enable
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS);

  Serial.println("Arduino : Going to Sleep!");

  //re-enable interrupts & call sleep instructions
  asm("sei");      //enable interrupts
  asm("sleep");    //go to sleep

  }

  /* disable sleeping as precaution */
  SMCR &= ~(1 << SE);   // Disable Sleep
  
  //sleep_dis();


  // Turn power on to ADC
  // PRR &= ~(1 << PRADC);
  
  // Enable ADC
  // ADCSRA = keep_ADCSRA;
  // ADCSRA |= (1 << ADEN);
  
  /* wake up here & handles ISR */
  //Serial.println("Arduino : Just Woke up!");
  //Ultrasonic_sensor_data();

  // Discard first ADC reading
  Serial.print(analogRead(moisture_sensor_pin));
}

// Arduino Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};


void soil_moisture_data()
{
    int sensorValue = analogRead(moisture_sensor_pin);  // Read the analog value from sensor
    int outputValue = map(sensorValue, 0, 1023, 255, 0); // map the 10-bit data to 8-bit data
    //analogWrite(led_pin, outputValue); // generate PWM signal
    //Serial.print(dtostrf(outputValue,6,2,sensor_data));
    //dtostrf(outputValue,6,2,sensor_data);
    //char s[10] = (char *)sensor.data;
    dtostrf(outputValue,6,4,sensor.data);
    delay(500);
    //Serial.print("Soil Moisture Data : ");
    //Serial.print(dtostrf(outputValue,6,2,sensor_data));    
    //return outputValue;             // Return analog moisture value
}


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmoc/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) {
}

osjob_t txjob;
osjob_t timeoutjob;
static void tx_func (osjob_t* job);

// Transmit the given string and call the given function afterwards
void tx(const char *str, osjobcb_t func) {
  
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
  LMIC.dataLen = 0;
  while (*str)
    //Serial.println("inside tx");
    LMIC.frame[LMIC.dataLen++] = *str++;
    Serial.println(*str);

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  Serial.println("TX");
}

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)
  os_radio(RADIO_RXON);
  Serial.println("RX");
}

static void rxtimeout_func(osjob_t *job) {
  digitalWrite(LED_BUILTIN, LOW); // off
}

static void rx_func (osjob_t* job) {
 // Blink once to confirm reception and then keep the led on
  digitalWrite(LED_BUILTIN, LOW); // off
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH); // on

  // Timeout RX (i.e. update led status) after 3 periods without RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);
  // digitalWrite(led_pin, LOW); Serial.println("No RX : Bulb off! ");
  // Reschedule TX so that it should not collide with the other side's
  // next TX
  os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);

  Serial.print("Got ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");
  Serial.write(LMIC.frame, LMIC.dataLen);

  // char* str1 = (char*) LMIC.frame - 97;
  // for (int i = LMIC.dataBeg; i < LMIC.dataBeg + LMIC.dataLen; i++) {
  //                   Serial.print(LMIC.frame[i], HEX);
  //                   Serial.print(" ");
  //               }
  // char* str1 =  LMIC.frame ;

  if(LMIC.dataLen)
  {
      Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      Serial.println();

      for (int i = 0; i < LMIC.dataLen; i++) 
        {
          Serial.print(LMIC.frame[LMIC.dataBeg + i],HEX);
           ((unsigned char* )&sensor_d)[i] = LMIC.frame[LMIC.dataBeg + i];
        } 
      Serial.println(sensor_d.node_addr);
      Serial.println(Node_addr);

    if(sensor_d.node_addr == Node_addr) 
    {    
      Serial.println((uint8_t)sensor_d.node_addr,HEX);
      Serial.println((uint8_t)sensor_d.data,HEX);
      Serial.println((uint8_t)sensor_d.ack,HEX);
      Serial.println((uint8_t)sensor_d.chksm,HEX);

      if((uint8_t)sensor_d.ack == '2' || (uint8_t)sensor_d.ack == '1')
      {
        ack_flag = true;
      }

      uint8_t chksm_d = calc_checksum(&sensor_d,1);
      Serial.print("Downlink Checksum : ");
      Serial.print(chksm_d);

      if(chksm_d == (uint8_t)sensor_d.chksm )
      {
        Serial.print("Checksum Matched! ");

/*        
        uint8_t addr = LMIC.frame[LMIC.dataBeg] ; //Serial.write(LMIC.frame, LMIC.dataLen); //atoi((const char *)LMIC.frame);
        //uint8_t result = LMIC.frame[LMIC.dataBeg + 0] ;
        // sensor_d.node_addr = addr;
        Serial.print(" addr : ");
        Serial.println((uint8_t)sensor_d.node_addr,HEX);
        //Serial.println((uint8_t)addr,HEX);
        Serial.println();

        motor_data = LMIC.frame[1]; //Serial.write(LMIC.frame, LMIC.dataLen); //atoi((const char *)LMIC.frame);
        //uint8_t result = LMIC.frame[LMIC.dataBeg + 0] ;
        // sensor_d.data = motor_data;
        Serial.print(" motor_data : ");
        //Serial.println((uint8_t)motor_data,HEX);
        Serial.println((uint8_t)sensor_d.data,HEX);
        Serial.println();
        
        uint8_t ack = LMIC.frame[2]; //Serial.write(LMIC.frame, LMIC.dataLen); //atoi((const char *)LMIC.frame);
        //uint8_t result = LMIC.frame[LMIC.dataBeg + 0] ;
        // sensor_d.ack = ack;
        Serial.print(" ack : ");
        Serial.println((uint8_t)sensor_d.ack,HEX);
        //Serial.println((uint8_t)ack,HEX);
        Serial.println();

        uint8_t chksm = LMIC.frame[3]; //Serial.write(LMIC.frame, LMIC.dataLen); //atoi((const char *)LMIC.frame);
        //uint8_t result = LMIC.frame[LMIC.dataBeg + 0] ;
        // sensor_d.chksm = chksm;
        Serial.print(" chksm : ");
        Serial.println((uint8_t)sensor_d.chksm,HEX);
        //Serial.println((uint8_t)chksm,HEX);
        Serial.println();
*/        
        if((uint8_t)sensor_d.ack == '2')
        {
          if(sensor_d.data == '0') { digitalWrite(led_pin, LOW); Serial.println("Bulb off! ");}
          else if(sensor_d.data == '1') { digitalWrite(led_pin, HIGH); Serial.println("Bulb on! ");}
        }
      }
        //RXflag = true;
    }
  }
  else { digitalWrite(led_pin, LOW); Serial.println("Bulb off! "); }

  // Restart RX
  rx(rx_func);
}

static void txdone_func (osjob_t* job) {
  rx(rx_func);
}

// log text to USART and toggle LED
static void tx_func (osjob_t* job) {
  // say hello
  //Serial.print("Soil Moisture Data : ");
  //Serial.print(sensor_data);
  
   //Serial.print(sizeof(sensor));
   
   for (int i = 0; i < sizeof(sensor); i++)
    {
        sensor_data[i] = ((unsigned char *)&sensor)[i];
        //sensor_data.push_back(((unsigned char *)&sensor)[i]);
        Serial.print(sensor_data[i]);
    }
  

/* 
   uint8_t checksum = calculateChecksum(sensor_data,sizeof(sensor));
   Serial.print("Checksum : ");
   Serial.print(checksum);
*/  
   //uint8_t checksum2 = calculateChecksum2(sensor);
   uint8_t checksum2 = calc_checksum(&sensor,0);
   //Serial.print("Checksum2 : ");
   //Serial.print(checksum2);
   //Serial.print("  ");

  //  Serial.print("size : ");
  // Serial.print(sizeof(sensor));

  //Serial.print("size : ");
  // Serial.print(sizeof(sensor_data));

  sensor_data[9] = (unsigned char* )checksum2;

  
  //Serial.print("    ");
    // Serial.print(sizeof(sensor_data));
  //Serial.print(sensor_data);
  //Serial.print("  size:");
    //Serial.print(strlen(sensor_data));
  //Serial.print(sizeof(sensor_data));

/* Checking Data 
  uint8_t ans [50];

    for (int i = 0; i < sizeof(sensor); i++)
    {
       ans[i] = ((unsigned char *)&sensor)[i];
       // Serial.print(ans[i]);
    }
     Serial.print(" ans   : ");
   //Serial.print(ans);
  for (int i = 0; i < sizeof(sensor); i++)
    {
        ((unsigned char *)&sensor)[i] = ans[i]; //sensor_data[i];
        //Serial.println();
       Serial.print(((char *)&sensor)[i]);
    }
*/

/*
    Serial.print("   ");
    
    Serial.print(sensor.addr);
    Serial.print("   ");
    Serial.print("   ");
    Serial.print(sensor.data);
*/

  //Serial.print(sensor.data);

  tx(sensor_data, txdone_func);
  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}

// application entry point
void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  //soil moisture sensor
  sensor.addr = Node_addr;

  /* Enablign Sleep Registers */
 // sleep_init();

  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif

/* soil moisture sensor */
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  pinMode(LED_BUILTIN, OUTPUT);

  // initialize runtime env
  os_init();



#if defined(CFG_eu868)
  // Use a frequency in the g3 which allows 10% duty cycling.
  LMIC.freq = 869525000;
  // Use a medium spread factor. This can be increased up to SF12 for
  // better range, but then, the interval should be (significantly)
  // raised to comply with duty cycle limits as well.
  LMIC.datarate = DR_SF9;
  // Maximum TX power
  LMIC.txpow = 27;
#elif defined(CFG_us915)
  // make it easier for test, by pull the parameters up to the top of the
  // block. Ideally, we'd use the serial port to drive this; or have
  // a voting protocol where one side is elected the controller and
  // guides the responder through all the channels, powers, ramps
  // the transmit power from min to max, and measures the RSSI and SNR.
  // Even more amazing would be a scheme where the controller could
  // handle multiple nodes; in that case we'd have a way to do
  // production test and qualification. However, using an RWC5020A
  // is a much better use of development time.

  // set fDownlink true to use a downlink channel; false
  // to use an uplink channel. Generally speaking, uplink
  // is more interesting, because you can prove that gateways
  // *should* be able to hear you.
  const static bool fDownlink = false;

  // the downlink channel to be used.
  const static uint8_t kDownlinkChannel = 3;

  // the uplink channel to be used.
  const static uint8_t kUplinkChannel = 8 + 3;

  // this is automatically set to the proper bandwidth in kHz,
  // based on the selected channel.
  uint32_t uBandwidth;

  if (! fDownlink)
        {
        if (kUplinkChannel < 64)
                {
                LMIC.freq = US915_125kHz_UPFBASE +
                            kUplinkChannel * US915_125kHz_UPFSTEP;
                uBandwidth = 125;
                }
        else
                {
                LMIC.freq = US915_500kHz_UPFBASE +
                            (kUplinkChannel - 64) * US915_500kHz_UPFSTEP;
                uBandwidth = 500;
                }
        }
  else
        {
        // downlink channel
        LMIC.freq = US915_500kHz_DNFBASE +
                    kDownlinkChannel * US915_500kHz_DNFSTEP;
        uBandwidth = 500;
        }

  // Use a suitable spreading factor
  if (uBandwidth < 500)
        LMIC.datarate = US915_DR_SF7;         // DR4
  else
        LMIC.datarate = US915_DR_SF12CR;      // DR8

  // default tx power for US: 21 dBm
  LMIC.txpow = 21;
#elif defined(CFG_au915)
  // make it easier for test, by pull the parameters up to the top of the
  // block. Ideally, we'd use the serial port to drive this; or have
  // a voting protocol where one side is elected the controller and
  // guides the responder through all the channels, powers, ramps
  // the transmit power from min to max, and measures the RSSI and SNR.
  // Even more amazing would be a scheme where the controller could
  // handle multiple nodes; in that case we'd have a way to do
  // production test and qualification. However, using an RWC5020A
  // is a much better use of development time.

  // set fDownlink true to use a downlink channel; false
  // to use an uplink channel. Generally speaking, uplink
  // is more interesting, because you can prove that gateways
  // *should* be able to hear you.
  const static bool fDownlink = false;

  // the downlink channel to be used.
  const static uint8_t kDownlinkChannel = 3;

  // the uplink channel to be used.
  const static uint8_t kUplinkChannel = 8 + 3;

  // this is automatically set to the proper bandwidth in kHz,
  // based on the selected channel.
  uint32_t uBandwidth;

  if (! fDownlink)
        {
        if (kUplinkChannel < 64)
                {
                LMIC.freq = AU915_125kHz_UPFBASE +
                            kUplinkChannel * AU915_125kHz_UPFSTEP;
                uBandwidth = 125;
                }
        else
                {
                LMIC.freq = AU915_500kHz_UPFBASE +
                            (kUplinkChannel - 64) * AU915_500kHz_UPFSTEP;
                uBandwidth = 500;
                }
        }
  else
        {
        // downlink channel
        LMIC.freq = AU915_500kHz_DNFBASE +
                    kDownlinkChannel * AU915_500kHz_DNFSTEP;
        uBandwidth = 500;
        }

  // Use a suitable spreading factor
  if (uBandwidth < 500)
        LMIC.datarate = AU915_DR_SF7;         // DR4
  else
        LMIC.datarate = AU915_DR_SF12CR;      // DR8

  // default tx power for AU: 30 dBm
  LMIC.txpow = 30;
#elif defined(CFG_as923)
// make it easier for test, by pull the parameters up to the top of the
// block. Ideally, we'd use the serial port to drive this; or have
// a voting protocol where one side is elected the controller and
// guides the responder through all the channels, powers, ramps
// the transmit power from min to max, and measures the RSSI and SNR.
// Even more amazing would be a scheme where the controller could
// handle multiple nodes; in that case we'd have a way to do
// production test and qualification. However, using an RWC5020A
// is a much better use of development time.
        const static uint8_t kChannel = 0;
        uint32_t uBandwidth;

        LMIC.freq = AS923_F1 + kChannel * 200000;
        uBandwidth = 125;

        // Use a suitable spreading factor
        if (uBandwidth == 125)
                LMIC.datarate = AS923_DR_SF7;         // DR7
        else
                LMIC.datarate = AS923_DR_SF7B;        // DR8

        // default tx power for AS: 21 dBm
        LMIC.txpow = 16;

        if (LMIC_COUNTRY_CODE == LMIC_COUNTRY_CODE_JP)
                {
                LMIC.lbt_ticks = us2osticks(AS923JP_LBT_US);
                LMIC.lbt_dbmax = AS923JP_LBT_DB_MAX;
                }
#elif defined(CFG_kr920)
// make it easier for test, by pull the parameters up to the top of the
// block. Ideally, we'd use the serial port to drive this; or have
// a voting protocol where one side is elected the controller and
// guides the responder through all the channels, powers, ramps
// the transmit power from min to max, and measures the RSSI and SNR.
// Even more amazing would be a scheme where the controller could
// handle multiple nodes; in that case we'd have a way to do
// production test and qualification. However, using an RWC5020A
// is a much better use of development time.
        const static uint8_t kChannel = 0;
        uint32_t uBandwidth;

        LMIC.freq = KR920_F1 + kChannel * 200000;
        uBandwidth = 125;

        LMIC.datarate = KR920_DR_SF7;         // DR7
        // default tx power for KR: 14 dBm
        LMIC.txpow = KR920_TX_EIRP_MAX_DBM;
        if (LMIC.freq < KR920_F14DBM)
          LMIC.txpow = KR920_TX_EIRP_MAX_DBM_LOW;

        LMIC.lbt_ticks = us2osticks(KR920_LBT_US);
        LMIC.lbt_dbmax = KR920_LBT_DB_MAX;
#elif defined(CFG_in866)
// make it easier for test, by pull the parameters up to the top of the
// block. Ideally, we'd use the serial port to drive this; or have
// a voting protocol where one side is elected the controller and
// guides the responder through all the channels, powers, ramps
// the transmit power from min to max, and measures the RSSI and SNR.
// Even more amazing would be a scheme where the controller could
// handle multiple nodes; in that case we'd have a way to do
// production test and qualification. However, using an RWC5020A
// is a much better use of development time.
        const static uint8_t kChannel = 0;
        uint32_t uBandwidth;

        LMIC.freq = IN866_F1 + kChannel * 200000;
        Serial.println(LMIC.freq/100000);
        uBandwidth = 125;

        LMIC.datarate = IN866_DR_SF7;         // DR7
        // default tx power for IN: 30 dBm
        LMIC.txpow = IN866_TX_EIRP_MAX_DBM;
#else
# error Unsupported LMIC regional configuration.
#endif


  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  // This sets CR 4/5, BW125 (except for EU/AS923 DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  Serial.print("Frequency: "); Serial.print(LMIC.freq / 1000000);
            Serial.print("."); Serial.print((LMIC.freq / 100000) % 10);
            Serial.print("MHz");
  Serial.print("  LMIC.datarate: "); Serial.print(LMIC.datarate);
  Serial.print("  LMIC.txpow: "); Serial.println(LMIC.txpow);

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  Serial.println("Started");
  Serial.flush();

  // setup initial job
  os_setCallback(&txjob, tx_func);
}

void loop() {

  unsigned long currentMillis = millis(); // Get the current time
  if( InterruptFlag == 1)
  {
    Serial.println("Arduino : Just Woke up!"); 
    soil_moisture_data();

    //while(RXflag == false) 
  for(int i = 0; i < txrx_loop ; i++)
   {
     reset_wdt();
     //Serial.print("i : ");
     //Serial.print(i);
     //delay(3500);
     soil_moisture_data();
     os_runloop_once();
    
    // Serial.print(currentMillis);
    //  if (millis() <  (currentMillis + interval)) 
    //  { 
    //  // Code to be executed after the interval
    //  Serial.println("6 second has passed");
    //  }
    // delay(1000);

     /*
     if((i == txrx_loop - 1) && (RXflag == false))
     { 
       reset_wdt();
       Serial.println("No Downlinks! ");
       txrx_loop = txrx_loop * (3/2); 
       Serial.println(txrx_loop);
     }
     */
   }
    delay(1000);
    InterruptFlag = 0;
    //RXflag = false;
  }

  if(ack_flag != true)
  { 
    min_to_sleep = (min_to_sleep/2); 
    Serial.println("1 Minute Sleep");
    //Serial.print(min_to_sleep);
  }
  else 
  { 
    Serial.println("2 Minute Sleep"); 
    ack_flag = false;
  }

  /* n is factor of 8 ex: for 2 minutes n, will be 15 , 5min[37]*/
  goToSleep(8 * min_to_sleep);
 
  /* disable sleeping as precaution */
 // sleep_dis();
 // SMCR &= ~(1 << SE);   // Disable Sleep

}
