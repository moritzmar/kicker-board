// rf24_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF24 class. RH_RF24 class does not provide for addressing or
// reliability, so you should only use RH_RF24 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf24_server.
// Tested on Anarduino Mini http://www.anarduino.com/mini/ with RFM24W and RFM26W

#include <RH_RF24.h>
// Singleton instance of the radio driver
#include <SPI.h>
#include <TimerOne.h>
#include <util/crc16.h>


#define RFM24_SS 6
#define RFM24_MOSI 11
#define RFM24_MISO 12 
#define RFM24_IRQ 2
#define RFM24_SDN 5
RH_RF24 rf24 = RH_RF24(RFM24_SS, RFM24_IRQ, RFM24_SDN); //Initialize a new instance of the RadioHead RFM24W driver as rf24

volatile uint16_t btn_a_state = 0;
volatile uint16_t btn_a_hold = 0;

unsigned char Master_address = 0x01;
unsigned char slave_a_address = 0xBE;  
unsigned char slave_b_address = 0xEF; //Funktionen fuer diesen Slave
//uint8_t len;
uint8_t last_crc;
uint8_t AddressTemp, Data1Temp, Data2Temp, crc8Temp, Address, Data1, Data2, crc8;
uint8_t package[4]={};
uint8_t goal = 0;
//uint8_t buf[4] = {};

uint8_t calc_crc8(uint8_t * package, uint8_t len)// uses the ccitt generator polynom 0x07
{
  uint8_t i = 0, last_crc = 0;
  for (i = 0; i<len; i++){
    last_crc = _crc8_ccitt_update(last_crc, package[i]);
  }
  return last_crc;
}

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       Serial.print("0x"); 
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {Serial.print("0");} 
         Serial.print(data[i],HEX); 
         Serial.print(" "); 
       }
        Serial.println();
}
void goal_worker(void) {
    static uint8_t goal_state = 0;
        uint16_t goal_current = 0;

    goal_current = digitalRead(A1);
    if(goal_state == 0 && (goal_current == LOW)) {
      // taster gedrueckt, fallende flanke
      goal_state = 1;
                goal++;
      }
    else if((goal_state == 1) && (goal_current == LOW)) {
                // gehalten
      goal_state = 2;
    }
    else if(goal_state == 2 && (goal_current == HIGH)) {
      // taster wird los gelassen
      goal_state = 3;
    }
    else if(goal_state == 3 && (goal_current == HIGH)) {
      // taster nicht gedrueckt
      goal_state = 0;
    }
}

void timer_setup(void) {
  TCCR2B |= ((1 << CS22) | (1 << CS21) ); // Prescaler 256 on Timer2
  TIMSK2 |= ((1 << TOIE0)); // Overflow Interrupt enable on Timer 2
  GTCCR &= (~(1 << TSM)); // Start all timers by disabling the synced halt
}

void setup() 
{
  pinMode(A1, INPUT);         //Pinbelegung für de Knopf

  timer_setup();
  
  Serial.begin(115200);
  if (!rf24.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb5Fd10, power 0x10
//  if (!rf24.setFrequency(433.0))
//    Serial.println("setFrequency failed");
}
void loop()
{
  Serial.println("Sending to rf24_server");
  // Send a message to rf24_server
  package[0] = Master_address;
  package[1] = 0x01;
  package[2] = goal;
  package[3] = calc_crc8(package, 3);

 PrintHex8(package, 4);
 Serial.print("Anzahl Tore:");
 Serial.println(goal);
    
  uint8_t buf[4]; //RH_RF24_MAX_MESSAGE_LEN
  uint8_t len = sizeof(buf);

  
  if (rf24.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (rf24.recv(buf, &len))
    {
      Serial.print("got reply: ");
      PrintHex8(buf, sizeof(buf));
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is rf24_server running?");
  }
  Serial.println("Empfangen: ");
  PrintHex8(buf, 4);
  if(slave_b_address == buf[0] && 0x01 == buf[1] && 0x00 == buf[2] && calc_crc8(buf, 4) == 0){
      rf24.send(package, sizeof(package));
      rf24.waitPacketSent();
      Serial.println("Slave_a sent Package to Master.");
  }
  if(slave_b_address == buf[0] && (0x02 || 0x03) == buf[1] && calc_crc8(buf, 4) == 0){
    goal = 0;
  }
  delay(400);
}

ISR(TIMER2_OVF_vect) { // Will be called with roughly 183,1 Hz
  goal_worker();
}
