/********************************************************************************************************
 * title: soccer_table_goal_control
 * developers:  Hanneken, Patrick
 *              Huptas, Jill Catherine
 *              Lehmann, Kjell-Arne
 *              Linssen, Janina
 *              Martinius, Moritz
 *              Niehr, Jan Henrik
 *              Schneider, Fabian

********************************************************************************************************/
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

//********************************************************************************************************
//program settings Slave A

#define my_address slave_b_address
#define request_zero 0x00
#define request_goal 0x01
#define request_temp 0x02
#define request_batt 0x03
#define request_reset 0x0F

#define response_goal 0x11
#define response_batt 0x12
#define response_temp 0x13


//********************************************************************************************************

volatile uint16_t btn_a_state = 0;
volatile uint16_t btn_a_hold = 0;

unsigned char Master_address = 0x01;  // address for master
unsigned char slave_a_address = 0xBE; // address for slave a 
unsigned char slave_b_address = 0xEF; // address for slave b
uint8_t last_crc;                     // result of crc8 calculation
//uint8_t AddressTemp, Data1Temp, Data2Temp, crc8Temp, Address, Data1, Data2, crc8;   
uint8_t package[4]={};                // array for four byte data package
uint8_t goal = 0;                     // declaration for number of goals
uint8_t batt;                         // initialization of battery variable
uint8_t temp;                         // initialization of temperature variable

uint8_t calc_crc8(uint8_t * package, uint8_t len){               // function for crc8 calculation; uses the ccitt generator polynom 0x07
  uint8_t i = 0, last_crc = 0;        
  for (i = 0; i<len; i++){
    last_crc = _crc8_ccitt_update(last_crc, package[i]);
  }
  return last_crc;
}

void PrintHex8(uint8_t *data, uint8_t length){                   // prints 8-bit data in hex with leading zeroes
       Serial.print("0x"); 
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {Serial.print("0");} 
         Serial.print(data[i],HEX); 
         Serial.print(" "); 
       }
        Serial.println();
}
void goal_worker(void){                                         // button request for goal counter
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
  TCCR2B |= ((1 << CS22) | (1 << CS21) );                       // prescaler 256 on Timer2
  TIMSK2 |= ((1 << TOIE0));                                     // overflow Interrupt enable on Timer 2
  GTCCR &= (~(1 << TSM));                                       // start all timers by disabling the synced halt
}

uint8_t check_batt(void){
  float voltage;                          // variable for battery voltage
  uint8_t volt_return;                    // variable to convert the battery voltage to one byte
  voltage = rf24.get_battery_voltage();   // request
  voltage += 0.5;                         // round up or down to the next integer
  volt_return = voltage;                  // cast
  return volt_return;                     // return 
}

uint8_t check_temp(void){
  float temperature;                      // variable for temperature
  uint8_t temp_return;                    // voriable to convert the temperature to one byte
  temperature = rf24.get_temperature();   // request
  temperature += 0.5;                     // round up or down to the next integer
  temp_return = temperature;              // cast
  return temp_return;                     // return
}

//********************************************************************************************************
//begin of setup
  
void setup() 
{
  pinMode(A1, INPUT);         // Pinbelegung für den Knopf, später durch die Lichtschranke ersetzen

  timer_setup();              // initialize of timer function
  
  Serial.begin(115200);       // setting serial interface (baudrate)
  if (!rf24.init())           // case of hardware connection fail
    Serial.println("init failed");
}

//********************************************************************************************************
//begin of forever loop

void loop()
{ 
  package[0] = Master_address;        // only communicates with masterboard
  batt = check_batt();
  temp = check_temp();

 PrintHex8(package, 4);               // call function to print HEX number with zeroes
 Serial.print("number of goals since last request: ");
 Serial.println(goal);
    
  uint8_t buf[4];                     // receive the message
  uint8_t len = sizeof(buf);          // length of message

  
  if (rf24.waitAvailableTimeout(500)){       // wait for message
    if (rf24.recv(buf, &len)){               // write received message into buffer
      Serial.print("got reply: ");
      PrintHex8(buf, sizeof(buf));           // write received message into serial monitor
    }
    else{
      Serial.println("recv failed");
    }
  }
  else{
    Serial.println("No reply, is rf24_server running?");
  }
  Serial.println("receive: ");
  PrintHex8(buf, 4);
  
  if(my_address == buf[0] && calc_crc8(buf, 4) == 0){       // start process data if message is for this board and crc8 is ok
    Serial.println("Sending to rf24_server");
      switch(buf[1]){                       //switch case depending on master command/request writen in buf[1]
        case request_goal:                  //Master request goals since last request
                package[1] = response_goal;
                package[2] = goal;
                package[3] = calc_crc8(package, 3);
                rf24.send(package, sizeof(package));    //send built package
                rf24.waitPacketSent();
                Serial.println("Slave_a sent goal to Master.");
                
                if (rf24.waitAvailableTimeout(500)){    //wait for goal reset command  
                    if (rf24.recv(buf, &len)){
                        Serial.print("got reply: ");
                        PrintHex8(buf, sizeof(buf));
                    }
                    else{
                        Serial.println("recv failed");
                    }
                    if(my_address == buf[0] && (request_reset == buf[1] || request_zero == buf[1]) && calc_crc8(buf, 4) == 0){ //set goal to zero 
                        goal = 0;
                        Serial.println("Reset goals");
                    }
                    else{
                      Serial.println("No goal reset");
                    }
                }
                else{
                    Serial.println("No reply, is rf24_server running?");
                }
                break;
                
        case request_batt:
                package[1] = response_batt;
                package[2] = batt;
                package[3] = calc_crc8(package, 3);
                rf24.send(package, sizeof(package));
                rf24.waitPacketSent();
                Serial.println("Slave_a sent battery voltage to Master.");
                break;
                
        case request_temp:
                package[1] = response_temp;
                package[2] = temp;
                package[3] = calc_crc8(package, 3);
                rf24.send(package, sizeof(package));
                rf24.waitPacketSent();
                Serial.println("Slave_a sent temperature to Master.");
                break;
        }
  }
  delay(10);
}

ISR(TIMER2_OVF_vect) { // Will be called with roughly 183,1 Hz
  goal_worker();
}
