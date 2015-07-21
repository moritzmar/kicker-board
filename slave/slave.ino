#include <RH_RF24.h>

/****************************************************************
Client-Programm - Slave
Sende- und Empfangsfunktion

30.06.2015
Vers. 1
*****************************************************************/

#include <SPI.h>
#include <util/crc16.h>

#define DEBUG 1

#define RFM24_SS 6
#define RFM24_MOSI 11
#define RFM24_MISO 12 
#define RFM24_IRQ 2
#define RFM24_SDN 5
RH_RF24 rf24 = RH_RF24(RFM24_SS, RFM24_IRQ, RFM24_SDN); //Initialize a new instance of the RadioHead RFM24W driver as rf24

unsigned char Master_address = 0x01;
unsigned char slave_a_address = 0xBE;  //Funktionen fuer diesen Slave
unsigned char slave_b_address = 0xEF;

uint8_t goal = 0;

uint8_t AddressTemp, Data1Temp, Data2Temp, crc8Temp, Address, Data1, Data2, crc8;
uint8_t Package[4]={};

uint8_t calc_crc8(char * dataIn, uint8_t len)// uses the ccitt generator polynom 0x07
{
  uint8_t i = 0, last_crc = 0;
  for (i = 0; i<len; i++){
    last_crc = _crc8_ccitt_update(last_crc, dataIn[i]);
  }
  return last_crc;
}

void timer_setup(void) {
	TCCR2B |= ((1 << CS22) | (1 << CS21) ); // Prescaler 256 on Timer2
	TIMSK2 |= ((1 << TOIE0)); // Overflow Interrupt enable on Timer 2
	GTCCR &= (~(1 << TSM)); // Start all timers by disabling the synced halt
}


int SendSuccess = 0;
//Funktion zur Uebergabe von true oder false an das Gesamtprogramm
bool SendSuccessFunc(int SendSuccess){
   if (SendSuccess = 1){
       return true;
       SendSuccess = 0;
   }
   else return false;
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

void setup() {
        cli(); // global Interrupt disable, prevents flicker while initialzing
        
        Serial.begin(115200);
        Serial.println("Kicker-Funkslave up and running!");
        Serial.println("(c) by Patrick H. Jill H. Kjell-Arne L. Janina L. Moritz M. Jan N. Fabian S.");
        
        
        pinMode(A1, INPUT);
        timer_setup();
        
        if (!rf24.init())  { //Initialisierung des Funkmoduls fehlgeschlagen? 
            Serial.println("Init failed");
        }
        #ifdef DEBUG 
        else {
          Serial.println("RF24 successfully initialized");
        }
        #endif 

        sei(); // global Interrupt enable
}


void loop() {
    
      // Send a message to rf24_server
      uint8_t data[8] = "Score:";
      if(goal > 0) {
        data[6] = goal+48;
        data[7] = 0;
        Serial.println("Sending to rf24_server");
        rf24.send(data, sizeof(data));
        
      }
      
      
      rf24.waitPacketSent();
      // Now wait for a reply
      uint8_t buf[RH_RF24_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf24.waitAvailableTimeout(500))
      { 
        // Should be a reply message for us now   
        if (rf24.recv(buf, &len))
        {
          Serial.print("got reply: ");
          Serial.println((char*)buf);
        }
        else
        {
          Serial.println("recv failed");
        }
      }
     
    }
ISR(TIMER2_OVF_vect) { // Will be called with roughly 183,1 Hz
	goal_worker();
}

