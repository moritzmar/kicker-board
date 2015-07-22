#include <SPI.h>
#include <RH_RF24.h>
#include <util/crc16.h>

#define DEBUG 1

#define RFM24_SS 6
#define RFM24_MOSI 11
#define RFM24_MISO 12 
#define RFM24_IRQ 2
#define RFM24_SDN 5


#define DATA_EMPTY 0x00

#define MASTER_SEND_GOAL_RST 0x00
#define MASTER_REQUEST_GOAL 0x01
#define MASTER_REQUEST_BAT_VOLTAGE 0x02
#define MASTER_REQUEST_TEMPERATURE 0x03
#define MASTER_RST 0x0F

#define SLAVE_GOALS_DETECTED 0x11
#define SLAVE_SEND_BAT_VOLTAGE 0x12
#define SLAVE_SEND_TEMPERATURE 0x13
#define SLAVE_OFFSET 0x10



typedef struct node {
  uint8_t address;
  uint8_t order_no;
} node;

node master = {0x01, 0};
node slave_a = {0xBE, 0};
node slave_b = {0xEF, 1};


static uint8_t score_a = 0;
static uint8_t score_b = 0;

volatile uint16_t btn_a_state = 0;
volatile uint16_t btn_b_state = 0;

volatile uint16_t btn_a_hold = 0;
volatile uint16_t btn_b_hold = 0;

const uint8_t segments_lut_1 [] =  {0x3E, 0x30, 0x2D, 0x39, 0x33, 0x1B, 0x1F, 0x30, 0x3F, 0x3B, 0x0F};
const uint8_t segments_lut_2 [] =  {0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80};


RH_RF24 rf24 = RH_RF24(RFM24_SS, RFM24_IRQ, RFM24_SDN); //Initialize a new instance of the RadioHead RFM24W driver as rf24

void timer_setup(void) {
	TCCR2B |= ((1 << CS22) | (1 << CS21) ); // Prescaler 256 on Timer2
	TIMSK2 |= ((1 << TOIE0)); // Overflow Interrupt enable on Timer 2
	GTCCR &= (~(1 << TSM)); // Start all timers by disabling the synced halt
}


void segments_setup(void) {
  	DDRC |= ( (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5)); // Output pins for 7 segment G-B
	DDRD |= (1 << PD7); // Output pin for 7 segment A

	DDRB |= (1 << PB0) | (1 << PB1); // 7 segment drive transistor
}


void segments_worker(uint8_t score_a, uint8_t score_b) {  // Segments multiplexing
	static uint8_t element = 0;
	if(element == 0) {
		PORTB &= ~(1 << PB1); // disable 2nd display element
		element = 1; // set flag for next element
		PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5));
		PORTD &= ~(1 << PD7);
		PORTC |= segments_lut_1[score_a]; // set segment G-B from LUT
		PORTD |= segments_lut_2[score_a]; // set segment A from LUT
		PORTB |= (1 << PB0);  // enable 1st display segment
	}
	else {
		PORTB &= ~(1 << PB0); // disable 1st display element
		element = 0; // set flag for next element
		PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5));
		PORTD &= ~(1 << PD7);
		PORTC |= segments_lut_1[score_b]; // set segment G-B from LUT
		PORTD |= segments_lut_2[score_b]; // set segment A from LUT
		PORTB |= (1 << PB1); // enable 2nd display segment
	}
}

void button_worker(void) {
  	static uint8_t btn = 0;
	uint16_t btn_current = 0;
  	static uint8_t btn_1_state = 0;
    	static uint8_t btn_2_state = 0;


	if(btn == 0){
		btn_current = analogRead(6);
		if(btn_1_state == 0 && (btn_current < 50)) {
			// taster gedrueckt, fallende flanke
			btn_a_state = 1;
			btn_a_hold = 0;
			btn_1_state = 1;
		}
		else if((btn_1_state == 1 || btn_1_state == 2) && (btn_current < 50)) {
			// taster gehalten
			//btn_a_state = 1;
			btn_a_hold++;
			btn_1_state = 2;
		}
		else if(btn_1_state == 2 && (btn_current > 50)) {
			// taster wird los gelassen
			//btn_a_state = 1;
			btn_a_hold = 0;
			btn_1_state = 3;
		}
		else if(btn_1_state == 3 && (btn_current > 50)) {
			// taster nicht gedrueckt
			//btn_a_state = 0;
			btn_a_hold = 0;
			btn_1_state = 0;
		}
		btn = 1;
	}
	else {
		btn_current = analogRead(7);
		if(btn_2_state == 0 && (btn_current < 50)) {
			// taster gedrueckt, fallende flanke
			btn_b_state = 1;timer_setup();
			btn_b_hold = 0;
			btn_2_state = 1;
		}
		else if((btn_2_state == 1 || btn_2_state == 2) && (btn_current < 50)) {
			// taster gehalten
			//btn_b_state = 1;
			btn_b_hold++;
			btn_2_state = 2;
		}
		else if(btn_2_state == 2 && (btn_current > 50)) {
			// taster wird los gelassen
			//btn_b_state = 1;
			btn_b_hold = 0;
			btn_2_state = 3;
		}
		else if(btn_2_state == 3 && (btn_current > 50)) {
			// taster nicht gedrueckt
			//btn_b_state = 1;
			btn_b_hold = 0;
			btn_2_state = 0;
		}
		btn = 0;
	}
}

uint8_t calc_crc8(uint8_t * dataIn, uint8_t len)// uses the ccitt generator polynom 0x07
{
  uint8_t i = 0, last_crc = 0;
  for (i = 0; i<len; i++){
    last_crc = _crc8_ccitt_update(last_crc, dataIn[i]);
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

void setup() {
        cli(); // global Interrupt disable, prevents flicker while initialzing

	segments_setup();
	timer_setup();
        pinMode(RFM24_SDN, OUTPUT);
        
        Serial.begin(115200);
        Serial.println("Kicker-Funkmaster up and running!");
        Serial.println("(c) by Patrick H. Jill H. Kjell-Arne L. Janina L. Moritz M. Jan N. Fabian S.");
        

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

uint8_t get_goals_from_slave(struct node slave) {
 	uint8_t data[4] = {};
  	uint8_t buf[4];
  	uint8_t res = 0;

  	uint8_t len = sizeof(buf);

  	data[0] = slave.address;
  	data[1] = MASTER_REQUEST_GOAL;
  	data[2] = DATA_EMPTY;
  	data[3] = calc_crc8(data, 3);

  	Serial.print("Sending Request to slave 0x");
  	Serial.print(slave.address, HEX);
  	Serial.println(":");
  	PrintHex8(data, 4);

  	rf24.send(data, sizeof(data));
  	rf24.waitPacketSent();

	if (rf24.waitAvailableTimeout(500)) {
		// Should be a reply message for us now
		if (rf24.recv(buf, &len))
		{
			Serial.print("got reply: ");
			PrintHex8(buf, 4);
			if(buf[0] == master.address && buf[1] == (SLAVE_GOALS_DETECTED+(slave.order_no*SLAVE_OFFSET)) && calc_crc8(buf, 4) == 0x00) {
				res = buf[2];

				data[0] = slave.address;
			  	data[1] = MASTER_SEND_GOAL_RST;
			  	data[2] = DATA_EMPTY;
			  	data[3] = calc_crc8(data, 3);
				Serial.print("Sending score reset request to slave 0x");
				Serial.print(slave.address, HEX);
  				Serial.println(":");
			  	PrintHex8(data, 4);
			}
			else {
				Serial.print("Package broken or wrong CRC8 from slave 0x");
				Serial.print(slave.address, HEX);
  				Serial.println("!");
				return -1;
			}
		}
		else {
			Serial.println("recv failed");
			return -1;
		}
	}
	else {
		Serial.print("No reply, is Slave 0x");
		Serial.print(slave.address, HEX);
  		Serial.println(" running?");
		return -1;
	}

	return res;

}



void loop() {
  	uint8_t slave_res = 0;
	// put your main code here, to run repeatedly:
	if( btn_a_hold > 46) {
		score_a = 0;
		score_b = 0;
		btn_a_hold = 0;
	}
	if( btn_b_state == 1) {
		btn_a_hold = 0;
	}
	if(btn_a_state == 1 && btn_b_state != 1) {
		if(score_a < 8 && score_b != 8) {
			score_a++;
                        Serial.print("Score for team A is ");
                        Serial.println(score_a);
                    }
		btn_a_state = 0;
	}

	if(btn_b_state == 1 && btn_a_state != 1) {
		if(score_b < 8 && score_a != 8) {
			score_b++;
                        Serial.print("Score for team B is ");
                        Serial.println(score_b);
		}
		btn_b_state = 0;
	}
	slave_res = get_goals_from_slave(slave_a);

	if(slave_res >= 0 && slave_res<=8, score_a < 8) {
		score_a += slave_res;
	}
	else {
		score_a = 10; // 11 represents E symbol for Error
	}

	slave_res = get_goals_from_slave(slave_b);

	if(slave_res >= 0 && slave_res<=8 && score_b < 8) {
		score_b += slave_res;
	}
	else {
		score_b = 10;
	}

}

ISR(TIMER2_OVF_vect) { // Will be called with roughly 183,1 Hz
	segments_worker(score_a, score_b);
	button_worker();
}
