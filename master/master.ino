/*
 *  Kicker-Funkmaster
 *  by P. Hanneken; J. Huptas; K-A. Lehmann; J. Linssen; M. Martinius; J. Niehr; F. Schneider
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

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

uint8_t game_end = 0;

static uint8_t score_a = 0;
static uint8_t score_b = 0;

volatile uint16_t btn_a_state = 0;
volatile uint16_t btn_b_state = 0;

volatile uint16_t btn_a_hold = 0;
volatile uint16_t btn_b_hold = 0;

const uint8_t segments_lut_1 [] =  {0x3E, 0x30, 0x2D, 0x39, 0x33, 0x1B, 0x1F, 0x30, 0x3F, 0x3B, 0x0F, 0x00, 0x01}; // 0 1 2 3 4 5 6 7 8 9 E OFF -
const uint8_t segments_lut_2 [] =  {0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00}; // 0 1 2 3 4 5 6 7 8 9 E OFF -


//Initialize a new instance of the RadioHead RFM24W driver as rf24 from the pin defines above
RH_RF24 rf24 = RH_RF24(RFM24_SS, RFM24_IRQ, RFM24_SDN);


/*
 * void timer_setup ()
 * Sets up the Timer2 for multiplexing and debouncing (see ISR(TIMER2_OVF_vect))
 */
void timer_setup(void) {
	TCCR2B |= ((1 << CS22) | (1 << CS21) ); // Prescaler 256 on Timer2
	TIMSK2 |= ((1 << TOIE0)); // Overflow Interrupt enable on Timer 2
	GTCCR &= (~(1 << TSM)); // Start all timers by disabling the synced halt
}


/*
 * void segments_setup()
 * Sets up the 7-segment display module. Note that the pins for the segments are scattered to PORTC and PORTD
 */
void segments_setup(void) {
  	DDRC |= ( (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5)); // Output pins for 7 segment G-B
	DDRD |= (1 << PD7); // Output pin for 7 segment A

	DDRB |= (1 << PB0) | (1 << PB1); // 7 segment drive transistor
}

/*
 * void segments_worker(uint8_t score_a, uint8_t score_b)
 * arg uint8_t score_a: left score to display
 * arg uint8_t score_b: right score to display
 * note that the values above 9 are special chars, refer to LUT
 *
 * worker function for segment multiplexing, gets periodically called by TIMER2
 */

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

/*
 * void button_worker()
 *
 * worker function for buttons, gets periodically called by TIMER2
 * note that the buttons are connected to analog inputs only, so they can only be read by analogRead()
 */

void button_worker(void) {
  	static uint8_t btn = 0;
	uint16_t btn_current = 0;
  	static uint8_t btn_1_state = 0;
    	static uint8_t btn_2_state = 0;


	if(btn == 0){
		btn_current = analogRead(6);
		if(btn_1_state == 0 && (btn_current < 50)) {
			// button pressed, falling edge
			btn_a_state = 1;
			btn_a_hold = 0;
			btn_1_state = 1;
		}
		else if((btn_1_state == 1 || btn_1_state == 2) && (btn_current < 50)) {
			// button hold
			btn_a_hold++;
			btn_1_state = 2;
		}
		else if(btn_1_state == 2 && (btn_current > 50)) {
			// button released, rising edge
			btn_a_hold = 0;
			btn_1_state = 3;
		}
		else if(btn_1_state == 3 && (btn_current > 50)) {
			// button not pressed
			btn_a_hold = 0;
			btn_1_state = 0;
		}
		btn = 1;
	}
	else {
		btn_current = analogRead(7);
		if(btn_2_state == 0 && (btn_current < 50)) {
			// button pressed, falling edge
			btn_b_state = 1;
			btn_b_hold = 0;
			btn_2_state = 1;
		}
		else if((btn_2_state == 1 || btn_2_state == 2) && (btn_current < 50)) {
			// button hold
			btn_b_hold++;
			btn_2_state = 2;
		}
		else if(btn_2_state == 2 && (btn_current > 50)) {
			// button released, rising edge
			btn_b_hold = 0;
			btn_2_state = 3;
		}
		else if(btn_2_state == 3 && (btn_current > 50)) {
			// button not pressed
			btn_b_hold = 0;
			btn_2_state = 0;
		}
		btn = 0;
	}
}

/*
 * uint8_t calc_crc8(uint8_t * dataIn, uint8_t len)
 * arg uint8_t * dataIn: Points to the data array of the crc8 to generate
 * arg uint8_t len: gives the length of the data to read. Can be zero! Care for out-of-boundary calls!
 * ret uint8_t: returns the calculated CRC8
 *
 * This functions calculates CRC8 sum for the given data with the CCITT polynomial 0x07
 */

uint8_t calc_crc8(uint8_t * dataIn, uint8_t len)
{
  uint8_t i = 0, last_crc = 0;
  for (i = 0; i<len; i++){ // update crc for each byte in data in
    last_crc = _crc8_ccitt_update(last_crc, dataIn[i]);
  }
  return last_crc;
}

/*
 * void PrintHex8(uint8_t *data, uint8_t len)
 * arg uint8_t * data: Points to the data to print
 * arg uint8_t len: gives the length of the data to read. Can be zero! Care for out-of-boundary calls!
 *
 * Used for debugging to print RF packages
 */

void PrintHex8(uint8_t *data, uint8_t len)
{
       Serial.print("0x");
       for (int i=0; i<len; i++) {
         if (data[i]<0x10) {Serial.print("0");}
         Serial.print(data[i],HEX);
         Serial.print(" ");
       }
	Serial.println();
}

/*
 * uint8_t get_goals_from_slave(struct node slave)
 * arg struct node slave: input the slave struct you want to send to
 * ret uint8_t: returns the number of goals the slave buffered (max 9). Returns 0 if slave does not answer. Returns -1 if package is corrupted.
 *
 */

uint8_t get_goals_from_slave(struct node slave) {
 	uint8_t data[4] = {};
  	uint8_t buf[4];
  	uint8_t res = 0;

  	uint8_t len = sizeof(buf);

  	// build package for goal request
  	data[0] = slave.address;
  	data[1] = MASTER_REQUEST_GOAL;
  	data[2] = DATA_EMPTY;
  	data[3] = calc_crc8(data, 3);
	#ifdef DEBUG
	  	Serial.print("Sending Request to slave 0x");
	  	Serial.print(slave.address, HEX);
	  	Serial.println(":");
	  	PrintHex8(data, 4);
	#endif

  	// send goal request
  	rf24.send(data, sizeof(data));
  	rf24.waitPacketSent();

	if (rf24.waitAvailableTimeout(500)) { // wait for package and get in from the transceiver
		if (rf24.recv(buf, &len))
		{
			#ifdef DEBUG
				Serial.print("INFO: got reply: ");
				PrintHex8(buf, 4);
				Serial.println();
			#endif
			// check if valid package is recevived
			if(buf[0] == master.address && buf[1] == (SLAVE_GOALS_DETECTED+(slave.order_no*SLAVE_OFFSET)) && calc_crc8(buf, 4) == 0x00 ) {
				if(buf[2] ==0 ) {
					return 0; // bail out if score is 0
				}
				res = buf[2]; // score is valid, save to res

				// package for resetting the score on the slave board
				data[0] = slave.address;
			  	data[1] = MASTER_SEND_GOAL_RST;
			  	data[2] = DATA_EMPTY;
			  	data[3] = calc_crc8(data, 3);
				#ifdef DEBUG
					Serial.print("INFO: Sending score reset request to slave 0x");
					Serial.print(slave.address, HEX);
	  				Serial.println(":");
				  	PrintHex8(data, 4);
				#endif
				// send score reset request
				rf24.send(data, sizeof(data));
  				rf24.waitPacketSent();
			}
			else {
				Serial.print("ERROR: Package broken or wrong CRC8 from slave 0x");
				Serial.print(slave.address, HEX);
  				Serial.println("!");
				return -1; // hard failure, bail out
			}
		}
		else {
			Serial.println("WARNING: Receiving failed!");
			return 0; // soft failure, just dont update score
		}
	}
	else {
		Serial.print("WARNING: No reply, is Slave 0x");
		Serial.print(slave.address, HEX);
  		Serial.println(" running?");
		return 0; // soft failure, just dont update score
	}

	return res; // return score if everything succeeds

}

void setup() {
        cli(); // global Interrupt disable, prevents flicker while initialzing

	segments_setup();
	timer_setup();

        Serial.begin(115200);
        Serial.println("Kicker-Funkmaster up and running!");
        Serial.println("by P. Hanneken; J. Huptas; K-A. Lehmann; J. Linssen; M. Martinius; J. Niehr; F. Schneider");
  	Serial.println("This program is open source and licensed under the conditions of the GNU GPL v3!");
  	Serial.println("This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.");

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
  	uint8_t slave_res = 0;

	if( btn_a_hold > 46) {
		score_a = 0;
		score_b = 0;
		btn_a_hold = 0;
		game_end = 0;
	}

	if( btn_b_state == 1) {
		btn_a_hold = 0;
	}

	if(game_end == 0) {
		if(btn_a_state == 1 && btn_b_state != 1) {
			score_a++;
			btn_a_state = 0;
		}
		if(btn_b_state == 1 && btn_a_state != 1) {
			score_b++;
			btn_b_state = 0;
		}

		slave_res = get_goals_from_slave(slave_a);

		if(slave_res >= 0 && slave_res<=9 && score_a <= 9) {
			score_a += slave_res;
		}
		else if (slave_res = -1){
			score_b = 10;
		}

		slave_res = get_goals_from_slave(slave_b);

		if(slave_res >= 0 && slave_res<=9 && score_b <= 9) {
			score_b += slave_res;
		}
		else if (slave_res = -1){
			score_b = 10;
		}
	}

	if(score_a >= 10 && game_end == 0) {
		game_end = 1;
		Serial.println("Team A won the game!");
		score_a = 12;
		score_b = 11;
	}
	if(score_b >= 10 && game_end == 0) {
		game_end = 1;
		Serial.println("Team B won the game!");
		score_b = 12;
		score_a = 11;
	}
}

ISR(TIMER2_OVF_vect) { // Will be called with roughly 183,1 Hz
	segments_worker(score_a, score_b);
	button_worker();
}
