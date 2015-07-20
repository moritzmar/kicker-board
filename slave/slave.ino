#include <SPI.h>
#include <RH_RF24.h>

#define DEBUG 1

#define RFM24_SS 6
#define RFM24_MOSI 11
#define RFM24_MISO 12 
#define RFM24_IRQ 2
#define RFM24_SDN 5
RH_RF24 rf24 = RH_RF24(RFM24_SS, RFM24_IRQ, RFM24_SDN); //Initialize a new instance of the RadioHead RFM24W driver as rf24


static uint8_t score_a = 0;
static uint8_t score_b = 0;

volatile uint16_t btn_a_state = 0;
volatile uint16_t btn_b_state = 0;

volatile uint16_t btn_a_hold = 0;
volatile uint16_t btn_b_hold = 0;

const uint8_t segments_lut_1 [] =  {0x3E, 0x30, 0x2D, 0x39, 0x33, 0x1B, 0x1F, 0x30, 0x3F, 0x3B};
const uint8_t segments_lut_2 [] =  {0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80};

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


void setup() {
        cli(); // global Interrupt disable, prevents flicker while initialzing
	// put your setup code here, to run once:
	segments_setup();
	timer_setup();
        
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
        Serial.println("Battery voltage is");
        Serial.println(rf24.get_battery_voltage());
        Serial.println("Temperature is");
        Serial.println(rf24.get_temperature());
        #endif 
        
        rf24.setModemConfig(RH_RF24::GFSK_Rb5Fd10);
        rf24.setTxPower(0x4f);
        #ifdef DEBUG 
        Serial.println("Modem configured in GFSK modulation with 5kbs and 10kHz decimation");
        #endif 
        sei(); // global Interrupt enable
}  

void rfm24_on() {
  rf24.setModeIdle();
  rf24.setFrequency(433.000);
}

uint8_t demo_pkg[] = {0xDE, 0xAD, 0xBE, 0xEF};

void loop() {
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
                        rf24.send(demo_pkg, sizeof(demo_pkg));

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
}

ISR(TIMER2_OVF_vect) { // Will be called with roughly 183,1 Hz
	segments_worker(score_a, score_b);
	button_worker();
}
