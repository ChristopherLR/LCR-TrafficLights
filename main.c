#include <avr/io.h>
#include <avr/interrupt.h>

/* AVR definitions*/
#define FOSC 16000000
#define I2C_FREQ 40000
//#define TWI_BIT_RATE ((FOSC/I2C_FREQ)-16)/2
#define TWI_BIT_RATE 193

/* Addresses for PORT EXPANDER
 * Assumes IOCON.BANK = 0 */
#define IODIRA   0x00
#define IODIRB   0x01
#define IPOLA    0x02
#define IPOLB    0x03
#define GPINTENA 0x04
#define GPINTENB 0x05
#define DEFVALA  0x06
#define DEFVALB  0x07
#define INTCONA  0x08
#define INTCONB  0x09
#define IOCON    0x0A
#define GPPUA    0x0C
#define GPPUB    0x0D
#define INTFA    0x0E
#define INTFB    0x0F
#define INTCAPA  0x10
#define INTCAPB  0x11
#define GPIOA    0x12
#define GPIOB    0x13
#define OLATA    0x14
#define OLATB    0x15

/* TWI Definitions */
#define OWN_ADR       00
#define SUCCESS       0x53
#define FAIL          0x46
#define START         0x10
#define ADR_ACK       0x18
#define DATA_ACK      0x28

/* LCD Definitions */
#define BACKLIGHT     0x08
#define DATA_BYTE     0x01
#define LCD_ADDRESS   0x27
#define LCD_CLEAR     0x01
#define LCD_POS       0x80
#define FOUR_BIT_MODE 0x28

/* LCD POSITIONS */
#define A 0
#define B 1
#define C 2
#define D 3
#define E 4
#define F 5
#define G 6

/* Encapsulating the twi transmission into a struct */
typedef struct {
	unsigned char address;    // Address of slave
	unsigned char num_bytes;  // Number of bytes in data
	unsigned char lower;      // Setting the Backlight/Data
	unsigned char *data;      // Pointer to the data for transmission
} twi_frame;

typedef struct {
  char green_counter;
  char orange_counter;
  char red_counter;
} traffic_light;

char get_current_light(traffic_light);

typedef struct {
  char out_a;
  char out_b;
  char in;
} port_expander;

typedef struct {
  traffic_light * tlight_1;
  traffic_light * tlight_2;
} mcu_state;

/* Scales to a standard of 1 sec, so scaler 0.5 would be half second
 * Be careful because the underlying value is 16 bits */
void configure_clock1(float scaler);

void configure_int0();
void configure_int1();
char analog_init();
unsigned char analog_read();
void check_analog();
char check_hazard();
void increment_state();
char check_next_state_1();
char check_next_state_2();
void hazard();
void configure_spi();
void configure_port_expander_a();
void read_port_expander_a();
void configure_port_expander_b();
void read_port_expander_b();
void transfer_state();
char spi_master_transmit(char);
char spi_send(char, char, char);
char spi_read(char, char, char);
char twi_init();
unsigned char twi_start();
void twi_wait();
void twi_stop();
void display_state();
unsigned char twi_send_addr(unsigned char);
unsigned char twi_send_byte(unsigned char, unsigned char);
unsigned char twi_send_nibble(unsigned char);
unsigned char twi_send_data(twi_frame);
unsigned char lcd_position(unsigned char);
unsigned char lcd_init();
unsigned char lcd_write_str(char*, unsigned char, unsigned char);
void display_counter();
void ERROR();

/* Global variables */
float timer_scaler = 0.1;
unsigned char LED_VAL= 0xFF;
char HAZARD_COUNTER = 0;
unsigned char HAZARD_LED = 0b00100010;
unsigned char TICK = 1;
char CHECK_PE = 0;
unsigned char LED_DISP1[] = "                ";
unsigned char LED_DISP2[] = "                ";
traffic_light tlight_a = {20, 0, 0};
traffic_light tlight_b = {20, 0, 0};
traffic_light tlight_c = {0, 0, 0};
traffic_light tlight_d = {0, 0, 0};
traffic_light tlight_e = {0, 0, 0};
traffic_light tlight_f = {0, 0, 0};
traffic_light tlight_g = {0, 0, 0};
traffic_light tlight_non = {0, 0, 0};
mcu_state state = {&tlight_a, &tlight_b};
unsigned char state_counter_1 = 0;
unsigned char state_counter_2 = 0;

/* [][][][][G][B][D][F] */
port_expander PE_A = {0b00010001, 0b00010100, 0};
/* [][][][][][E][A][C] */
port_expander PE_B = {0b00010100, 0b00000001, 0};

int main() {

  /* Set hazard input */
  DDRD  &= 0b01111111;
  PORTD |= 0b10000000;

	configure_clock1(1);
	configure_int0();
	configure_int1();
	configure_spi();
  analog_init();
	configure_port_expander_a();
	configure_port_expander_b();
	twi_init();

  /* Clear any interrupts that may have occurred */
	spi_read(GPIOA, 0x00, 'A');
	spi_read(GPIOB, 0x00, 'A');
	spi_read(GPIOA, 0x00, 'B');
	spi_read(GPIOB, 0x00, 'B');
	unsigned char state = SUCCESS;

	state = lcd_init();

  spi_send(OLATA, 0xFF, 'A');
  spi_send(OLATB, 0xFF, 'A');
  spi_send(OLATA, 0xFF, 'B');
  spi_send(OLATB, 0xFF, 'B');

	/* Enable Global interrupts */
	sei();

	unsigned char str[] = "HELLO WORLD";
	state = lcd_write_str(str, 0x40, 11);

	if(state != SUCCESS) ERROR();


	while (1) {
    check_hazard();
    if(TICK){
      check_analog();
      if(HAZARD_COUNTER) {
        hazard();
      }
      TICK = 0;
      display_state();
    }
    if(CHECK_PE > 0){
        read_port_expander_a();
        read_port_expander_b();
        CHECK_PE --;
    }
  }
	return 0;
};

/* PE_A */
ISR(INT0_vect){
  CHECK_PE++;
  //read_port_expander_a();
  LED_DISP2[14] = 'A';
  return;
};

/* PE_B */
ISR(INT1_vect){
  CHECK_PE++;
  //read_port_expander_b();
  LED_DISP2[15] = 'B';
  return;

};

ISR(TIMER1_COMPB_vect){
	/* TCNT1 - (TCNT1H & TCNT1L) - 16 Bit
	 * Timer/Counter 1
	 * Bit 15-0: The value of the timer
	 */
	TCNT1 = 0;
  TICK = 1;
  configure_clock1(timer_scaler);
};

void display_state(){
  LED_DISP1[F] = (PE_A.in & 0b0001) ? 'X' : ' ';
  LED_DISP1[D] = (PE_A.in & 0b0010) ? 'X' : ' ';
  LED_DISP1[B] = (PE_A.in & 0b0100) ? 'X' : ' ';
  LED_DISP1[G] = (PE_A.in & 0b1000) ? 'X' : ' ';
  LED_DISP1[C] = (PE_B.in & 0b0001) ? 'X' : ' ';
  LED_DISP1[A] = (PE_B.in & 0b0010) ? 'X' : ' ';
  LED_DISP1[E] = (PE_B.in & 0b0100) ? 'X' : ' ';
  LED_DISP2[A] = get_current_light(tlight_a);
  LED_DISP2[B] = get_current_light(tlight_b);
  LED_DISP2[C] = get_current_light(tlight_c);
  LED_DISP2[D] = get_current_light(tlight_d);
  LED_DISP2[E] = get_current_light(tlight_e);
  LED_DISP2[F] = get_current_light(tlight_f);
  LED_DISP2[G] = get_current_light(tlight_g);

  if(HAZARD_COUNTER > 0){
    spi_send(OLATA, HAZARD_LED, 'A');
    spi_send(OLATB, HAZARD_LED, 'A');
    spi_send(OLATA, HAZARD_LED, 'B');
    spi_send(OLATB, HAZARD_LED, 'B');
  } else {
    increment_state();
    transfer_state();
    spi_send(OLATA, PE_A.out_a, 'A');
    spi_send(OLATB, PE_A.out_b, 'A');
    spi_send(OLATA, PE_B.out_a, 'B');
    spi_send(OLATB, PE_B.out_b, 'B');
  }
  display_counter();
  lcd_write_str(LED_DISP1, 0x00, 16);
  lcd_write_str(LED_DISP2, 0x40, 16);

};

void display_counter(){
  unsigned char tens = 0;
  unsigned char units = 0;

  tens = (state_counter_1/10)%10;
  units = state_counter_1%10;
  LED_DISP1[12] = '0' + tens;
  LED_DISP1[13] = '0' + units;

  tens = (state_counter_2/10)%10;
  units = state_counter_2%10;
  LED_DISP1[14] = '0' + tens;
  LED_DISP1[15] = '0' + units;

}

void increment_state(){
  traffic_light * tlight_1 = state.tlight_1;
  traffic_light * tlight_2 = state.tlight_2;

  if(tlight_1->green_counter > 1){
    tlight_1->green_counter --;
    state_counter_1 = tlight_1 -> green_counter;
  } else if( tlight_1->green_counter > 0){
    if(((tlight_1 == &tlight_a) || (tlight_1 == &tlight_b)) && check_next_state_1()){
      tlight_1->green_counter = 6;
      state_counter_1 = tlight_1 -> green_counter;
    } else {
      tlight_1->green_counter = 0;
      tlight_1->orange_counter = 4;
      state_counter_1 = tlight_1 -> orange_counter;
    }
  } else if ( tlight_1->orange_counter > 1){
    tlight_1->orange_counter --;
    state_counter_1 = tlight_1 -> orange_counter;
  } else if ( tlight_1->orange_counter > 0){
    tlight_1->orange_counter = 0;
    tlight_1->red_counter = 3;
    state_counter_1 = tlight_1 -> red_counter;
  } else {
    ERROR();
  }

  if(tlight_2->green_counter > 1){
    tlight_2->green_counter --;
    state_counter_2 = tlight_2 -> green_counter;
  } else if( tlight_2->green_counter > 0){
    if(((tlight_2 == &tlight_a) || (tlight_2 == &tlight_b)) && check_next_state_2()){
      tlight_2->green_counter = 6;
      state_counter_2 = tlight_2 -> green_counter;
    } else {
      tlight_2->green_counter = 0;
      tlight_2->orange_counter = 4;
      state_counter_2 = tlight_2 -> orange_counter;
    }
  } else if ( tlight_2->orange_counter > 1){
    tlight_2->orange_counter --;
    state_counter_2 = tlight_2 -> orange_counter;
  } else if ( tlight_2->orange_counter > 0){
    tlight_2->orange_counter = 0;
    tlight_2->red_counter = 3;
    state_counter_2 = tlight_2 -> red_counter;
  } else {
    ERROR();
  }
}

/* Returns true if next state stays green */
char check_next_state_1(){
  return 1;
}

char check_next_state_2(){
  return 1;
}

void transfer_state(){
  PE_B.out_a = 0;
  PE_B.out_b = 0;
  switch(get_current_light(tlight_a)){
  case 'G':
    PE_B.out_a |= 0b00000100;
    break;
  case 'Y':
    PE_B.out_a |= 0b00000010;
    break;
  case 'R':
    PE_B.out_a |= 0b00000001;
    break;
  }
  switch(get_current_light(tlight_c)){
  case 'G':
    PE_B.out_a |= 0b01000000;
    break;
  case 'Y':
    PE_B.out_a |= 0b00100000;
    break;
  case 'R':
    PE_B.out_a |= 0b00010000;
    break;
  }
  switch(get_current_light(tlight_e)){
  case 'G':
    PE_B.out_b |= 0b00000100;
    break;
  case 'Y':
    PE_B.out_b |= 0b00000010;
    break;
  case 'R':
    PE_B.out_b |= 0b00000001;
    break;
  }
  PE_A.out_a = 0;
  PE_A.out_b = 0;
  switch(get_current_light(tlight_g)){
  case 'G':
    PE_A.out_b |= 0b00000100;
    break;
  case 'Y':
    PE_A.out_b |= 0b00000010;
    break;
  case 'R':
    PE_A.out_b |= 0b00000001;
    break;
  }
  switch(get_current_light(tlight_b)){
  case 'G':
    PE_A.out_b |= 0b01000000;
    break;
  case 'Y':
    PE_A.out_b |= 0b00100000;
    break;
  case 'R':
    PE_A.out_b |= 0b00010000;
    break;
  }
  switch(get_current_light(tlight_f)){
  case 'G':
    PE_A.out_a |= 0b01000000;
    break;
  case 'Y':
    PE_A.out_a |= 0b00100000;
    break;
  case 'R':
    PE_A.out_a |= 0b00010000;
    break;
  }
  switch(get_current_light(tlight_d)){
  case 'G':
    PE_A.out_a |= 0b00000100;
    break;
  case 'Y':
    PE_A.out_a |= 0b00000010;
    break;
  case 'R':
    PE_A.out_a |= 0b00000001;
    break;
  }
}

char get_current_light(traffic_light tlight){
  char retv = '?';
  retv = (tlight.green_counter) ? 'G' :
    (tlight.orange_counter) ? 'Y' : 'R';
  return retv;
}

void check_analog(){
  unsigned char an = analog_read();
  float scaled = 0;
  if(an < 20 ){
    scaled = 0;
  } else if(an < 50){
    scaled = 0.2;
  } else if(an < 120){
    scaled = 0.4;
  } else if( an < 170){
    scaled = 0.6;
  } else if( an < 220){
    scaled = 0.8;
  } else scaled = 0.9;
  timer_scaler = 1.0 - scaled;
}

void configure_int0(){
	/* EIMSK - External Interrupt Mask
	 * Bit 7-2: Nothing
	 * Bit 1: INT1
	 * Bit 0: INT0
	 */
	EIMSK |= 0b00000001;

	/* EICRA - External Int Control Reg
	 * [-][-][-][-][ISC11][ISC10][ISC01][ISC00]
	 * ISC1(1-0) - INT1
	 * ISC0(1-0) - INT0
	 * ISCx: 00 low level generate int req
	 * ISCx: 01 logical change generates
	 * ISCx: 10 Falling edge
	 * ISCx: 11 Rising edge
	 */
	EICRA |= 0b00000010;
};

void configure_int1(){
	/* EIMSK - External Interrupt Mask
	 * Bit 7-2: Nothing
	 * Bit 1: INT1
	 * Bit 0: INT0
	 */
	EIMSK |= 0b00000010;

	/* EICRA - External Int Control Reg
	 * [-][-][-][-][ISC11][ISC10][ISC01][ISC00]
	 * ISC1(1-0) - INT1
	 * ISC0(1-0) - INT0
	 * ISCx: 00 low level generate int req
	 * ISCx: 01 logical change generates
	 * ISCx: 10 Falling edge
	 * ISCx: 11 Rising edge
	 */
	EICRA |= 0b00001000;
};

char check_hazard(){
  if(!(PIND & 0b10000000)){
    if(!HAZARD_COUNTER) {
      HAZARD_COUNTER = 60;
      PE_A.in = 0;
      PE_B.in = 0;
      LED_VAL = 0b00100010;
    }
  }
  return 1;
};

void hazard(){
  HAZARD_LED ^= 0b00100010;
  HAZARD_COUNTER--;
  if(HAZARD_COUNTER > 60) HAZARD_COUNTER = 0;
};

void configure_spi(){
	/* DDRB - Data Direction B
	 * [XTAL][XTAL][SCK][MISO0][MOSI0][!SS][-][-]
	 */
	DDRB = 0b00101111;
	PORTB |= 0b00000101; /* Setting Slaves back high */

	/* SPCR - SPI Control Register
	 * [SPIE][SPE][DORD][MSTR][CPOL][CPHA][SPR1][SPR0]
	 * SPIE - Interrupt Enable
	 * SPE - SPI Enable
	 * DORD - Data Order 1: LSB, 0: MSB
	 * MSTR - Master/Slave Select
	 * SPR - Clock Rate Select (used with SPI2X)
	 */
	SPCR |= (1 << SPE) | (1 << MSTR);

	/* SPSR - SPI Status Register
	 * [SPIF][WCOL][-][-][-][-][-][SPI2X]
	 */
	SPSR = 0;
};


char spi_master_transmit(char data){
	/* SPDR - SPI Data Register */
	SPDR = data;
	/* Wait for transfer */
	while(!(SPSR & (1 << SPIF)));
	return SPDR;
};

char spi_send(char cmd, char data, char chip){

	char retv = 0;

  if(chip == 'A'){
    /* Clear Bit 2 (SS) */
    PORTB &= 0b11111011;
  } else {
    /* Clear Bit 0 (SS) */
    PORTB &= 0b11111110;
  }
	spi_master_transmit(0x40);
	spi_master_transmit(cmd);
	retv = spi_master_transmit(data);

  if(chip == 'A'){
    /* Set   Bit 2 (SS) */
    PORTB |= 0b00000100;
  } else {
    /* Set   Bit 0 (SS) */
    PORTB |= 0b00000001;
  }

	return retv;
};

char spi_read(char cmd, char data, char chip){

	char retv = 0;

  if(chip == 'A'){
    /* Clear Bit 2 (SS) */
    PORTB &= 0b11111011;
  } else {
    /* Clear Bit 0 (SS) */
    PORTB &= 0b11111110;
  }

	spi_master_transmit(0x41);
	spi_master_transmit(cmd);
	retv = spi_master_transmit(data);

  if(chip == 'A'){
    /* Set   Bit 2 (SS) */
    PORTB |= 0b00000100;
  } else {
    /* Set   Bit 0 (SS) */
    PORTB |= 0b00000001;
  }

	return retv;
};

unsigned char swap(unsigned char x){
	return ((x & 0x0F)<<4 | (x & 0xF0)>>4);
};

void configure_port_expander_a(){
	/* IOCON */
	spi_send(IOCON, 0b01000000, 'A');
	/* IODIRx Port x DDR */
	spi_send(IODIRB, 0b10001000, 'A');
	spi_send(IODIRA, 0b10001000, 'A');
	/* GPPUx Port x pullups */
	spi_send(GPPUB, 0b10001000, 'A');
	spi_send(GPPUA, 0b10001000, 'A');
	/* GPINTENx Interrupt on change */
	spi_send(GPINTENB, 0b10001000, 'A');
	spi_send(GPINTENA, 0b10001000, 'A');
	/* INTCONx Compare DEFVAL=1 or Prev-Val=0 */
	spi_send(INTCONB, 0b10001000, 'A');
	spi_send(INTCONA, 0b10001000, 'A');
	/* DEFVAL - sets the default val */
	spi_send(DEFVALB, 0b10001000, 'A');
	spi_send(DEFVALA, 0b10001000, 'A');

};

void read_port_expander_a(){
  char tmp = 0;
  tmp = spi_read(GPIOA, 0x00, 'A');
  /* Checking F & D */
  if(!(tmp & 0b10000000)) PE_A.in |= 0b000000001;
  if(!(tmp & 0b00001000)) PE_A.in |= 0b000000010;

  tmp = spi_read(GPIOB, 0x00, 'A');
  /* Checking G & B */
  if(!(tmp & 0b10000000)) PE_A.in |= 0b000000100;
  if(!(tmp & 0b00001000)) PE_A.in |= 0b000001000;
}

void configure_port_expander_b(){
	/* IOCON */
	spi_send(IOCON, 0b01000000, 'B');
	/* IODIRx Port x DDR */
	spi_send(IODIRB, 0b00001000, 'B');
	spi_send(IODIRA, 0b10001000, 'B');
	/* GPPUx Port x pullups */
	spi_send(GPPUB, 0b00001000, 'B');
	spi_send(GPPUA, 0b10001000, 'B');
	/* GPINTENx Interrupt on change */
	spi_send(GPINTENB, 0b00001000, 'B');
	spi_send(GPINTENA, 0b10001000, 'B');
	/* INTCONx Compare DEFVAL=1 or Prev-Val=0 */
	spi_send(INTCONB, 0b00001000, 'B');
	spi_send(INTCONA, 0b10001000, 'B');
	/* DEFVAL - sets the default val */
	spi_send(DEFVALB, 0b00001000, 'B');
	spi_send(DEFVALA, 0b10001000, 'B');
};

void read_port_expander_b(){
  char tmp = 0;

  tmp = spi_read(GPIOA, 0x00, 'B');
  /* Checking C & A */
  if(!(tmp & 0b10000000)) PE_B.in |= 0b000000001;
  if(!(tmp & 0b00001000)) PE_B.in |= 0b000000010;

  tmp = spi_read(GPIOB, 0x00, 'B');
  /* Checking E */
  if(!(tmp & 0b00001000)) PE_B.in |= 0b000000100;
};

void configure_clock1(const float scaler){

	/* TIMSK1
	 * Timer/Counter 1 Interrupt Mask Register
	 * Bit 5: ICF1 - Input Capture Flag
	 * Bit 2: OCF1B - Output Compare B Match
	 * Bit 1: OCF1A -
	 * bit 0: TOV1 - Overflow Flag
	 */
	TIMSK1 = 0b00000100;

	/* TCCR1B - Timer 1 Control Reg B
	 * [ICNC1][ICES1][-][WGM13][WGM12][CS12][CS11][CS10]
	 * ICNC1 - Input Capture Noise Canceler
	 * ICES1 - Input Capture Edge Select
	 * WGM - Waveform Gen Mode - Setting PWM/Normal
	 * CS - Clock Select - Pre-scaling */
	TCCR1B = 0b00000000; /* Stopping the clock */

	/* TCCR1A - Timer 1 Control Reg A
	 * [COM1A1][COM1A0][COM1B1][COM1B0][-][-][WGM11][WGM10]
	 * COM1A - Compare Output mode channel A
	 * COM1B - ** for A
	 * WGM - Wave Gen Mode */
	TCCR1A = 0b00000000;

	/* OCR1B - (OCR1AH & OCR1AL) - 16 Bit
	 * Output Compare Register A
	 * Bit 15-0: Output to compare to timer
	 * 16,000,000/1024 = 15625 Ticks per sec */
	OCR1B = 15625*scaler;

	/* TIFRx - Timer Interrupt Flag Registers
	 * [-][-][-][-][-][OCFxB][OCFxA][TOVx]
	 * OCFxB - Compare Match B
	 * OCFxA - Compare Match A
	 * TOVx - Timer Overflow flagc */
	TIFR1 = 0b00000100;

	/* To Set WGM = 0000 and Prescaler to 1024
	 * TCCR1B = [00][-][00][101]
	 * TCCR1C = [FOC1A][FOC1B][-][-][-][-][-][-]
	 * FOC - Force Output Compare for A/B */
	TCCR1B = 0b00000101;
	TCCR1C = 0b00000000;

	/* TCNT1 - (TCNT1H & TCNT1L) - 16 Bit
	 * Timer/Counter 1
	 * Bit 15-0: The value of the timer */
	TCNT1 = 0;

};

char analog_init(){
  DDRC &= 0b11111110;

  /* ADMUX - ADC Mulitplexer Selection
   * [REFS1][REFS0][ADLAR][-][MUX3][MUX2][MUX1][MUX0]
   */
  ADMUX = 0b01100000;

  /* ADCSRA - ADC Control and Status Reg
   * [ADEN][ADSC][ADATE][ADIF][ADIE][ADPS2][ADPS1][ADPS0]
   * ADEN = Interrupt Enable
   * ADSC = ADC Start Conversion
   * ADATE = ADC Auto Trigger Enable
   * ADIF = ADC Interrupt Flag
   * ADIE = ADC Interrupt Enable
   * ADPS = Prescaler Select Bits
   */
  ADCSRA = 0b10000111;

  return 1;
}

unsigned char analog_read(){
  ADCSRA |= (1 << ADSC);
  while(ADCSRA & (1 << ADSC));

  return ADCH;
}

char twi_init(){

	DDRC &= 0b11001111;

	PORTC |= 0b00110000;

	/* TWAR - TWI Address */
	TWAR = OWN_ADR;

	/* TWBR - TWI Bit Rate Reg
	 * SCL = Fosc / (16 + 2(TWBR).(TWPS))
	 * We want SCL = 40KHz
	 * Fosc = 16MHz
	 * TWBR = 193 */
	TWBR = TWI_BIT_RATE;

	TWCR = (1 << TWEN);

	return 1;
};

unsigned char twi_start(){
	/* TWCR - TWI Control Register
	 * [TWINT][TWEA][TWSTA][TWSTO][TWWC][TWEN][-][TWIE]
	 * TWI Interrupt = 1 to clear flag, set when job is done
	 * TWI Enable Ack = 1 ack pulse gen when conditions met
	 * TWI STArt = 1 -> Generates start condition when avail
	 * TWI STop = 1 -> Gen stop cond, cleared automaticall
	 * TWI Write Collision = 1 when writing to TWDR and TWINT low
	 * TWI Enable Bit = Enable TWI transmission
	 * TWI Interrupt Enable = */
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	twi_wait();

	/* TWSR - TWI Status Register
	 * [TWS7][TWS6][TWS5][TWS4][TWS3][-][TWPS1][TWPS0]
	 * TWI Status = 5 Bit status of TWI
	 * TWI PreScaler = sets the pre-scaler (1, 4, 16, 64) */
	if(TWSR != START) return FAIL;

	return SUCCESS;
};

void twi_stop(){
	/* Sending the stop condition */
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
};

unsigned char twi_send_data(twi_frame tx_frame){
	unsigned char state, i;
	state = SUCCESS;

	state = twi_start();
	if(state == SUCCESS){
		state = twi_send_addr(tx_frame.address);
	} else {
		twi_stop();
		return state;
	}

  for(i = 0; ((i<tx_frame.num_bytes)&&(state=SUCCESS)); i++)
				state = twi_send_byte(tx_frame.data[i],tx_frame.lower);

	twi_stop();

	return state;
};

unsigned char twi_send_byte(unsigned char data,
														unsigned char lower){
	unsigned char state = SUCCESS;
	unsigned char first_nib  = (data & 0xF0) | lower;
	unsigned char second_nib = ((data << 4)&0xF0) | lower;

	state = twi_send_nibble(first_nib);
	if(state != SUCCESS) return state;
	state = twi_send_nibble(second_nib);
	if(state != SUCCESS) return state;

	return state;
};

unsigned char twi_send_nibble(unsigned char nibble){
	unsigned char tx = nibble ;
	TWDR = tx | 0x04;
	TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
	twi_wait();

	if((TWSR&0xF8) != DATA_ACK) return TWSR;

	TWDR = tx & 0xFB;
	TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
	twi_wait();

	if((TWSR&0xF8) != DATA_ACK) return TWSR;
	return SUCCESS;
};

unsigned char twi_send_addr(unsigned char addr){

	twi_wait();

	TWDR = addr + addr; //send Address across twi
	TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
	twi_wait();
	if((TWSR&0xF8) != ADR_ACK) return TWSR; // Checking for ack
	return SUCCESS;

};

void twi_wait(){
	while(!(TWCR & (1 << TWINT)));
};

void ERROR(){
};

unsigned char lcd_init(){

	unsigned char state = SUCCESS;

	twi_start();
	twi_send_addr(LCD_ADDRESS);
	twi_send_nibble(0x00);
	twi_send_nibble(0x30);
	twi_send_nibble(0x30);
	twi_send_nibble(0x30);
	twi_send_nibble(0x30);
	twi_send_nibble(0x20);
	twi_send_byte(0x0C, 0x00);
	twi_send_byte(0x01, 0x00);
	twi_stop();

	twi_start();
	twi_send_addr(LCD_ADDRESS);
	state = twi_send_byte(0x80, BACKLIGHT);
	twi_stop();
	unsigned char* init_str_1 = "LCD INIT";
	twi_frame twi_init;
	twi_init.address = LCD_ADDRESS;
	twi_init.num_bytes = 8;
	twi_init.lower = (BACKLIGHT | DATA_BYTE);
	twi_init.data = init_str_1;
	state = twi_send_data(twi_init);


	return state;
};

unsigned char lcd_position(unsigned char pos){
	unsigned char state;

	twi_start();
	twi_send_addr(LCD_ADDRESS);
	state = twi_send_byte(pos | 0x80, BACKLIGHT);
	twi_stop();


	return state;
};

unsigned char lcd_write_str(char* str,
														unsigned char pos,
														unsigned char size){
	unsigned char state = SUCCESS;
	twi_frame twi_frame;

	state = lcd_position(pos);

	twi_frame.address = LCD_ADDRESS;
	twi_frame.lower = (BACKLIGHT | DATA_BYTE);
	twi_frame.num_bytes = size;
	twi_frame.data = str;

	state = twi_send_data(twi_frame);

	return state;
};
