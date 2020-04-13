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


/* Encapsulating the twi transmission into a struct */
typedef struct {
	unsigned char address;    // Address of slave
	unsigned char num_bytes;  // Number of bytes in data
	unsigned char lower;      // Setting the Backlight/Data
	unsigned char *data;      // Pointer to the data for transmission
} twi_frame;

/* Scales to a standard of 1 sec, so scaler 0.5 would be half second
 * Be careful because the underlying value is 16 bits */
void configure_clock1(const float scaler);
void configure_int0();
void configure_int1();
void configure_spi();
void configure_port_expander();
char spi_master_transmit(char);
char spi_send(char, char);
char spi_read(char, char);
char twi_init();
unsigned char twi_start();
void twi_wait();
void twi_stop();
unsigned char twi_send_addr(unsigned char);
unsigned char twi_send_byte(unsigned char, unsigned char);
unsigned char twi_send_nibble(unsigned char);
unsigned char twi_send_data(twi_frame);
unsigned char lcd_position(unsigned char);
unsigned char lcd_init();
unsigned char lcd_write_str(char*, unsigned char, unsigned char);
void ERROR();



int main() {

	DDRD  |= (1 << PD5) | (1 << PD6) | (1 << PD7);
	PORTD |= (1 << PD2) | (1 << PD3);

	PORTD = 0;

	configure_clock1(1);
	configure_int0();
	configure_int1();
	configure_spi();
	configure_port_expander();
	twi_init();

	spi_send(0x14, 0x00);
	unsigned char state = SUCCESS;

	state = lcd_init();

	/* Enable Global interrupts */
	sei();

	unsigned char str[] = "HELLO WORLD";
	state = lcd_write_str(str, 0x40, 11);

	if(state != SUCCESS) ERROR();

	while (1) {}
	return 0;
};

ISR(INT0_vect){
	PORTD ^= (1 << PD5);
};

ISR(INT1_vect){
	char PE_B = spi_read(0x13,0);
	PORTD ^= (1 << PD6);
};

ISR(TIMER1_COMPB_vect){
	//PORTD ^= (1 << PC5);
	/* TCNT1 - (TCNT1H & TCNT1L) - 16 Bit
	 * Timer/Counter 1
	 * Bit 15-0: The value of the timer
	 */
	TCNT1 = 0;
	spi_send(0x14, 0x00);
};

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


void configure_spi(){
	/* DDRB - Data Direction B
	 * [XTAL][XTAL][SCK][MISO0][MOSI0][!SS][-][-]
	 */
	DDRB = 0b00101111;
	PORTB |= 0b00000100; /* Setting Slave back high */

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

char spi_send(char cmd, char data){

	char retv = 0;

	/* Clear Bit 2 (SS) */
	PORTB &= 0b11111011;

	spi_master_transmit(0x40);
	spi_master_transmit(cmd);
	retv = spi_master_transmit(data);

	/* Set Bit 2 (SS) */
	PORTB |= 0b00000100;
	return retv;
};

char spi_read(char cmd, char data){

	char retv = 0;

	/* Clear Bit 2 (SS) */
	PORTB &= 0b11111011;

	spi_master_transmit(0x41);
	spi_master_transmit(cmd);
	retv = spi_master_transmit(data);

	/* Set Bit 2 (SS) */
	PORTB |= 0b00000100;
	return retv;
};

unsigned char swap(unsigned char x){
	return ((x & 0x0F)<<4 | (x & 0xF0)>>4);
};

void configure_port_expander(){
	/* IOCON */
	spi_send(IOCON, 0b01000000);
	/* IODIRA Port A DDR */
	spi_send(IODIRA, 0x00);
	/* IODIRB Port B DDR */
	spi_send(IODIRB, 0xFF);
	/* GPPUB Port B pullups */
	spi_send(GPPUB, 0xFF);
	/* GPINTENB Interrupt on change */
	spi_send(GPINTENB, 0b00000001);
	/* INTCONB Compare DEFVAL=1 or Prev-Val=0 */
	spi_send(INTCONB, 0b00000001);
	/* DEFVAL - sets the default val */
	spi_send(DEFVALB, 0b00000001);
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

char twi_init(){

	DDRC |= 0b00000001;
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
	PORTD |= 0b10000000;
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
