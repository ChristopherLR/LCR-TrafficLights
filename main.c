#include <avr/io.h>
#include <avr/interrupt.h>

/* Scales to a standard of 1 sec, so scaler 0.5 would be half second
 * Be careful because the underlying value is 16 bits
 */
void configure_clock1(const float scaler);
void configure_int0();
void configure_int1();
void configure_spi();
void configure_port_expander();
char spi_master_transmit(char);
char spi_send(char, char);
char spi_read(char, char);

int main() {

  DDRD  |= (1 << PD5) | (1 << PD6);
	PORTD |= (1 << PD2) | (1 << PD3);

	PORTD = 0;

	configure_clock1(1);
	configure_int0();
	configure_int1();
	configure_spi();
	configure_port_expander();

	spi_send(0x14, 0x00);
	/* Enable Global interrupts */
	sei();

  while (1) {
	}
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

void configure_port_expander(){
	/* IOCON */
	spi_send(0x0A, 0b01000000);
	/* IODIRA Port A DDR */
	spi_send(0x00, 0x00);
	/* IODIRB Port B DDR */
	spi_send(0x01, 0xFF);
	/* GPPUB Port B pullups */
	spi_send(0x0D, 0xFF);
	/* GPINTENB Interrupt on change */
	spi_send(0x05, 0b00000001);
	/* INTCONB Compare DEFVAL=1 or Prev-Val=0 */
	spi_send(0x09, 0b00000001);
	/* DEFVAL - sets the default val */
	spi_send(0x07, 0b00000001);
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

	/* TCCR1B
	 * Timer 1 Control Reg B
	 * Bit 7: ICNC1 - Input Capture Noise Canceler
	 * Bit 6: ICES1 - Input Capture Edge Select
	 * Bit 4-3: WGM13/12 - Waveform Gen Mode
	 * Bit 2-1: Clock Select
	 */
	TCCR1B = 0b00000000; /* Stopping the clock */

	/* TCCR1A
	 * Timer 1 Control Reg A
	 * Bit 7-6: COM1A(1-0) - Compare Output mode channel A
	 * Bit 5-4: COM1B(1-0) - ** for A
	 * Bit 1-0: WGM1(1-0) - Wave Gen Mode
	 */
	TCCR1A = 0b00000000;

	/* OCR1B - (OCR1AH & OCR1AL) - 16 Bit
	 * Output Compare Register A
	 * Bit 15-0: Output to compare to timer
	 * 16,000,000/1024 = 15625 Ticks per sec
	 */
	OCR1B = 15625*scaler;

	/* TIFRx
	 * Timer Interrupt Flag Registers
	 * Bit 7-3: Nothing
	 * Bit 2: OCFxB - Compare Match B
	 * Bit 1: OCFxA - Compare Match A
	 * Bit 0: TOVx - Timer Overflow flag
	 */
	TIFR1 = 0b00000100;

	/* To Set WGM = 0000 and Prescaler to 1024
	 * TCCR1B = [00][0][00][101]
	 * TCCR1C = [FOC1A][FOC1B][5-0: Nothing]
	 */
	TCCR1B = 0b00000101;
	TCCR1C = 0b00000000;

	/* TCNT1 - (TCNT1H & TCNT1L) - 16 Bit
	 * Timer/Counter 1
	 * Bit 15-0: The value of the timer
	 */
	TCNT1 = 0;

};


