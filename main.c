/* Embedded Software - Project 2 - Bálint Roland
 *	- this time i followed the proposed code structure :)
 *	- all port initializations are in port_init(), hence some init() functions 
 *		do not cover the full initialization of the given device only by themselves 
 *	- instead of using new_timer and new_adc, we could have made sure that none of the messages get overridden
 *		- theoretically any sent message could be overridden if we press another key quickly enough
 *		- "fixing" this would over complicate the program (keep a queue of all actions waiting for USART)
 */

#include <avr/io.h>			// for standard library iom4809.h
#include <avr/interrupt.h>	// for ISR
#include <avr/cpufunc.h>	// for ccp_write_io		
#include <stdio.h>			// for sprintf()
#include <stdbool.h>		// for bool


#define F_CPU 20000000																		// main clock frequency
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)	// baud rate formula

/* 10-bit ADC => max count 1023
 * VDD reference voltage => max voltage 5 V
 *		5V   ... 1023
 *		3.5v ... 716.1
 */	
#define VOLT_3DOT5 716

/* similar reasoning:
 *		5000 mV  ... 1024
 *		   x mV  ... COUNT 
 */	
#define COUNT_TO_MILIVOLT(COUNT) ((uint32_t)COUNT * 5000 / 1023)


/* variables used in sending messages
 *	- must be global since it is used by both usart3_send_message and ISR(USART3_TXC_vect)
 *	- for more detailed explanation see the functions mentioned before
 */
unsigned char queue[50];			// queue of characters to be sent
unsigned char queue_counter = 0;	// number of characters in the queue
unsigned char sent_counter = 0;		// number of characters sent

// data related to the 555 timer
uint16_t period_555;
uint16_t high_time_555;
uint16_t low_time_555;

// ADC0.RES has 10-bits => uint8_t is not enough
uint16_t adc_reading;	

/* some custom flags used by the program
 *	- keyword volatile shows the compiler that the variable may be changed "out of scope" (ex: in an ISR)
 *	- basically it makes sure variable stays global even after optimization
 *	- in our case we compiled in debug mode, hence all global variables are implicitly set to volatile
 */ 
volatile bool new_timer = false;	// a new timer value is ready to be reported, in cont. counter mode
volatile bool new_adc = false;		// a new ADC value is ready to be reported, in cont. ADC mode
volatile bool overflow = false;		// the 555 timer stopped oscillating (TCB1 overflowed, see: TCB1)
bool cont_timer_display = false;	// is continuous reporting of timer on
bool cont_adc_display = false;		// is continuous reporting of ADC on


void clock_init(void);
void port_init(void);

void evsys_init(void);
void tca0_init(void);
void tcb0_init(void);
void tcb1_init(void);
void usart3_init(void);
void adc0_init(void);

void usart3_send_message (char *message);


int main(void)
{
	// char type is exactly the same as uint8_t, however usage of char in this case
	//  may be justified by the will to show our intent (working with characters)
	char character;
	
	// buffer to store the message we want to send
	char str_buffer[60];
	
	clock_init();
	port_init();
	
	evsys_init();
	tca0_init();
	tcb0_init();
	tcb1_init();
	usart3_init();
	adc0_init();

	// enable global interrupts
	sei();
	
	while (1)
	{
		// parse input
		//	- USART_RXCIF = USART receive complete interrupt flag
		//		- set when there are unread data in the receive buffer 
		//		- cleared automatically when the receive buffer is empty
		//	- intermittent polling
		//		- we are in a bigger loop, checking flag when it's "its turn"
		//		- allows other code to run (in main loop) as well (unlike dedicated polling)
		if (USART3.STATUS & USART_RXCIF_bm)
		{
			// this reading clears RXCIF
			// no need to use RXDATAH, because our messages are just a byte long (characters)
			character = USART3.RXDATAL;
			
			// this message should never appear
			sprintf(str_buffer, "No new message\n");
			
			switch (character)
			{
				// set direction of motor spin to forward
				case 'f':
				case 'F':
					PORTE.OUTSET = PIN2_bm;	// DIR 1
					sprintf(str_buffer, "Forward Motor\n");
					break;
				// set direction of motor spin to reverse 
				case 'R':
				case 'r':
					PORTE.OUTCLR = PIN2_bm;	// DIR 0
					sprintf(str_buffer, "Reverse Motor\n");
					break;
				// set motor speed- top value (CNT) is 6250, duty cycle is a percentage of this
				case '0':
					TCA0.SINGLE.CMP0 = 0;
					sprintf(str_buffer, "Speed: 0%%\n");
					break;
				case '1':	
					TCA0.SINGLE.CMP0 = 625;
					sprintf(str_buffer, "Speed: 10%%\n");
					break;
				case '2':
					TCA0.SINGLE.CMP0 = 1250;
					sprintf(str_buffer, "Speed: 20%%\n");
					break;
				case '3':
					TCA0.SINGLE.CMP0 = 1875;
					sprintf(str_buffer, "Speed: 30%%\n");
					break;
				case '4':
					TCA0.SINGLE.CMP0 = 2500;
					sprintf(str_buffer, "Speed: 40%%\n");
					break;
				case '5':	
				case '6':
				case '7':
				case '8':
				case '9':
					sprintf(str_buffer, "Speed: 50%%\n");
					TCA0.SINGLE.CMP0 = 3125;
					break;
				// report TCA0.SINGLE.CMP0, which controls PW => controls motor speed
				case 'S':
				case 's':
					sprintf(str_buffer, "TCA0.SINGLE.CMP0: %d\n", TCA0.SINGLE.CMP0);
					break;
				// report data about the 555 timer
				case 'T':
				case 't':
					if (overflow)
					{
						sprintf(str_buffer, "555 timer stopped oscillating!\n");
					}
					else
					{
						sprintf(str_buffer, "555 Period: %d us\n", period_555);
					}
					break;
				case 'L':
				case 'l':
					if (overflow)
					{
						sprintf(str_buffer, "555 timer stopped oscillating!\n");
					}
					else
					{
						sprintf(str_buffer, "555 Low Time: %d us\n", low_time_555);
					}
					break;
				case 'H':
				case 'h':
					if (overflow)
					{
						sprintf(str_buffer, "555 timer stopped oscillating!\n");
					}
					else
					{
						sprintf(str_buffer, "555 High Time: %d us\n", high_time_555);
					}
					break;
				// continuous report of TCB0 period
				case 'C':
				case 'c':
					sprintf(str_buffer, "Continuous Timer Display Start\n");
					cont_timer_display = true;
					break;
				case 'E':
				case 'e':
					sprintf(str_buffer, "Continuous Timer Display Stop\n");
					cont_timer_display = false;
					break;
				// report the ADC conversion (count) 
				case 'A':
				case 'a':
					sprintf(str_buffer, "ADC Count: %d\n", adc_reading);
					break;
				// report the ADC conversion (millivolts) 
				case 'V':
				case 'v':
					sprintf(str_buffer, "ADC Voltage: %ld mV\n", COUNT_TO_MILIVOLT(adc_reading));
					break;
				// continuous report of ADC conversion (millivolts)
				case 'M':
				case 'm':
					sprintf(str_buffer, "Continuous ADC Display Start\n");
					cont_adc_display = true;
					break;
				case 'N':
				case 'n':
					sprintf(str_buffer, "Continuous ADC Display Stop\n");
					cont_adc_display = false;
					break;
				default:
					sprintf(str_buffer, "Invalid Entry\n");
					break;
			}
			
			// sent response to parsed input
			usart3_send_message(str_buffer);
		}
		
		if (cont_timer_display && new_timer)
		{
			if (overflow)
			{
				sprintf(str_buffer, "555 timer stopped oscillating!\n");
			}
			else
			{
				sprintf(str_buffer, "Timer Period: %d us\n", period_555);
			}
			usart3_send_message(str_buffer);
			while (queue_counter != sent_counter);	// wait for message to be sent
			new_timer = false;						// signal that timer data was processed
		}
		else if (cont_adc_display && new_adc)
		{
			sprintf(str_buffer, "ADC Voltage: %ld mV\n", COUNT_TO_MILIVOLT(adc_reading));	
			usart3_send_message(str_buffer);
			while (queue_counter != sent_counter);	// wait for message to be sent
			new_adc = false;						// signal that ADC data was processed
		}
	}
}


// make clock run with 20 MHz (instead of it having a /6 prescaler)
void clock_init (void)
{
    ccp_write_io( (void *) &CLKCTRL.MCLKCTRLB , (0 << CLKCTRL_PEN_bp));
}


// initialize all ports (especially the outputs)
void port_init(void)
{
	PORTA.DIR = PIN0_bm | PIN1_bm;	// PIN0 LED2			PIN1 LED7
	PORTB.DIR = PIN4_bm;			// PIN4 TX output		PIN5 RX input
	PORTC.DIR = PIN6_bm;			// PIN6 LED4
	PORTE.DIR = PIN0_bm | PIN2_bm;	// PIN0 DC Motor dir	PIN2 PWM output
	PORTF.DIR = PIN4_bm;			// PIN4 LED6
}


/* TCA0 generates a signal for the motor through PWM
 *	- to set 100 Hz PWM frequency, we use formula 
 *			fPWM = fCLK / (prescaler * 2 * top_count)
 *		  100 Hz = 20 000 000 Hz / (16 * 2 * top_count)
 *	   top_count = 6,250
 */			
void tca0_init(void)
{
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTE_gc;									// select PORTE as the TCA0 output
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_DSBOTTOM_gc | TCA_SINGLE_CMP0EN_bm;	// update on bottom, compare on pin 0
	TCA0.SINGLE.PERBUF = 6250;													// calculated top value
	TCA0.SINGLE.CMP0 = 0;														// 0% of top value -> 0% duty cycle
}


/* TCB0 captures signal generated by 555 timer
 *	- calculates properties of 555 timer
 *	- uses a prescaler of 2, meaning 10 MHz => every tick is 0.1 us
 */
void tcb0_init(void)
{
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
	TCB0.CTRLB = TCB_CNTMODE_FRQPW_gc;			// clock frequency + pulse width measurement mode
	TCB0.EVCTRL = TCB_CAPTEI_bm;				// enable CAPT event interrupt
	TCB0.INTCTRL = TCB_CAPT_bm;					// enable CAPT interrupt 
}


/* TCB1 signals when 555 timer stops oscillating
 *	- one approach to achieve this was checking if low_time > 33, and it kind of worked
 *		- when 555 timer stops oscillating, last measurement showed weird low time
 *		- usually low time is constant (32-33) and only high time varies
 *		- while this worked, it feels like a hardware tweak, and sometimes gave false positives
 *	- the other, much clearer approach: at each interrupt, start a counter
 *		- frequency of 555 ranges from approximately 2 kHz to 13 kHz (actually, limits even narrower)
 *		- this means a period of 500 us - 76 us
 *		- this further means that at least every 500 us there should be a new value, a new interrupt
 *		- theoretically 5,000 ticks (500 us) is enough, but in practice it leads to false positives (fading light)
 *		- i settled with 10,000 but it could have been much higher as well, since for human eye it's still instantaneous
 *	- we start/restart TCB1 when new value is encountered
 *		- if it finishes counting 500 us before getting a new value, 555 timer is stuck
 *		- finishing counting means overflow, which means ISR will be executed
 */	
void tcb1_init(void)
{
	TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;	// f = 10 kHz	t = 0.1 us
	TCB1.CTRLB = TCB_CNTMODE_INT_gc;					// periodic interrupt mode
	TCB1.CCMP = 10000;									// overflow at 1001 us
	TCB1.INTCTRL = TCB_CAPT_bm;							// enable interrupt
}


// ADC is just a regular ADC, showing varying values based on first potentiometer
void adc0_init(void)
{
	ADC0.CTRLA = ADC_RESSEL_10BIT_gc | ADC_FREERUN_bm | ADC_ENABLE_bm;
	ADC0.CTRLB = ADC_SAMPNUM_ACC1_gc;	// no sample accumulation
	ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV128_gc;
	ADC0.CTRLD = ADC_INITDLY_DLY16_gc;
	ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
	ADC0.INTCTRL = ADC_RESRDY_bm;
	ADC0.COMMAND = ADC_STCONV_bm;		//begin the first ADC0 conversion
}


// ties together 555 timer and TCB0
void evsys_init (void)
{
	EVSYS.CHANNEL0 = EVSYS_GENERATOR_PORT1_PIN1_gc;	// set PORTB_PIN1 as event generator (555 timer)
	EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;		// set TCB0 as event user of 555 timer
}


void usart3_init(void)
{
	// default USART3 TxD and RxD are on PB0 and PB1 but Nano Every uses PB4 and PB5
	//  so PORTMUX is needed to select PB4 and PB5 for use with USART3
	PORTMUX.USARTROUTEA |= PORTMUX_USART3_ALT1_gc;
	
	USART3.CTRLA = USART_TXCIE_bm;																	// transmit complete interrupt enable (TXCIF set when byte sent)
	USART3.CTRLB = USART_RXEN_bm | USART_TXEN_bm;													// transmitter and receiver enabled
	USART3.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;	// asynchronous mode, no parity, 8-bit data
	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(115200);												// set baud rate
}


/* starts the sending process
 *	- loads the queue of characters (elements of message)
 *	- sends the first character, causing a transfer complete interrupt in the future
 *	- works hand-in-hand with ISR(USART3_TXC_vect)
 *	- does not append endline
 */
void usart3_send_message (char *message)
{
	queue_counter = 0;  // reset indices
	sent_counter = 1;	// set to one because first character already (/will be) sent (see below) 

	// put characters in queue
	while (*message)
	{
		queue[queue_counter++] = *message++;
	}
	
	// transmit data register low byte; sends first character => starts process
	USART3.TXDATAL = queue[0];  
}


ISR(ADC0_RESRDY_vect)
{
	adc_reading = ADC0.RES;
	
	// only report new value if there is no other value being processed
	if (!new_adc)
	{
		new_adc = true;
	}
	
	// manipulate LED 7
	if (adc_reading > VOLT_3DOT5)
	{
		PORTA.OUTSET = PIN1_bm;
	}
	else
	{
		PORTA.OUTCLR = PIN1_bm;
	}
}


// occurs when there is a new measurement/ pulse
ISR(TCB0_INT_vect)
{
	// clear interrupt flag
	TCB0.INTFLAGS = TCB_CAPT_bm;
	
	// new value means 555 is oscillating
	PORTF.OUTCLR = PIN4_bm;		// turn off LED 6 (overflow)
	overflow = false;			// clear possible previous overflow flag
	TCB1.CNT = 0;				// start/ restart TCB1
	
	// only report new value if there is no other value being processed
	if (!new_timer)
	{
		new_timer = true;
	}
	
	// input capture frequency and pulse-width measurement mode
	//	- TCB0.CNT = pulse width
	//	- TCB0.CCMP = high time
	// ---------------------
	//	- TCB0 has prescaler of 2 => f = 10 MHz = 10,000,000 Hz => t = 0.000,000,1 s = 0.1 us
	//	- this means that one tick equals 0.1 us, the TCB0.CNT and TCB0.CCMP values 
	//	- need to be divided by ten to give the time in microseconds (10 ticks = 1 us)
	// ---------------------
	//	- reading order of registers is important !
	period_555 = TCB0.CNT / 10;
	high_time_555 = TCB0.CCMP / 10;
	low_time_555 = period_555 - high_time_555;
	
	// manipulate LED 2
	if (period_555 > 150)
	{
		PORTA.OUTSET = PIN0_bm;
	}
	else
	{
		PORTA.OUTCLR = PIN0_bm;
	}
	
	// manipulate LED 4
	if (period_555 > 320)
	{
		PORTC.OUTSET = PIN6_bm;
	}
	else
	{
		PORTC.OUTCLR = PIN6_bm;
	}
}


// occurs when TCB1 overflows
ISR(TCB1_INT_vect)
{
 	TCB1.INTFLAGS = TCB_CAPT_bm;	// clear interrupt flag
	PORTF.OUTSET = PIN4_bm;			// set LED 6
	overflow = true;				// set overflow flag
	new_timer = true;				// need to report overflow in cont. mode as well
}


/* occurs when a character is sent
 *	- initially when first character is sent
 *	- then it "calls itself"
 *	- stops when we sent all characters
 */
ISR(USART3_TXC_vect)
{
	// TXCIF flag needs explicit clear
	USART3.STATUS |= USART_TXCIF_bm;

	// if we still have characters to send	
	if (queue_counter != sent_counter)
	{
		// can write to only when DREIF is set (i.e. TXDATAL and TXDATAH is empty)
		USART3.TXDATAL = queue[sent_counter++];
	}
}


