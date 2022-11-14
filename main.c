#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define F_CPU 20000000
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

/* Aiming for fPWM = 100Hz */
/* fPWM = fCLK_PER/(2*16*PER) = 20MHz/(2*16*6250) */
#define PWM_PERIOD_SETTING 6250

// 10-bit ADC => max count 1023
// VDD reference voltage => max voltage 5 V
//		5V   ... 1023
//		3.5v ... 716.1
#define VOLT_3DOT5 716

//		5000 mV   ... 1024
//			x mV  ... COUNT 
#define COUNT_TO_MILIVOLT(COUNT) ((uint32_t)COUNT * 5000 / 1023)


// queue and related information must be global since it is used
//  by both usart3_send_message and ISR(USART3_TXC_vect)
unsigned char queue[50];
unsigned char queue_counter = 0;
unsigned char sent_counter = 0;   

uint16_t period;
uint16_t high_time;
uint16_t low_time;

uint16_t adc_reading;	// ADC0.RES has 10-bits

//https://www.geeksforgeeks.org/understanding-volatile-qualifier-in-c/
volatile bool new_timer = false;
volatile bool new_adc = false;

void clock_init(void);
void port_init(void);

void evsys_init(void);
void tca0_init(void);
void tcb0_init(void);
void tcb1_init(void);
void usart3_init(void);
void adc0_init(void);

void usart3_send_message (char *message);


bool cont_timer_display = false;
bool cont_adc_display = false;


int main(void)
{
	// char type is appropriate in this case (over uint_8) because
	// tells us we are working with characters, and is of 1 byte as well ?
	char character;
	
	new_timer = 0;
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
		// USART_RXCIF = USART receive complete interrupt flag
		//	- set when there are unread data in the receive buffer 
		//  - cleared automatically when the receive buffer is empty
		// intermittent polling
		//	- we are in a bigger loop, checking flag when it's "its turn"
		//	- allows other code to run as well (unlike dedicated polling)
		if (USART3.STATUS & USART_RXCIF_bm)
		{
			// this reading clears RXCIF
			// no need to use RXDATAH, because our messages (characters) are just a byte long
			character = USART3.RXDATAL;
			sprintf(str_buffer, "No new message\n");	// this message should never appear
			switch (character)
			{
				case 'f':
				case 'F':
					/*Code to move the motor in the Forward Direction */
					sprintf(str_buffer, "Forward\n");
					break;
				case 'R':
				case 'r':
					sprintf(str_buffer, "Reverse\n");
					break;
				case '0':
					TCA0.SINGLE.CMP0 = 0;					// top value (CNT) is 6250, duty cycle is a percentage of this
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
				case 'S':
				case 's':
					sprintf(str_buffer, "TCA0.SINGLE.CMP0: %d\n", TCA0.SINGLE.CMP0);
					break;
				case 'T':
				case 't':
					sprintf(str_buffer, "555 period: %d\n", period);
					break;
				case 'L':
				case 'l':
					sprintf(str_buffer, "555 low time: %d\n", low_time);
					break;
				case 'H':
				case 'h':
					sprintf(str_buffer, "555 high time: %d\n", high_time);
					break;
				case 'C':
				case 'c':
					sprintf(str_buffer, "TCB0 cont starting.\n");
					cont_timer_display = true;
					break;
				case 'E':
				case 'e':
					sprintf(str_buffer, "TCB0 cont stopping.\n");
					cont_timer_display = false;
					break;
				case 'A':
				case 'a':
					sprintf(str_buffer, "ADC value: %d\n", adc_reading);
					break;
				case 'V':
				case 'v':
					sprintf(str_buffer, "ADC mV: %ld\n", COUNT_TO_MILIVOLT(adc_reading));
					break;
				case 'M':
				case 'm':
					sprintf(str_buffer, "ADC0 cont starting.\n");
					cont_adc_display = true;
					break;
				case 'N':
				case 'n':
					sprintf(str_buffer, "ADC0 cont stopping.\n");
					cont_adc_display = false;
					break;
				default:
					sprintf(str_buffer, "Invalid Entry\n");
					break;
			}
			usart3_send_message(str_buffer);
		}
		
		if (cont_timer_display && new_timer)
		{
			sprintf(str_buffer, "TCB0: %d.\n", period);	// or use var from ISR?
			usart3_send_message(str_buffer);
			while (queue_counter != sent_counter);
			new_timer = false;
		}
		else if (cont_adc_display && new_adc)
		{
			sprintf(str_buffer, "ADC mV: %ld\n", COUNT_TO_MILIVOLT(adc_reading));	
			usart3_send_message(str_buffer);
			while (queue_counter != sent_counter);
			new_adc = false;
		}
	}
}


void clock_init (void)
{
	/* Disable CLK_PER Prescaler */
    ccp_write_io( (void *) &CLKCTRL.MCLKCTRLB , (0 << CLKCTRL_PEN_bp));
    /* If set from the fuses during programming, the CPU will now run at 20MHz (default is /6) */   
}


void port_init(void)
{
	PORTA.DIR = PIN0_bm | PIN1_bm;	// PIN0 LED2			PIN1 LED7
	PORTB.DIR = PIN4_bm;			// PIN4 TX output		PIN5 RX input
	PORTC.DIR = PIN6_bm;			// PIN6 LED4
	PORTE.DIR = PIN0_bm | PIN2_bm;	// PIN0 DC Motor dir	PIN2 PWM output
	PORTF.DIR = PIN4_bm;			// PIN4 LED6
}

// frequency of PWM: fPWM = fCLK / (prescaler * 2 * top_count)
//				   100 Hz = 20 000 000 Hz / (16 * 2 * top_count)
//				top_count = 6,250
void tca0_init(void)
{
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTE_gc; // select PORTE as the TCA0 output
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;	// enable later ?
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_DSBOTTOM_gc | TCA_SINGLE_CMP0EN_bm;	// update on bottom, compare on pin 0
	TCA0.SINGLE.PERBUF = 6250;	// top value
	TCA0.SINGLE.CMP0 = 0;		// 0% of top value -> 0% duty cycle
}

void tcb0_init(void)
{
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;		//CLK_PER/2 and Enable TCB0
	TCB0.CTRLB = TCB_CNTMODE_FRQPW_gc;						//Clock Frequency - Pulse Width Measurement mode
	TCB0.EVCTRL = TCB_CAPTEI_bm;	// TCB0_EDGE ? Enable Event Input and Falling Event Edge
	TCB0.INTCTRL = TCB_CAPT_bm;								//Enable Capture interrupt 
}

void tcb1_init(void)
{
	TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;	//CLK_PER/2 and Enable TCB1 
	// challenge ?
}

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

// sets PORTB Pin 0 as the Event Generator and TCB0 as the Event User
// Each event user can be connected to one channel, and several users can be connected to the same channel.
void evsys_init (void)
{
	//  set PORTB_PIN1 as event generator
	EVSYS.CHANNEL0 = EVSYS_GENERATOR_PORT1_PIN1_gc;
	
	// set TCB0 as event user
	EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;
	//EVSYS.USERTCB1 = EVSYS_CHANNEL_CHANNEL0_gc;		/* Same channel - one for PW Measurement, one for timer overflow (to extend timer) ?? */
}

void usart3_init(void)
{
	//PORTB.DIR &= ~PIN5_bm;	// RX input 
	//PORTB.DIR |= PIN4_bm;	// TX output
	
	// default USART3 TxD and RxD are on PB0 and PB1 but Nano Every uses PB4 and PB5
	//  so PORTMUX is needed to select PB4 and PB5 for use with USART3
	PORTMUX.USARTROUTEA |= PORTMUX_USART3_ALT1_gc;
	
	USART3.CTRLA = USART_TXCIE_bm;																		// transmit complete interrupt enable (TXCIF set when byte sent)
	USART3.CTRLB = USART_RXEN_bm | USART_TXEN_bm;														// transmitter and receiver enabled
	USART3.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;		// asynchronous mode, no parity, 8-bit data?
	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(115200);													// set baud rate
}




/* Starts the sending process
 *	- loads the queue of characters (elements of message)
 *	- sends the first character, causing a transfer complete interrupt in the future
 *		- works hand-in-hand with ISR(USART3_TXC_vect)
 */
void usart3_send_message (char *message)
{
	queue_counter = 0;  // reset indices
	sent_counter = 1;	// set to one because first character already sent (see below) 
	
	//queue[qcntr++] = 0x0d;   /*put CRLF into the queue first*/
	//queue[qcntr++] = 0x0a;
	

	while (*message)
	{
		queue[queue_counter++] = *message++;   /*put characters into queue*/
	}
	
	// linux/windows ? 
	//	queue[qcntr++] = 0x0d;   /*put CRLF into the queue next*/ \r
	//	queue[qcntr++] = 0x0a;	/* Do not add these characters if they are in your strings */ \n
	
	// Transmit Data Register Low Byte
	//	- send first character to start process
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
	
	if (adc_reading > VOLT_3DOT5)
	{
		PORTA.OUTSET = PIN1_bm;
	}
	else
	{
		PORTA.OUTCLR = PIN1_bm;
	}
}


ISR(TCB0_INT_vect)
{
	TCB0.INTFLAGS = TCB_CAPT_bm; // where should int flag be cleared?
	
	//Input Capture Frequency and Pulse-Width Measurement Mode
	// cnt = pulse width
	// ccmp = high time
	// reading order is important
	uint16_t cnt = TCB0.CNT;
	uint16_t ccmp = TCB0.CCMP;
	
	// only report new value if there is no other value being processed
	if (!new_timer)
	{
		new_timer = true;
	}
	
	
	
	
	
	// 100 Hz (given) => 1 cnt = 0.01 s = 10 ms => need to divide by 10
	period = cnt / 10;
	high_time = ccmp / 10;
	low_time = period - high_time;
	
	// manipulate LED2
	if (period > 150)
	{
		PORTA.OUTSET = PIN0_bm;
	}
	else
	{
		PORTA.OUTCLR = PIN0_bm;
	}
	
	// manipulate LED4
	if (period > 320)
	{
		PORTC.OUTSET = PIN6_bm;
	}
	else
	{
		PORTC.OUTCLR = PIN6_bm;
	}
	
	// turn off LED6 ?
	PORTF.OUTCLR = PIN5_bm;
}

// TODO TCB1 ISR

/* Occurs when a character is sent
 *	- initially when first character is sent
 *	- then it "calls itself"
 *	- stops when we sent all characters
 *	- TXC interrupts only happen when a character has been transmitted.
 */
ISR(USART3_TXC_vect)
{
	// TXCIF flag needs explicit clear
	USART3.STATUS |= USART_TXCIF_bm;

	// if we still have characters to send	
	if (queue_counter != sent_counter)
	{
		// can write to only when DREIF is set
		// DREIF is set if TXDATAL and TXDATAH is empty
		USART3.TXDATAL = queue[sent_counter++];
	}
}


