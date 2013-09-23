/*
 * CodeLock.c
 *
 * Created: 21/09/2013 10:15:22
 *  Author: seblovett
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#ifdef DEBUG
#include "../../../../Programming/Library/C/USART/USART.h"
#include <stdio.h>
#endif

#include <util/delay.h>
#include <avr/sleep.h>
#define KEYPAD_R_PIN	PINC
#define KEYPAD_R_PORT	PORTC
#define KEYPAD_R_DDR	DDRC
#define KEYPAD_R_PINTERRUPT	PCIE1
#define KEYPAD_R_0		(1 << PC0)
#define KEYPAD_R_1		(1 << PC1)
#define KEYPAD_R_2		(1 << PC2)
#define KEYPAD_R_3		(1 << PC3)
#define KEYPAD_R_Msk	((KEYPAD_R_0) | (KEYPAD_R_1) | (KEYPAD_R_2) | (KEYPAD_R_3))

#define KEYPAD_C_PIN	PIND
#define KEYPAD_C_PORT	PORTD
#define KEYPAD_C_DDR	DDRD
#define KEYPAD_C_0		(1 << PD5)
#define KEYPAD_C_1		(1 << PD6)
#define KEYPAD_C_2		(1 << PD7)

#define KEYPAD_C_Msk	((KEYPAD_C_0) | (KEYPAD_C_1) | (KEYPAD_C_2))

#define KEYPAD_KEY_1	(1 << 0)
#define KEYPAD_KEY_2	(1 << 1)
#define KEYPAD_KEY_3	(1 << 2)
#define KEYPAD_KEY_4	(1 << 3)
#define KEYPAD_KEY_5	(1 << 4)
#define KEYPAD_KEY_6	(1 << 5)
#define KEYPAD_KEY_7	(1 << 6)
#define KEYPAD_KEY_8	(1 << 7)
#define KEYPAD_KEY_9	(1 << 8)
#define KEYPAD_KEY_STAR	(1 << 9)
#define KEYPAD_KEY_0	(1 << 10)
#define KEYPAD_KEY_HASH	(1 << 11)
#define HASH			12
uint16_t KeyPad_Scan()
{
	uint16_t buttonState = 0x00;
	uint8_t input = 0;
	//set c0 low
	KEYPAD_C_PORT &= ~KEYPAD_C_0;
	_delay_us(1);
	//read input
	input = KEYPAD_R_PIN & KEYPAD_R_Msk; //get the input state
	if (!(input & KEYPAD_R_0))
	{
		buttonState |= KEYPAD_KEY_1;
	}
	if (!(input & KEYPAD_R_1))
	{
		buttonState |= KEYPAD_KEY_4;
	}
	if (!(input & KEYPAD_R_2))
	{
		buttonState |= KEYPAD_KEY_7;
	}
	if (!(input & KEYPAD_R_3))
	{
		buttonState |= KEYPAD_KEY_STAR;
	}
	//set c0 high
	KEYPAD_C_PORT |= KEYPAD_C_0;
	//set c1 low
	KEYPAD_C_PORT &= ~(KEYPAD_C_1);
	_delay_us(1);
	//read input
	input = (KEYPAD_R_PIN & KEYPAD_R_Msk); //get the input state
	if (!(input & KEYPAD_R_0))
	{
		buttonState |= KEYPAD_KEY_2;
	}
	if (!(input & KEYPAD_R_1))
	{
		buttonState |= KEYPAD_KEY_5;
	}
	if (!(input & KEYPAD_R_2))
	{
		buttonState |= KEYPAD_KEY_8;
	}
	if (!(input & KEYPAD_R_3))
	{
		buttonState |= KEYPAD_KEY_0;
	}
	//set c1 high
	KEYPAD_C_PORT |= (KEYPAD_C_1);
	
	//set c2 low
	KEYPAD_C_PORT &= ~(KEYPAD_C_2);
	_delay_us(1);
	//read input
	input = (KEYPAD_R_PIN & KEYPAD_R_Msk); //get the input state
	if (!(input & KEYPAD_R_0))
	{
		buttonState |= KEYPAD_KEY_3;
	}
	if (!(input & KEYPAD_R_1))
	{
		buttonState |= KEYPAD_KEY_6;
	}
	if (!(input & KEYPAD_R_2))
	{
		buttonState |= KEYPAD_KEY_9;
	}
	if (!(input & KEYPAD_R_3))
	{
		buttonState |= KEYPAD_KEY_HASH;
	}
	//set c2 high
	KEYPAD_C_PORT |= (KEYPAD_C_2);
	return buttonState;
}
#if KEYPAD_R_PINTERRUPT == PCIE1
ISR(PCINT1_vect)
#elif KEYPAD_R_PINTERRUPT == PCIE2
ISR(PCINT2_vect)
#elif KEYPAD_R_PINTERRUPT == PCIE0
ISR(PCINT0_vect)
#else
#error Pin Interrupt not supported
#endif
{
// 	uint16_t button,temp=0;
// 	button = 0;
// 	temp = KeyPad_Scan();
// 	for(uint8_t i = 0; i < 5 ; i++)
// 	{
// 		button = KeyPad_Scan();
// // 		if (button != temp)
// // 			continue;
// 		temp = button;
// 	}
}
void KeyPad_Init()
{
	//set columns to output
	KEYPAD_C_DDR |= KEYPAD_C_Msk;
	//set all columns high
	KEYPAD_C_PORT &= ~KEYPAD_C_Msk;
	//set rows to inputs
	KEYPAD_R_DDR &= ~(KEYPAD_R_Msk);
	//pull up rows
	KEYPAD_R_PORT |= KEYPAD_R_Msk;
	
	//initialise PCINT on the rows
	PCICR |= (1 << KEYPAD_R_PINTERRUPT);
#if KEYPAD_R_PINTERRUPT == PCIE1
	PCMSK1 |= KEYPAD_R_Msk;
#elif KEYPAD_R_PINTERRUPT == PCIE2
	PCMSK2 |= KEYPAD_R_Msk;
#elif KEYPAD_R_PINTERRUPT == PCIE0
	PCMSK0 |= KEYPAD_R_Msk;
#else
	#error Pin Interrupt not supported
#endif
	sei();
}

#define CODESIZE 4
const uint8_t CODE[CODESIZE] = {6,4,2,9};

#define STATE_ERR		-1
#define STATE_LOCKED	0
#define STATE_UNLOCK	1
#define LED_GREEN		PD2
#define LED_RED			PD3
#define DELAY			2000
extern int8_t GlobalState;

void PWM_Init()
{
	DDRB |= (1 << PB1);
	TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << CS10);
	OCR1A = 0;
	//OCR1AL = 255;
	
}

#define PWM_ON (0 << CS12) | (1 << CS11) | (1 << CS10)
#define PWM_OFF (0 << CS12) | (0 << CS11) | (0 << CS10)
#define PWM_Msk 0xF8

#define PWM_LOCK	0x010
#define PWM_UNLOCK	0x005F
void ChangeState(int8_t state)
{
	switch(state)
	{
		case STATE_UNLOCK:
			//enable TC1
			TCCR1B &= PWM_Msk;
			TCCR1B |= (PWM_ON);
			OCR1A = PWM_UNLOCK;
			PORTD |= (1 << LED_GREEN);
			PORTD &= ~(1 << LED_RED);
			_delay_ms(DELAY);
			TCCR1B &= PWM_Msk;
			TCCR1B |= (PWM_OFF); //diable PWM to save power
			break;
			
		case STATE_LOCKED:
			//enable TC1
			TCCR1B &= PWM_Msk;
			TCCR1B |= (PWM_ON);
			
			OCR1A = PWM_LOCK;
			PORTD &= ~(1 << LED_GREEN);
			PORTD |= (1 << LED_RED);
			_delay_ms(DELAY);
			TCCR1B &= PWM_Msk;
			TCCR1B |= (PWM_OFF);//diable PWM to save power
			break;
			
		case STATE_ERR:
		default:
			PORTD |= (1 << LED_RED);
			PORTD |= (1 << LED_GREEN);
			break;
	}
}

int main(void)
{
	uint16_t button, temp16 = 0;
	uint8_t temp, state = 0;
	
	uint8_t location = 0;
#ifdef DEBUG
	Usart_Init(F_CPU);//8MHz clock
#endif
	KeyPad_Init();
	PWM_Init();
	
	DDRD |= (1<< LED_GREEN) | (1 << LED_RED);
	
#ifdef DEBUG
	printf("\x0C\rCode Locker\n\r%02x", MCUSR);
	MCUSR = 0;
#endif
	PRR = (1 << PRTWI) | (1 << PRTIM2) | (0 << PRTIM1) | (1 << PRTIM0) | (1 << PRSPI) | (1 << PRADC)
#ifdef NDEBUG //if not building debug, disable UART
| (1 << PRUSART0)
#endif
; //shut down all peripherals - timer one used for PWM of servo.
	SMCR = (1 << SM2) | (1 << SM1) | (1 << SE); //enable deep sleep mode
	
	//initialise to LOCKED state 
	state = STATE_LOCKED;
	ChangeState(state);
    while(1)
    {
        //TODO:: Please write your application code 
		KEYPAD_C_PORT &= ~KEYPAD_C_Msk;
		PORTD &= ~(1 << LED_RED);
		PORTD &= ~(1 << LED_GREEN);
		_delay_ms(10);
		
		sleep_cpu();
		button = 0;
		temp16 = KeyPad_Scan();
		for(uint8_t i = 0; i < 5 ; i++)
		{
			button = KeyPad_Scan();
			if (button != temp16)
				break;
			temp16 = button;
		}
#ifdef DEBUG
		printf("\n\r%04x", button);
#endif
		if (button == 0) //a release of button / not got a reliable button.
		{
			continue;
		}
		
		//decode the button and check that it's only one key pressed at once.
		switch(button)
		{
			case KEYPAD_KEY_1:
				temp = 1;
				break;
			case KEYPAD_KEY_2:
				temp = 2;
				break;
			case KEYPAD_KEY_3:
				temp = 3;
				break;
			case KEYPAD_KEY_4:
				temp = 4;
				break;
			case KEYPAD_KEY_5:
				temp = 5;
				break;
			case KEYPAD_KEY_6:
				temp = 6;
				break;
			case KEYPAD_KEY_7:
				temp = 7;
				break;
			case KEYPAD_KEY_8:
				temp = 8;
				break;
			case KEYPAD_KEY_9:
				temp = 9;
				break;
			case KEYPAD_KEY_0:
				temp = 0;
				break;
			case KEYPAD_KEY_HASH:
				temp = HASH;
				location = 0;
				break;
			default:
				location = 0;
				temp = -1;
		}
		
		//if it is a valid key. 
		if (0 <= temp)
		{
			if(temp == CODE[location])	
			{
				location ++;
			}
			else if (temp == CODE[0])
			{
				location = 1;
			}
			else if (HASH == temp)
			{
				if(STATE_UNLOCK == state)//unlocked and pressed hash key
				{
					state = STATE_LOCKED;
#ifdef DEBUG
					printf("\n\rHASH Locked");
#endif
					ChangeState(state);
					
				}
			}
			else 
			{
				location = 0;
			}
		}
		
		
		
		//change state if need be
		if (CODESIZE == location)
		{
#ifdef DEBUG
			printf("\n\rSUCCESS!");
#endif
			location = 0;
			if (STATE_UNLOCK == state)
			{
				state = STATE_LOCKED;
#ifdef DEBUG
				printf("\n\rLocked");
#endif
				ChangeState(state);
				
			}
			else if (STATE_LOCKED == state)
			{
				state = STATE_UNLOCK;
#ifdef DEBUG
				printf("\n\rUnlocked");
#endif		
				ChangeState(state);
			}
			else
			{
				;
			}
		}
		PORTD |= (1 << LED_RED);
		_delay_ms(100);
		PORTD &= ~(1 << LED_RED);
    }
}