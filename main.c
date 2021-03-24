/*
 * @file	main.c
 * @author	<a href="https://github.com/AntaresLab">Sergey Starovoitov aka AntaresLab</a>
 * @version	0.2
 * @date	22-February-2021
 *
 * There is a firmware for the <a href="https://oshwlab.com/AntaresLab/SH72">alternative SH72 soldering iron controller</a>
 * with turbo mode, two-stage sleep mode, grounding the soldering iron tip to a "-" power wire and the protection against
 * the reverse polarity, short circuit and overvoltage. The firmware is designed for the ATtini13 MCU and works with the
 * factory-setted fuse bits.
 *
 * @note		Should be compiled with -O1 optimization level.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdbool.h>

#define SLEEP_MODE_THRESHOLD		300		///< Sleep mode delay, seconds.
#define POWER_DOWN_MODE_THRESHOLD	600		///< Power down mode delay, seconds.

static uint16_t sleepModeCounter = 0;		///< Sleep / power down mode program timer counter.

/**
 * Turns on normal mode: connects corresponding temperature control potentiometer pins to the
 * upper (hot) and lower (cold) sides and disables the output key control blocking. In this mode,
 * the temperature setpoint is adjusted with potentiometer.
 */
static inline void goToNormalMode(){
	PORTB |= (1 << PORTB4);
	PORTB &= ~((1 << PORTB2) | (1 << PORTB3));
}

/**
 * Turns on turbo mode: connects temperature control potentiometer pins to the upper (hot) side
 * and disables the output key control blocking. In this mode the maximum temperature setpoint is
 * applied, the potentiometer position has no effect.
 */
static inline void goToTurboMode(){
	PORTB |= (1 << PORTB3) | (1 << PORTB4);
	PORTB &= ~(1 << PORTB2);
}

/**
 * Turns on sleep mode: connects temperature control potentiometer pins to the lower (cold) side
 * and disables the output key control blocking. In this mode the minimum temperature setpoint is
 * applied, the potentiometer position has no effect.
 */
static inline void goToSleepMode(){
	PORTB &= ~((1 << PORTB2) | (1 << PORTB3) | (1 << PORTB4));
}

/**
 * Turns on power down mode: enables the output key control blocking. In this mode heater power is
 * switched off.
 */
static inline void goToPowerDownMode(){
	PORTB |= (1 << PORTB2);
}

/**
 * Turbo button state handler.
 * @retval false if turbo button released.
 * @retval true if turbo button pressed.
 */
static inline bool turboButtonIsActive(){
	return (PINB & (1 << PINB1)) == 0;
}

/**
 * Program entry point. MCU initialization and infinite loop with sleep mode. All logic is
 * implemented in the interrupt handlers.
 *
 * Preipherial initialization:
 * - GPIO:
 * 	- PB0 - vibration sensor input (PCINT0 logic level change interrupt, pull-up enabled);
 * 	- PB1 - turbo button input (INT0 logic level change interrupt, pull-up enabled);
 * 	- PB2 - output key block output;
 * 	- PB3 - lower (cold) output of the temperature control potentiometer;
 * 	- PB4 - upper (hot) output of the temperature control potentiometer;
 * - System clock divider: 9 600 000 Hz / 256 = 37 500 Hz (for the power consumption reducing);
 * - Analog comparator: disabled (for the power consumption reducing);
 * - TIM0 timer: generates interrupts every ~1s;
 * - Sleep mode: Idle;
 * - Interrupts: enabled.
 */
int main(void){
	// GPIO initialization
	DDRB &= ~((1 << DDB0) | (1 << DDB1));
	DDRB |= (1 << DDB2) | (1 << DDB3) | (1 << DDB4);
	PORTB &= ~((1 << PORTB2) | (1 << PORTB3));
	PORTB |= (1 << PORTB0) | (1 << PORTB1) | (1 << PORTB4);
	// System clock divider initialization
	CLKPR = (1 << CLKPCE);
	CLKPR = (1 << CLKPS3);
	// Analog comparator disabling
	ACSR &= ~(1 << ACD);
	// Timer initialization
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS02);
	TCNT0 = 0;
	OCR0A = 145;
	TIFR0 = 0;
	TIMSK0 = (1 << OCIE0A);
	// INT0 and PCINT0 external interrupts initialization
	MCUCR |= (1 << ISC00);
	PCMSK |= (1 << PCINT0);
	GIFR = 0;
	GIMSK |= (1 << INT0) | (1 << PCIE);
	// Sleep mode initialization
	set_sleep_mode(SLEEP_MODE_IDLE);
	// Interrupts global enabling
	sei();
	while(1){
		sleep_mode();
	}
	return 0;
}

/**
 * @brief TIM0 timer interrupt handler. Calls every ~1 second.
 *
 * Increments sleep / power down mode program timer counter. If counter reaches sleep mode delay
 * value turns on sleep mode. If If counter reaches power down mode delay value turns on power
 * down mode and stops counting.
 */
ISR(TIM0_COMPA_vect){
	if(sleepModeCounter < POWER_DOWN_MODE_THRESHOLD){
		++sleepModeCounter;
		if(sleepModeCounter == SLEEP_MODE_THRESHOLD){
			goToSleepMode();
		}else if(sleepModeCounter == POWER_DOWN_MODE_THRESHOLD){
			goToPowerDownMode();
		}
	}
}

/**
 * @brief INT0 external interrupt handler (turbo button).
 *
 * Turns on turbo mode if turbo button pressed. Turns on normal mode if turbo button released.
 * Resets sleep / power down mode program timer counter in both cases.
 */
ISR(INT0_vect){
	if(turboButtonIsActive()){
		goToTurboMode();
	}else{
		goToNormalMode();
	}
	sleepModeCounter = 0;
}

/**
 * @brief PCINT0 external interrupt handler (vibration sensor).
 *
 * Resets sleep / power down mode program timer counter if vibration (moving the soldering iron)
 * was detected and power down mode was not turned on. Turns on normal mode if sleep mode was
 * turned on.
 */
ISR(PCINT0_vect){
	if(sleepModeCounter < POWER_DOWN_MODE_THRESHOLD){
		if(sleepModeCounter >= SLEEP_MODE_THRESHOLD){
			goToNormalMode();
		}
		sleepModeCounter = 0;
	}
}

/**
 * A trap for the handlerless interrupts
 */
EMPTY_INTERRUPT(__vector_default)
