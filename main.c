/*
 * @file	main.c
 * @author	<a href="https://github.com/AntaresLab">Sergey Starovoitov aka AntaresLab</a>
 * @version	0.3
 * @date	28-March-2022
 *
 * There is a firmware for the <a href="https://oshwlab.com/AntaresLab/sh72al1">alternative SH72 soldering iron controller</a>
 * with turbo mode, two-stage sleep mode, grounding the soldering iron tip to a "-" power wire and the protection against
 * the reverse polarity, short circuit and overvoltage. The firmware is designed for the ATtini13(A) MCU and works with the
 * factory-setted fuse bits.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdbool.h>

#define MAIN_CYCLE_FREQUENCY		100			///< Main cycle start frequency

#define SLEEP_MODE_THRESHOLD		130			///< Sleep mode delay, seconds
#define POWER_DOWN_MODE_THRESHOLD	180			///< Power down mode delay (from normal mode), seconds

#define OUTPUT_BLOCK_PIN			(1 << PB0)	///< Pseudo open-drain output blocking the iron power (active low)
#define VIBRATION_SENSOR_PIN		(1 << PB1)	///< Digital input for the movement sensor (triggers by the level change)
#define TURBO_BUTTON_PIN			(1 << PB2)	///< Digital input for the turbo button (active low)
#define HIGH_SIDE_REGULATOR_PIN		(1 << PB3)	///< Digital output for the high side regulator pin (high - normal, high - sleep mode)
#define LOW_SIDE_REGULATOR_PIN		(1 << PB4)	///< Digital output for the low side regulator pin (low - normal, high - turbo mode)

/// Digital pin state aliases
typedef enum{
	GPIO_LOW = 0,
	GPIO_HIGH
} GpioState;

/// Possible modes
typedef enum{
	NORMAL_MODE,				///< Normal mode
	TURBO_MODE,					///< Maximal temperature mode
	SLEEP_MODE,					///< Minimal temperature mode
	POWER_DOWN_MODE				///< Heater off mode
}Mode;

uint32_t sleepCounter = 0;		///< Time counter for sleep mode triggering, increases every 10ms

/*
 * Returns turbo button state
 * @return true if turbo button is activated (pressed)
 */
bool turboModeButtonIsActivated() {
	return (PINB & TURBO_BUTTON_PIN) == GPIO_LOW;
}

/*
 * Returns movement sensor state
 * @return true if movement sensor changed it state from the last function call
 */
bool vibrationSensorIsActivated() {
	static GpioState previousState = GPIO_HIGH;
	GpioState currentState = ((PINB & VIBRATION_SENSOR_PIN) == GPIO_LOW) ? GPIO_LOW : GPIO_HIGH;
	bool result = (previousState != currentState);
	previousState = currentState;
	return result;
}

/*
 * Sets all pins state to the normal mode and resets sleep counter
 */
void goToNormalMode(){
	PORTB |=  HIGH_SIDE_REGULATOR_PIN;
	PORTB &= ~LOW_SIDE_REGULATOR_PIN;
	sleepCounter = 0;
}

/*
 * Sets all pins state to the turbo mode and resets sleep counter
 */
void goToTurboMode(){
	PORTB |=  HIGH_SIDE_REGULATOR_PIN;
	PORTB |=  LOW_SIDE_REGULATOR_PIN;
	sleepCounter = 0;
}

/*
 * Sets all pins state to the sleep mode
 */
void goToSleepMode(){
	PORTB &= ~HIGH_SIDE_REGULATOR_PIN;
	PORTB &= ~LOW_SIDE_REGULATOR_PIN;
}

/*
 * Sets all pins state to the power down mode
 */
void goToPowerDownMode(){
	DDRB |= OUTPUT_BLOCK_PIN;
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

/*
 * Returns current mode based on the microcontroller pins state
 * @return current mode
 */
Mode getMode() {
	if ((PORTB & LOW_SIDE_REGULATOR_PIN) != GPIO_LOW) return TURBO_MODE;
	else if ((PORTB & HIGH_SIDE_REGULATOR_PIN) != GPIO_LOW) return NORMAL_MODE;
	else if ((DDRB & OUTPUT_BLOCK_PIN) != 0) return POWER_DOWN_MODE;
	else return SLEEP_MODE;
}

int main(void){

	// GPIO initialization (normal mode, pull-ups for vibraion sensor and turbo mode pins)
	DDRB = HIGH_SIDE_REGULATOR_PIN | LOW_SIDE_REGULATOR_PIN;
	PORTB = VIBRATION_SENSOR_PIN | TURBO_BUTTON_PIN | HIGH_SIDE_REGULATOR_PIN;
	DIDR0 = (1 << ADC0D);

	// System clock divider initialization: 9 600 000 Hz / 256 = 37 500 Hz
	CLKPR = (1 << CLKPCE);
	CLKPR = (1 << CLKPS3);

	// Analog comparator disabling for the power saving
	ACSR &= ~(1 << ACD);

	// Timer initialization: 37 500 Hz / 8 / (46 + 1) = 100 Hz
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01);
	TCNT0 = 0;
	OCR0A = 46;
	TIFR0 = 0;
	TIMSK0 = (1 << OCIE0A);

	// Sleep mode initialization
	set_sleep_mode(SLEEP_MODE_IDLE);

	// Interrupts global enabling
	sei();

	// The cycle starts every ~10ms by the timer interrupts, the rest of the time MCU is in
	// the sleep mode
	while(1){
		++sleepCounter;
		bool ironIsMoving = vibrationSensorIsActivated();

		switch(getMode()) {
		case NORMAL_MODE:
			if(turboModeButtonIsActivated()) goToTurboMode();
			else if(ironIsMoving) sleepCounter = 0;
			else if((sleepCounter / MAIN_CYCLE_FREQUENCY) == SLEEP_MODE_THRESHOLD) goToSleepMode();
			break;

		case TURBO_MODE:
			if(!turboModeButtonIsActivated()) goToNormalMode();
			else if(ironIsMoving) sleepCounter = 0;
			else if((sleepCounter / MAIN_CYCLE_FREQUENCY) == SLEEP_MODE_THRESHOLD) goToSleepMode();
			break;

		case SLEEP_MODE:
			if(turboModeButtonIsActivated()) goToTurboMode();
			else if(ironIsMoving) goToNormalMode();
			else if((sleepCounter / MAIN_CYCLE_FREQUENCY) == POWER_DOWN_MODE_THRESHOLD) goToPowerDownMode();
			break;

		default:
			break;
		}

		sleep_mode();
	}
	return 0;
}

/**
 * A trap for the handlerless interrupts
 * @note 	Since the only one interrupt for MCU wake up only is used in the project, a separate
 * 			timer interrupt handler was not implemented
 */
EMPTY_INTERRUPT(__vector_default)
