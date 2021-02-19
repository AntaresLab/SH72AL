/*
 * @file	main.c
 * @author	<a href="https://github.com/AntaresLab">Sergey Starovoitov aka AntaresLab</a>
 * @version	0.1
 * @date	19-February-2021
 *
 *  There is a firmware for the <a href="https://oshwlab.com/AntaresLab/SH72">alternative SH72 soldering iron controller</a>
 *  with turbo mode, two-stage sleep mode, grounding the soldering iron tip to a "-" power wire and the protection against
 *  the reverse polarity, short circuit and overvoltage. The firmware is designed for the ATtini13 MCU and works with the
 *  factory-setted fuse bits.
 *
 *  @note	Should be compiled with -O1 optimization level.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

/**
 * Set of the possible states
 */
typedef enum{
	NORMAL_MODE,	///< Normal mode
	TURBO_MODE,		///< Maximal temperature mode
	SLEEP_MODE,		///< Minimal temperature mode
	POWER_DOWN_MODE	///< Heater off mode
}Mode;

const uint8_t turboButtonAntiGlitchCounterThreshold = 10;	/// Turbo button anti-glitch delay, x10ms
const uint8_t positionSensorAntiGlitchCounterThreshold = 10;/// Position sensor anti-glitch delay, x10ms
const uint16_t sleepModeThreshold = 30000;					/// Sleep mode delay, x10ms
const uint16_t powerDownModeThreshold = 60000;				/// Power down mode delay, x10ms

Mode mode = NORMAL_MODE;					/// Current mode
uint8_t turboButtonAntiGlitchCounter = 0;	/// Turbo button anti-glitch program timer counter
uint8_t positionSensorState = 0;			/// Last position sensor state
uint8_t positionSensorAntiGlitchCounter = 0;/// Position sensor anti-glitch program timer counter
uint16_t sleepModeCounter = 0;				/// Sleep / power down mode program timer counter

/**
 * Switches the system to the normal mode: connects corresponding temperature control potentiometer
 * pins to the upper (hot) and lower (cold) sides, disables the output key control blocking and
 * resets the sleep mode counter
 */
inline void goToNormalMode(){
	PORTB |= (1 << PORTB4);
	PORTB &= ~((1 << PORTB2) | (1 << PORTB3));
	mode = NORMAL_MODE;
	sleepModeCounter = 0;
}

/**
 * Switches the system to the turbo mode: connects temperature control potentiometer pins to the
 * upper (hot) side, disables the output key control blocking and resets the sleep mode counter
 */
inline void goToTurboMode(){
	PORTB |= (1 << PORTB3) | (1 << PORTB4);
	PORTB &= ~(1 << PORTB2);
	mode = TURBO_MODE;
	sleepModeCounter = 0;
}

/**
 * Switches the system to the sleep mode: connects temperature control potentiometer pins to the
 * lower (cold) side and disables the output key control blocking
 */
inline void goToSleepMode(){
	PORTB &= ~((1 << PORTB2) | (1 << PORTB3) | (1 << PORTB4));
	mode = SLEEP_MODE;
}

/**
 * Switches the system to the power down mode: enables the output key control blocking
 */
inline void goToPowerDownMode(){
	PORTB |= (1 << PORTB2);
	mode = POWER_DOWN_MODE;
}

/**
 * Resets sleep mode counter and switches the system to the normal mode if it was in the sleep mode.
 * Has no effect if the system is in the power down mode.
 */
inline void resetSleepModeCounter(){
	if(mode != POWER_DOWN_MODE){
		sleepModeCounter = 0;
		if(mode == SLEEP_MODE){
			goToNormalMode();
		}
	}
}

/**
 * Gets the turbo button state
 * @retval 0 if button is released
 * @retval 1 if button is pressed
 */
inline uint8_t isTurboButtonPressed(){
	return ((PINB & (1 << PINB1)) == 0) ? 1 : 0;
}

/**
 * Gets the position sensor state
 * @retval 0 if position sensor is open
 * @retval 1 if position sensor is closed
 */
inline uint8_t isPositionSensorClosed(){
	return ((PINB & (1 << PINB0)) == 0) ? 1 : 0;
}

/**
 * Initializes MCU peripherals:
 * - GPIO:
 * 	- PB0 - position sensor input
 * 	- PB1 - turbo button input
 * 	- PB2 - output key block output
 * 	- PB3 - lower (cold) output of the temperature control potentiometer
 * 	- PB4 - upper (hot) output of the temperature control potentiometer
 * - System clock divider: 9 600 000 Hz / 256 = 37 500 Hz (for the power consumption reducing)
 * - Analog comparator: disabled (for the power consumption reducing)
 * - TIM0 timer: generates interrupts every ~10ms
 * - Sleep mode: Idle
 * - Interrupts: enabled
 */
inline void peripheralInitialization(){
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
	TCCR0B = (1 << CS01);
	TCNT0 = 0;
	OCR0A = 46;
	TIFR0 = 0;
	TIMSK0 = (1 << OCIE0A);
	// Sleep mode initialization
	set_sleep_mode(SLEEP_MODE_IDLE);
	// Interrupts global enabling
	sei();
}

/**
 * Main logic function: handles signals from the turbo button, position sensor and program timers
 */
int main(void){
	peripheralInitialization();
	while(1){
		// The cycle starts every ~10ms by the timer interrupts, the rest of the time MCU is in
		// the sleep mode
		sleep_mode();
		if(isTurboButtonPressed()){
			// Switches to the turbo mode if turbo mode button was pressed throughout the turbo
			// button anti-glitch cycle
			if(turboButtonAntiGlitchCounter < turboButtonAntiGlitchCounterThreshold){
				if((++turboButtonAntiGlitchCounter == turboButtonAntiGlitchCounterThreshold) &&
						(mode != TURBO_MODE)){
					goToTurboMode();
					continue;
				}
			}
		}else{
			// Switches to the normal mode if turbo mode button was released throughout the turbo
			// button anti-glitch cycle
			if(turboButtonAntiGlitchCounter > 0){
				if((--turboButtonAntiGlitchCounter == 0) && (mode == TURBO_MODE)){
					goToNormalMode();
					continue;
				}
			}
		}
		// Resets the sleep mode counter if position sensor changed its state and maintained it
		// throughout the position sensor anti-glitch cycle and current mode is not a power down
		if(isPositionSensorClosed()){
			if(positionSensorAntiGlitchCounter < positionSensorAntiGlitchCounterThreshold){
				if((++positionSensorAntiGlitchCounter == positionSensorAntiGlitchCounterThreshold) &&
						(positionSensorState == 0)){
					positionSensorState = 1;
					resetSleepModeCounter();
					continue;
				}
			}
		}else{
			if(positionSensorAntiGlitchCounter > 0){
				if((--positionSensorAntiGlitchCounter == 0) && (positionSensorState == 1)){
					positionSensorState = 0;
					resetSleepModeCounter();
					continue;
				}
			}
		}
		// If there were no changes in the turbo button and position sensor states, increases the
		// sleep mode counter and switches to the appropriate mode if the corresponding threshold
		// has been reached
		if(sleepModeCounter < powerDownModeThreshold){
			++sleepModeCounter;
			if(sleepModeCounter == sleepModeThreshold){
				goToSleepMode();
			}else if(sleepModeCounter == powerDownModeThreshold){
				goToPowerDownMode();
			}
		}
	}
	return 0;
}

/**
 * Trap for the handlerless interrupts
 * @note 	Since the only one interrupt for MCU wake up only is used in the project, a separate
 * 			timer interrupt handler was not implemented
 */
EMPTY_INTERRUPT(__vector_default)
