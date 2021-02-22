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

#define TURBO_BUTTON_ANTI_GLITCH_COUNTER_THRESHOLD		10		///< Turbo button anti-glitch counter threshold, x10ms
#define POSITION_SENSOR_ANTI_GLITCH_COUNTER_THRESHOLD	10		///< Position sensor anti-glitch counter threshold, x10ms
#define SLEEP_MODE_THRESHOLD							30000	///< Sleep mode counter threshold, x10ms
#define POWER_DOWN_MODE_THRESHOLD						60000	///< Power down mode counter threshold, x10ms

/**
 * Set of the possible states
 */
typedef enum{
	NORMAL_MODE,				///< Normal mode
	TURBO_MODE,					///< Maximal temperature mode
	SLEEP_MODE,					///< Minimal temperature mode
	POWER_DOWN_MODE				///< Heater off mode
}Mode;

/**
 * Set of the possible turbo button states
 */
typedef enum{
	TURBO_BUTTON_RELEASED,		///< Turbo button released (state)
	TURBO_BUTTON_JUST_PRESSED,	///< Turbo button just pressed (event)
	TURBO_BUTTON_PRESSED,		///< Turbo button pressed (state)
	TURBO_BUTTON_JUST_RELEASED	///< Turbo button just released (event)
}TurboButtonState;

Mode mode = NORMAL_MODE;		///< Current mode
uint16_t sleepModeCounter = 0;	///< Sleep / power down mode program timer counter

/**
 * Switches the system to the normal mode: connects corresponding temperature control potentiometer
 * pins to the upper (hot) and lower (cold) sides, disables the output key control blocking and
 * resets the sleep mode counter
 */
static inline void goToNormalMode(){
	PORTB |= (1 << PORTB4);
	PORTB &= ~((1 << PORTB2) | (1 << PORTB3));
	mode = NORMAL_MODE;
	sleepModeCounter = 0;
}

/**
 * Switches the system to the turbo mode: connects temperature control potentiometer pins to the
 * upper (hot) side, disables the output key control blocking and resets the sleep mode counter
 */
static inline void goToTurboMode(){
	PORTB |= (1 << PORTB3) | (1 << PORTB4);
	PORTB &= ~(1 << PORTB2);
	mode = TURBO_MODE;
	sleepModeCounter = 0;
}

/**
 * Switches the system to the sleep mode: connects temperature control potentiometer pins to the
 * lower (cold) side and disables the output key control blocking
 */
static inline void goToSleepMode(){
	PORTB &= ~((1 << PORTB2) | (1 << PORTB3) | (1 << PORTB4));
	mode = SLEEP_MODE;
}

/**
 * Switches the system to the power down mode: enables the output key control blocking
 */
static inline void goToPowerDownMode(){
	PORTB |= (1 << PORTB2);
	mode = POWER_DOWN_MODE;
}

/**
 * Returns turbo button state
 * @return	Turbo button state
 * @note	The reaction to the turbo button state change does not occur immediately due to the
 * 			anti-glitch algorithm based on an incremental / decremental software counter that
 * 			checks the instantaneous button state every 10 ms. The threshold value of the counter
 * 			can be set by the TURBO_BUTTON_ANTI_GLITCH_COUNTER_THRESHOLD constant (x10ms).
 */
static inline TurboButtonState getTurboButtonState(){
	static TurboButtonState previousState = TURBO_BUTTON_RELEASED;	// Consider that button is released after the power up
	static uint8_t antiGlitchCounter = 0;
	if((PINB & (1 << PINB1)) == 0){					// Button is pressed at moment
		if(antiGlitchCounter < TURBO_BUTTON_ANTI_GLITCH_COUNTER_THRESHOLD){
			if((++antiGlitchCounter == TURBO_BUTTON_ANTI_GLITCH_COUNTER_THRESHOLD) &&
					(previousState == TURBO_BUTTON_RELEASED)){
				previousState = TURBO_BUTTON_PRESSED;
				return TURBO_BUTTON_JUST_PRESSED;	// Button pressing detected
			}
		}
	}else{											// Button is released at moment
		if(antiGlitchCounter > 0){
			if((--antiGlitchCounter == 0) && (previousState == TURBO_BUTTON_PRESSED)){
				previousState = TURBO_BUTTON_RELEASED;
				return TURBO_BUTTON_JUST_RELEASED;	// Button releasing detected
			}
		}
	}
	return previousState;
}

/**
 * Returns position sensor change status
 * @retval	0 if the sensor has not detected a position change
 * @retval	1 if the sensor has detected a position change
 * @note	The reaction to the sensor state change does not occur immediately due to the
 * 			anti-glitch algorithm based on an incremental / decremental software counter that
 * 			checks the instantaneous sensor state every 10 ms. The threshold value of the counter
 * 			can be set by the POSITION_SENSOR_ANTI_GLITCH_COUNTER_THRESHOLD constant (x10ms).
 */
static inline uint8_t getPositionSensorChangeStatus(){
	static uint8_t previousState = 1;	// Consider that sensor contact is open after the power up
	static uint8_t antiGlitchCounter = 0;
	if((PINB & (1 << PINB0)) == 0){		// Sensor contact is closed at moment
		if(antiGlitchCounter < POSITION_SENSOR_ANTI_GLITCH_COUNTER_THRESHOLD){
			if((++antiGlitchCounter == POSITION_SENSOR_ANTI_GLITCH_COUNTER_THRESHOLD) &&
					(previousState != 0)){
				previousState = 0;
				return 1;				// Sensor contact closing detected
			}
		}
	}else{								// Sensor contact is open at moment
		if(antiGlitchCounter > 0){
			if((--antiGlitchCounter == 0) && (previousState == 0)){
				previousState = 1;
				return 1;				// Sensor contact opening detected
			}
		}
	}
	return 0;							// No sensor contact state change detected
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
static inline void peripheralInitialization(){
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
		switch(getTurboButtonState()){
		case TURBO_BUTTON_JUST_PRESSED:
			goToTurboMode();				// If turbo button just was pressed - go to turbo mode
			break;
		case TURBO_BUTTON_JUST_RELEASED:
			goToNormalMode();				// If turbo button just was released - go to normal mode
			break;
		default:
			if(getPositionSensorChangeStatus()){
				if(mode != POWER_DOWN_MODE){// If position sensor has detected a position change reset
					sleepModeCounter = 0;	// sleep mode counter (except the power down mode)
					if(mode == SLEEP_MODE){
						goToNormalMode();
					}
				}
			}else{							// Handle the sleep mode counter
				if(sleepModeCounter < POWER_DOWN_MODE_THRESHOLD){
					if(++sleepModeCounter == SLEEP_MODE_THRESHOLD){
						goToSleepMode();
					}else if(sleepModeCounter == POWER_DOWN_MODE_THRESHOLD){
						goToPowerDownMode();
					}
				}
			}
			break;
		}
	}
	return 0;
}

/**
 * A trap for the handlerless interrupts
 * @note 	Since the only one interrupt for MCU wake up only is used in the project, a separate
 * 			timer interrupt handler was not implemented
 */
EMPTY_INTERRUPT(__vector_default)
