/*
 * MotorDriverTB6612FNG.h
 *
 *  Created on: 27.03.2016
 *      Author: butch
 *
 *      version: 1.0
 *      version history:
 *      1.0 - added breaking
 *          - added query for check if driving backwa_rds or breaking
 *          - changed some variables from int to uint_8 to save memory
 */

#ifndef MOTORDRIVERTB6612FNG_H_
#define MOTORDRIVERTB6612FNG_H_

#define IS_DEBUG_TB6612 0

#include "arduino.h"


#define PWM_MIN 0
#define PWM_MAX 255


#define DIRECTION_CW 0
#define DIRECTION_CCW 1
#define DIRECTION_BACKWARDS 0

// Break triggers needs to fit relative speed, meaning range of 0 to 127
#define MOTOR_DEFAULT_TRIGGER_BREAK 60
#define MOTOR_DEFAULT_TRIGGER_SHORT_BREAK 120

#define DRIVE_MODE_NEUTRAL 0
#define DRIVE_MODE_FORWARD 1
#define DRIVE_MODE_BACKWARDS 2
#define DRIVE_MODE_BRAKE 3
#define DRIVE_MODE_NEUTRAL_START 4

#define DRIVE_MODE_NEUTRAL_ZONE 5 // in this zone if
#define DRIVE_MODE_NEUTRAL_REVERSE_LOCK_TIME 750
#define DRIVE_MODE_NEUTRAL_TIMER_NOT_SET 0	// to save a boolean we use 0 for not set, if set we assign millis()
#define DISABLE_BREAK_LIGHTS_AFTER_MS 1200 // sets how many milliseconds the breaking light should be on after entering neutral mode
#define DRIVE_MODE_NEUTRAL_REVERSE_LOCK_TIMER_NOT_STARTED 0 // to save a boolean. Indicates that the reverse lock Timer is not started



class MotorDriverTB6612FNG {
public:
	MotorDriverTB6612FNG(int pinIn1, int pinIn2, int pinPWM, int pinStdby);
	MotorDriverTB6612FNG();
	virtual ~MotorDriverTB6612FNG();

	void standbyEnable();
	void standbyDisable();
	void move(int direction, int speed);
	void shortBreak();
	void stop();
	void movePWMOneWay(int speed, int direction, int minInput, int maxInput);
	void movePWMTwoWay(int speed, int minInput, int maxInput);
	void setDirection(int direction);
	void debugShowConfig();
	void configureBreaks(bool enableBreaking, uint8_t softBrakeTrigger , uint8_t hardbrakeTrigger, unsigned int neutralReverseLockTime  ); // defines switches between break
	bool isBreakingLightsOn(); 			// returns if we are currently in brake mode (stop or shortBrake) -> will be used to trogger breaking lights
	bool isBackwardsDrivingLightOn(); 	// returns if we are currently driving backwards -> will be used to trigger backwards drving lights
	void setBackwardsDirection(uint8_t backwardsDirection); 	// sets the direction which is the break / reverse
	int8_t getDriveModePrevious(); // returns the previous drive mode


private:
	uint8_t calculateDriveMode(uint8_t speed, uint8_t direction);
	void setDefaultValues();

	uint8_t _pinIn2;
	uint8_t _pinIn1;
	uint8_t _pinPWM;
	uint8_t _pinStdby; // is shared with other port on motor driver
    int8_t myDriveModePrevious; // saves previous drive mode. Drive mode is one of neutral, forward, backwards, break (signed to be able to use -1 for unset.


    bool myIsBackwardsDrivingLightsOn; 		// can be used for backwards driving lights
    bool myIsBreakingLightsOn;				// can be used for breaking lights
    bool isBreakModeConfigured;


    uint8_t myBreakeTrigger;
    uint8_t myShortBrakeTrigger;
    uint8_t myBackwardsDirection;
    unsigned long myDriveModeReverseLockTimerStartTime;
    unsigned int myDriveModeReverseLockTime;
    //bool myDriveModeReverseLockTimerStarted;
    unsigned long myDriveModeNeutralStartTime; // time when the car got neutral


#if IS_DEBUG_TB6612 == 1
    char* className;
#endif

};

#endif /* MOTORDRIVERTB6612FNG_H_ */























