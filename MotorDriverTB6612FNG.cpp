/*
 * MotorDriverTB6612FNG.cpp
 *
 *  Created on: 27.03.2016
 *      Author: butch
 */

#include "MotorDriverTB6612FNG.h"

//#include "arduino.h"



#if IS_DEBUG_TB6612 == 1
	#include "ArduinoLog.h"
	// define logging by function here
	#define IS_DEBUG_TB6612_MOVE_PWM_ONE_WAY 0
	#define IS_DEBUG_TB6612_MOVE 0
	#define IS_DEBUG_TB6612_STANDBY_DISABLE 0
	#define IS_DEBUG_TB6612_STANDBY_ENABLE 0
	#define IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE 1
	#define IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY 1


#endif


#define SPEED_DEAD_ZONE 8

/**
 * constructor
 * - pinIn1
 * - pinIn2
 * - pinPWM
 * - pinStdby (shared by both ports on controller. Set pin to -1 if not wired.
 */
MotorDriverTB6612FNG::MotorDriverTB6612FNG(int pinIn1, int pinIn2, int pinPWM, int pinStdby)
{
	_pinIn1=pinIn1;
	_pinIn2=pinIn2;
	_pinPWM=pinPWM;
	_pinStdby=pinStdby;

	pinMode(_pinIn1, OUTPUT);
	pinMode(_pinIn2, OUTPUT);
	pinMode(_pinPWM, OUTPUT);
	pinMode(_pinStdby, OUTPUT);

//	// initialize trigger values for breaking
//	myShortBrakeTrigger 	= MOTOR_DEFAULT_TRIGGER_SHORT_BREAK;
//	myBreakeTrigger 		= MOTOR_DEFAULT_TRIGGER_BREAK;
//	myBackwardsDirection 	= DIRECTION_CW;
//	isBreakModeConfigured 	= false;
//	myIsBackwardsDrivingLightsOn 		= false;
//	myIsBreakingLightsOn 				= false;
//
//	myDriveModePrevious=0;
//	myDriveModeReverseLockTimerStartTime=0;
//	//myNeutralDebounceTime	= DRIVE_MODE_NEUTRAL_DEBOUNCE_TIME_MS;
//	myDriveModeReverseLockTime	= 500;

	setDefaultValues();

	#if IS_DEBUG_TB6612 == 1
		className="MotorDriverTB6612FNG";
	#endif
}



MotorDriverTB6612FNG::MotorDriverTB6612FNG()
{
//	myShortBrakeTrigger 	= MOTOR_DEFAULT_TRIGGER_SHORT_BREAK;
//	myBreakeTrigger 		= MOTOR_DEFAULT_TRIGGER_BREAK;
//	myBackwardsDirection 	= DIRECTION_CW;
//
//	isBreakModeConfigured 	= false;
//	myIsBackwardsDrivingLightsOn		= false;
//	myIsBreakingLightsOn				= false;
	setDefaultValues();

}


/**
 * set default values called in the different constructors
 */
void MotorDriverTB6612FNG::setDefaultValues()
{
	// initialize trigger values for breaking
	myShortBrakeTrigger 	= MOTOR_DEFAULT_TRIGGER_SHORT_BREAK;
	myBreakeTrigger 		= MOTOR_DEFAULT_TRIGGER_BREAK;
	myBackwardsDirection 	= DIRECTION_CW;
	isBreakModeConfigured 	= false;
	myIsBackwardsDrivingLightsOn 		= false;
	myIsBreakingLightsOn 				= false;

	myDriveModePrevious=0;
	myDriveModeReverseLockTimerStartTime=0;
	//myDriveModeReverseLockTime	= DRIVE_MODE_NEUTRAL_DEBOUNCE_TIME_MS;
	myDriveModeReverseLockTime = DRIVE_MODE_NEUTRAL_REVERSE_LOCK_TIME;

	//myDriveModeReverseLockTimerStarted=true;
	myDriveModeReverseLockTimerStartTime=DRIVE_MODE_NEUTRAL_REVERSE_LOCK_TIMER_NOT_STARTED;
	myDriveModeNeutralStartTime=DRIVE_MODE_NEUTRAL_TIMER_NOT_SET;

}

MotorDriverTB6612FNG::~MotorDriverTB6612FNG() {
	// TODO Auto-generated destructor stub
}

/**
 * If we are driving in forward mode and then we go to reverse. This function configures what to do then:
 * enableBreaking:
 * 		false -> enter reverse mode
 * 		true -> enable breake mode.
 * 		To switch from brake to reverse we need to set throttle to neutral position.
 * 		softBrakeTrigger -> value above this limit triggers breaking (stop function)
 * 		hardBrakeTrgger -> value above this limit triggers hard brake (shortBrake function)
 */
void MotorDriverTB6612FNG::configureBreaks(bool enableBreaking, uint8_t softBrakeTrigger = MOTOR_DEFAULT_TRIGGER_BREAK, uint8_t hardbrakeTrigger=MOTOR_DEFAULT_TRIGGER_SHORT_BREAK, unsigned int neutralReverseLockTime=DRIVE_MODE_NEUTRAL_REVERSE_LOCK_TIME)
{
	if (enableBreaking)
	{
		isBreakModeConfigured 		= true;
		myBreakeTrigger				= softBrakeTrigger;
		myShortBrakeTrigger 		= hardbrakeTrigger;
		myDriveModeReverseLockTime	= neutralReverseLockTime;
	}
	else
	{
		isBreakModeConfigured = false;
	}
}



/**
 * moves a motor with direction and speed
 * motorChannel needs to be
 * - motorChannel: use a, A, 0 for port A and use b, B, or 1 for Port B
 * - direction:    use 0 for clockwise and 1 for counter-clockwise
 * - speed:		   0 for stop, 255 for full speed
 */
void MotorDriverTB6612FNG::move(int direction, int speed)
{
	#if IS_DEBUG_TB6612_MOVE == 1
		Log.notice(F("in MotorDriverTB6612FNG::move with parameters (direction = %d, speed = %d" CR), direction, speed);
//		Serial.print(direction);
//		Serial.print(", ");
//		Serial.print(speed);
//		Serial.print(")\n");
	#endif
	standbyDisable();

	#if IS_DEBUG_TB6612_MOVE == 1
		Log.verbose(F("  calling setDirection(%d)" CR), direction);
	#endif
	setDirection(direction);

	if (speed < SPEED_DEAD_ZONE)
	{
		speed = 0;
	}
	#if IS_DEBUG_TB6612_MOVE == 1
		Log.verbose(F("  calling analogWrite(%d,%d)" CR), _pinPWM, speed);
//		Serial.print("  calling analogWrite(");
//		Serial.print(_pinPWM);
//		Serial.print(", ");
//		Serial.print(speed);
//		Serial.print(")\n");
	#endif

	analogWrite(_pinPWM, speed);
	#if IS_DEBUG_TB6612_MOVE == 1
		Log.notice(F("end of MotorDriverTB6612FNG::move" CR));
	#endif
}


void MotorDriverTB6612FNG::setDirection(int direction)
{
	if(direction == DIRECTION_CW)
	{
		digitalWrite(_pinIn1, HIGH);
		digitalWrite(_pinIn2, LOW);
	}
	else
	{
		digitalWrite(_pinIn1, LOW);
		digitalWrite(_pinIn2, HIGH);
	}
}

/**
 * brakes the motor (should be faster than setting PWM to 0)
 */
void MotorDriverTB6612FNG::shortBreak()
{
	standbyDisable();
	digitalWrite(_pinIn1, HIGH);
	digitalWrite(_pinIn2, HIGH);
	digitalWrite(_pinPWM, HIGH);
	myIsBreakingLightsOn=true;
}

/**
 * high impedance stop for one motor
 */
void MotorDriverTB6612FNG::stop()
{
	standbyDisable();
	digitalWrite(_pinIn1, LOW);
	digitalWrite(_pinIn2, LOW);
	digitalWrite(_pinPWM, HIGH);
	myIsBreakingLightsOn=true;

}



/**
 * enables standby (high impedance)
 * Warning: using standby will stop _both_ motor ports on TB6612FNG H-Bridge immediately. You should trigger it from outside.
 *
 * does nothing if _pinStdby in constructer has been set to -1
 */
void MotorDriverTB6612FNG::standbyEnable()
{
	#if IS_DEBUG_TB6612_STANDBY_ENABLE == 1
		char *functionName="standbyEnable";
		Log.notice(F("%s::%s -()" CR), className, functionName);
	#endif
	if (_pinStdby != -1)
	{
		digitalWrite(_pinStdby, LOW);
		#if IS_DEBUG_TB6612_STANDBY_ENABLE == 1
			Log.trace(F("%s::%s - standby is enabled." CR), className, functionName);
		#endif
	}

	#if IS_DEBUG_TB6612_STANDBY_ENABLE == 1
		Log.notice(F("%s::%s - end" CR));
	#endif
}

/**
 * disbales standby (high impedance)
 * Warning: using standbyDisable will release standby for _both_ motor ports on TB6612FNG H-bridge immediately. You should trigger it from outside.
 *
 * does nothing if _pinStdby in constructer has been set to -1
 */
void MotorDriverTB6612FNG::standbyDisable()
{
	char *functionName =  "standbyDisable";
	#if IS_DEBUG_TB6612_STANDBY_DISABLE == 1
		Log.notice(F("%s::%s - ()" CR),className, functionName);
	#endif

	if (_pinStdby != -1)
	{
		digitalWrite(_pinStdby, HIGH); //disable standby
		#if IS_DEBUG_TB6612_STANDBY_DISABLE == 1
			Log.trace(F("%s::%s - standby is disabled." CR),className, functionName);
		#endif

	}

	#if IS_DEBUG_TB6612_STANDBY_DISABLE == 1
		Log.notice(F("%s::%s - end" CR), className, functionName);
	#endif

}

/**
 * is setting the speed.
 * direction is set. may be 0 or 1
 * is converting values to 0 - 255
 */
void MotorDriverTB6612FNG::movePWMOneWay(int speed, int direction, int minInput, int maxInput)
{
	#if IS_DEBUG_TB6612_MOVE_PWM_ONE_WAY == 1
		Log.notice(F("in MotorDriverTB6612FNG::movePWMOneWay(speed=%d, direction=%d, minInput=%d, maxInput=%d)" CR), speed, direction, minInput, maxInput);
		Log.trace(F("  calling move(%d, map(%d, %d, %d, %d, %d)" CR), direction, speed, minInput, maxInput, 0, 255);
	#endif

	move(direction, map(speed, minInput, maxInput, 0,255));

	#if IS_DEBUG_TB6612_MOVE_PWM_ONE_WAY == 1
		Log.notice(F("end of MotorDriverTB6612FNG::movePWMOneWay()" CR));
	#endif
}

/**
 * calculates the driving mode based on previousDriveMode (global var), speed and direction
 * use relative speed here
 */
uint8_t MotorDriverTB6612FNG::calculateDriveMode(uint8_t relativeSpeed, uint8_t direction)
{

	#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
		char functionName[] = "calculateDriveMode";
		Log.notice(F("%s::%s - (relativeSpeed= %d, direction = %d)" CR), className, functionName, relativeSpeed, direction);
	#endif



	#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
		Log.verbose(F("%s::%s - start= %d, myNeutralDebounceTime = %l" CR), className, functionName, myDriveModeReverseLockTimerStartTime, myDriveModeReverseLockTime);
	#endif


	int8_t currentDriveMode = -1;


	// check if we are driving forward
	if (direction != myBackwardsDirection  && relativeSpeed > DRIVE_MODE_NEUTRAL_ZONE)
	{
		// driving forward
		currentDriveMode = DRIVE_MODE_FORWARD;
		myIsBackwardsDrivingLightsOn = false;
		myIsBreakingLightsOn = false;
		myDriveModeNeutralStartTime = DRIVE_MODE_NEUTRAL_TIMER_NOT_SET;
		#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
			Log.verbose(F("forwards" CR));
		#endif

		//myDriveModeReverseLockTimerStarted=false;
		myDriveModeReverseLockTimerStartTime = DRIVE_MODE_NEUTRAL_REVERSE_LOCK_TIMER_NOT_STARTED;

	}
	else
	{
		// driving backwards

		// check if we are now in neutral mode
		//if (relativeSpeed <= DRIVE_MODE_NEUTRAL_ZONE)

		// check if we are in neutral (lower than break trigger)
		if (relativeSpeed <= myBreakeTrigger)
		{
			currentDriveMode = DRIVE_MODE_NEUTRAL;
			//Serial.println (myDriveModeNeutralStartTime);

			if (myDriveModeNeutralStartTime == DRIVE_MODE_NEUTRAL_TIMER_NOT_SET)
			{
				myDriveModeNeutralStartTime = millis();
			}


			#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
				Log.verbose(F("in neutral" CR));
			#endif
			// we are in neutral zone. now check If we have been previously there and how long. Compare with previous driveMode
			switch (myDriveModePrevious)
			{
				case DRIVE_MODE_NEUTRAL:
					// nothing to do, we are still in same drive mode.
					#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
						Log.verbose(F("in neutral, prev neutral" CR));
					#endif
					if (myDriveModeNeutralStartTime + DISABLE_BREAK_LIGHTS_AFTER_MS < millis())
					{
						myIsBreakingLightsOn = false;
					}
					break;

				case DRIVE_MODE_BRAKE:
					// we want to have the debounce time lock before we can go backwards. So we need to start the timer.
					// myDriveModeReverseLockTimerStarted = true;
					myDriveModeReverseLockTimerStartTime=millis();
					myIsBreakingLightsOn = true;
					#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
						Log.verbose(F("in neutral, prev break" CR));
					#endif
					break;

				case DRIVE_MODE_BACKWARDS:
					// we have been driving backwards before. We just released the throttle (for a short time).
					// So we want to go backwards without lock.
					// fake that timer is already started and fullfilled (setting to 1. (0 is reserved for not set).
					// myDriveModeReverseLockTimerStarted = true;
					myDriveModeReverseLockTimerStartTime = 1;

					// new
					myIsBreakingLightsOn = true;
					#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
						Log.verbose(F("in neutral, prev back" CR));
					#endif
					break;

				case DRIVE_MODE_FORWARD:
					// we have been driving forwards before. Now we are in neutral and need to start the reverse lock timer

					// new
					myIsBreakingLightsOn = true;
					//if (!myDriveModeReverseLockTimerStarted)
					if (myDriveModeReverseLockTimerStartTime == DRIVE_MODE_NEUTRAL_REVERSE_LOCK_TIMER_NOT_STARTED)
					{
						myDriveModeReverseLockTimerStartTime = millis();
						//myDriveModeReverseLockTimerStarted = true;
					}

					break;
			}


		}
		else
		{
			// break trigger is reached
			myDriveModeNeutralStartTime = DRIVE_MODE_NEUTRAL_TIMER_NOT_SET;

			//if (!myDriveModeReverseLockTimerStarted)
			if (myDriveModeReverseLockTimerStartTime == DRIVE_MODE_NEUTRAL_REVERSE_LOCK_TIMER_NOT_STARTED)
			{
				// reverse lock timer is not started yet. Start it now. Set brake mode
				myDriveModeReverseLockTimerStartTime = millis();
				//myDriveModeReverseLockTimerStarted = true;
				currentDriveMode = DRIVE_MODE_BRAKE;
				#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
					Log.verbose(F("breaking, starting timer" CR));
				#endif
			}
			else
			{
				// timer is already counting -> check if it is fulfilled
				unsigned long currentTime = millis();
				#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
					Log.verbose(F("myDriveModeReverseLockTimerStartTime=%d + myDriveModeReverseLockTime=%d = %d > %d-> %T" CR), myDriveModeReverseLockTimerStartTime, myDriveModeReverseLockTime, (myDriveModeReverseLockTimerStartTime+myDriveModeReverseLockTime),currentTime,(myDriveModeReverseLockTimerStartTime + myDriveModeReverseLockTime > currentTime) );
				#endif

				if (myDriveModeReverseLockTimerStartTime + myDriveModeReverseLockTime < currentTime)
				{
					// timer is run out. For backwards we need to go over neutral.
					if (myDriveModePrevious == DRIVE_MODE_NEUTRAL || myDriveModePrevious == DRIVE_MODE_BACKWARDS)
					{
						// we have been in neutral before -> go backwards
						currentDriveMode = DRIVE_MODE_BACKWARDS;

						// set backwards driving lights to on, breaking lights off
						myIsBackwardsDrivingLightsOn = true;
						myIsBreakingLightsOn = false;
						#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
							Log.verbose(F("backwards" CR));
						#endif
					}
					else if (myDriveModePrevious == DRIVE_MODE_BRAKE)
					{
						// still in lock time or still breaking-> we are in breaking mode
						currentDriveMode = DRIVE_MODE_BRAKE;

						// set braking lights to on
						myIsBreakingLightsOn = true;
						#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
							Log.verbose(F("break" CR));
						#endif
					}
				}
				else
				{
					// timer is not up
					currentDriveMode = DRIVE_MODE_BRAKE;

				}
			}

		}
	}
	//Serial.println(myDriveModeReverseLockTimerStartTime);
	#if IS_DEBUG_TB6612_CALCULATE_DRIVE_MODE == 1
		Log.notice(F("%s::%s - direction: %d, prev: %d, curr: %d" CR), className, functionName, direction, myDriveModePrevious, currentDriveMode);
	#endif

	//myDriveModeCurrent = currentDriveMode;
	return currentDriveMode;

}


/**
 * is setting the speed.
 * assumes that minInput >= 0 and maxInput >=0
 * is converting to two directions.
 * value < (minInput to maxInput-minInput) / 2 to direction 0 and speed to 0 - 255
 * value >= (minInput to maxInput-minInput) / 2 to direction 1 and speed to 0 - 255
 *
 * minInput is speed 255 on direction 0
 * middle between maxInput and minInput is center / neutral
 * maxInput is speed 255 on direction 1
 *
 */
void MotorDriverTB6612FNG::movePWMTwoWay(int speed, int minInput, int maxInput)
{
	char functionName[]="movePWMTwoWay";
	#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1

	Log.notice(F("%s::%s - (speed=%d, minInput=%d, maxInput=%d)" CR), className, functionName, speed, minInput, maxInput);
	#endif
	// value's interval: abs(maxInput - minInput)


	// valuediff: 	abs(speed - minInput)

	//int center =  (maxInput - minInput) /2 + minInput;
	int center =  ((abs(maxInput) + abs(minInput))/2)+minInput;
	int relativeSpeed = abs(speed - center);
	int direction;
	int halfInterval = abs(center-minInput);

	// fix overflow.
	if (relativeSpeed > halfInterval )
	{
		relativeSpeed = halfInterval;
	}




	// calculate the direction based on the center
	// center is calculated by the following: (maxInput - minInput) /2 + minInput
	if ( speed < center)
	{
		// speed has lower value than center -> use direction 0

		// as speed following is used:
		// move( 0, map(relativeSpeed, 0, center/2, 0, 255 ) );
		direction = DIRECTION_CW;
	}
	else
	{
		// speed has higher value than center -> use direction 1
		//move(1, map(relativeSpeed, 0, center/2, 0, 255 )) ;
		direction = DIRECTION_CCW;
	}
	#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
		Log.verbose(F("%s::%s - break configured= - %T, relSpeed=%d, breakTrigger=%d" CR), className, functionName, isBreakModeConfigured, relativeSpeed, myBreakeTrigger);
	#endif
	// here comes braking behavior, if configured
	if (isBreakModeConfigured)
	{

		// brake mode is enabled
		// check the current drive mode
		uint8_t driveMode = calculateDriveMode(relativeSpeed, direction);

		if (driveMode == DRIVE_MODE_BRAKE)
		{
			// set breaking intensivity, based on triggers
			if (relativeSpeed < myBreakeTrigger)
			{
				#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
					Log.verbose(F("%s::%s - #1 driveMode: float -> call move(%d, 0)" CR), className, functionName, direction);
				#endif
				// set speed to 0
				move(direction, 0) ;
			}
			else if (relativeSpeed >= myBreakeTrigger && relativeSpeed < myShortBrakeTrigger)
			{
				#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
					Log.verbose(F("%s::%s - #2 driveMode: call stop()" CR), className, functionName);
				#endif
				// set breaking
				stop();
			}
			else if (relativeSpeed >= myShortBrakeTrigger)
			{
				#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
					Log.verbose(F("%s::%s - #3 driveMode: hardBreak -> call shortBreak()" CR), className, functionName);
				#endif
				// break as fast as we can
				shortBreak();
			}
		}
		else
		{
			#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
				Log.verbose(F("%s::%s - #4 driveMode: normal drive -> move(%d, map(%d, %d, %d, %d, %d))" CR), className, functionName, direction, relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX);
			#endif
			// drive normally
			move(direction, map(relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX )) ;
		}

		// save current drive mode as old drive mode
		myDriveModePrevious = driveMode;
	}
	else
	{
		// no breaking configured, drive normal / directly
		#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
			Log.verbose(F("%s::%s - #5 driveMode: normal drive -> move(%d, map(%d, %d, %d, %d, %d))" CR), className, functionName, direction, relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX);
		#endif
		// drive normally
		move(direction, map(relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX )) ;
	}






		/*
		if (direction == myBackwardsDirection)
		{
			if (driveMode == DRIVE_MODE_BRAKE)
			{
				// set breaking intensivity, based on triggers
				if (relativeSpeed < myBreakeTrigger)
				{
					#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
						Log.verbose(F("%s::%s - #1 driveMode: float -> call move(%d, 0)" CR), className, functionName, direction);
					#endif
					// set speed to 0
					move(direction, 0) ;
				}
				else if (relativeSpeed >= myBreakeTrigger && relativeSpeed < myShortBrakeTrigger)
				{
					#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
						Log.verbose(F("%s::%s - #2 driveMode: call stop()" CR), className, functionName);
					#endif
					// set breaking
					stop();
				}
				else if (relativeSpeed >= myShortBrakeTrigger)
				{
					#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
						Log.verbose(F("%s::%s - #3 driveMode: hardBreak -> call shortBreak()" CR), className, functionName);
					#endif
					// break as fast as we can
					shortBreak();
				}
			}
			else
			{
				#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
					Log.verbose(F("%s::%s - #4 driveMode: normal drive -> move(%d, map(%d, %d, %d, %d, %d))" CR), className, functionName, direction, relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX);
				#endif
				// drive normally
				move(direction, map(relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX )) ;
			}
		}
		else
		{
			#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
				Log.verbose(F("%s::%s - #5 driveMode: normal drive -> move(%d, map(%d, %d, %d, %d, %d))" CR), className, functionName, direction, relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX);
			#endif
			// drive normally
			move(direction, map(relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX )) ;
		}

	#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
		Log.notice(F("%s::%s - driveMode: center=%d, relativeSpeed=%d, direction=%d, prevDriveMode=%d, currDriveMode=%d" CR), className, functionName, center, relativeSpeed, direction, myDriveModePrevious, driveMode);
	#endif

	// save current drive mode as old drive mode
	myDriveModePrevious = driveMode;
	}
	else
	{
		// brake mode not enabled
		move(direction, map(relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX )) ;

	}
*/
	// trigger the motor driver

/*
	// correct rounding issues -> limit relative speed to target limits of map function
	if (relativeSpeed > center)
	{
		relativeSpeed = center;
	}
	if (relativeSpeed < 0)
	{
		relativeSpeed = 0;
	}

	*/
//	#if IS_DEBUG_TB6612 == 1
//	Log.verbose(F("  calling: move(direction= %d ,map(relativeSpeed= %d ,0,center= %d"), direction, relativeSpeed, center,);
//	//Serial.print(direction);
//	Serial.print(",map(relativeSpeed=");
//	Serial.print(relativeSpeed);
//	Serial.print(",0,center=");
//	Serial.print(center);
//	Serial.print(",");
//	Serial.print(PWM_MIN);
//	Serial.print(",");
//	Serial.print(PWM_MAX);
//	Serial.println("))");
	//move(direction, map(relativeSpeed, 0, center, PWM_MIN, PWM_MAX )) ;
//	#endif
	//move(direction, map(relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX )) ;



	#if IS_DEBUG_TB6612_MOVE_PWM_TWO_WAY == 1
		Log.notice(F("%s::%s - end" CR), className, functionName);
	#endif
}


/**
 * sets the backwards direction
 */
void MotorDriverTB6612FNG::setBackwardsDirection(uint8_t backwardsDirection)
{
	myBackwardsDirection = backwardsDirection;
}

void MotorDriverTB6612FNG::debugShowConfig()
{
	Serial.print("MotorDriverTB6612FNG::debugShowConfig");
	Serial.print(" _pinIn1=");
	Serial.print(_pinIn1);
	Serial.print(" _pinIn2=");
	Serial.print(_pinIn2);
	Serial.print(" _pinPWM=");
	Serial.print(_pinPWM);
	Serial.print(" _pinStdby=");
	Serial.print(_pinStdby);
	Serial.print("\n");
}

/***
 * returns if breaking breaking lights are on
 */
bool MotorDriverTB6612FNG::isBreakingLightsOn()
{
	return myIsBreakingLightsOn;
}

/**
 * returns if backwardsDrivingLights are on
 */
bool MotorDriverTB6612FNG::isBackwardsDrivingLightOn()
{
	return myIsBackwardsDrivingLightsOn;
}


/**
 * returns previous drive mode
 */
int8_t MotorDriverTB6612FNG::getDriveModePrevious()
{
	return myDriveModePrevious;
}

