/*
 * MotorDriverTB6612FNG.cpp
 *
 *  Created on: 27.03.2016
 *      Author: butch
 */

#include "MotorDriverTB6612FNG.h"
#include "arduino.h"


#define IS_DEBUG_TB6612 0
#define PWM_MIN 0
#define PWM_MAX 255
#define CC 0
#define CCC 1

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
}

MotorDriverTB6612FNG::MotorDriverTB6612FNG()
{


}


MotorDriverTB6612FNG::~MotorDriverTB6612FNG() {
	// TODO Auto-generated destructor stub
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
	#if IS_DEBUG_TB6612
		Serial.print("in MotorDriverTB6612FNG::move with parameters (");
		Serial.print(direction);
		Serial.print(", ");
		Serial.print(speed);
		Serial.print(")\n");
	#endif
	standbyDisable();

	#if IS_DEBUG_TB6612
		Serial.println("  calling setDirection");
	#endif
	setDirection(direction);

	#if IS_DEBUG_TB6612
		Serial.print("  calling analogWrite(");
		Serial.print(_pinPWM);
		Serial.print(", ");
		Serial.print(speed);
		Serial.print(")\n");
	#endif
	analogWrite(_pinPWM, speed);
	#if IS_DEBUG_TB6612
		Serial.println("end of MotorDriverTB6612FNG::move");
	#endif
}


void MotorDriverTB6612FNG::setDirection(int direction)
{
	if(direction == 0)
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

}



/**
 * enables standby (high impedance)
 * Warning: using standby will stopp _both_ Motor Ports on TB6612FNG H-Bridge immediately. You should trigger it from outside.
 *
 * does nothing if _pinStdby in constructer has been set to -1
 */
void MotorDriverTB6612FNG::standbyEnable()
{
	if (_pinStdby != -1)
	{
		digitalWrite(_pinStdby, LOW);
	}
}

/**
 * disbales standby (high impedance)
 * Warning: using standbyDisable will release standby for _both_ motor ports on TB6612FNG H-bridge immediately. You should trigger it from outside.
 *
 * does nothing if _pinStdby in constructer has been set to -1
 */
void MotorDriverTB6612FNG::standbyDisable()
{
	#if IS_DEBUG_TB6612
		Serial.println("in MotorDriverTB6612FNG::standbyDisable()");
	#endif

	if (_pinStdby != -1)
	{
		digitalWrite(_pinStdby, HIGH); //disable standby
	#if IS_DEBUG_TB6612
		Serial.println("  standby is disabled.");
	#endif

	}

	#if IS_DEBUG_TB6612
		Serial.println("end of MotorDriverTB6612FNG::standbyDisable()");
	#endif

}

/**
 * is setting the speed.
 * direction is set. may be 0 or 1
 * is converting values to 0 - 255
 */
void MotorDriverTB6612FNG::movePWMOneWay(int speed, int direction, int minInput, int maxInput)
{
	move(direction, map(speed, minInput, maxInput, 0,255));
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
	#if IS_DEBUG_TB6612
		Serial.print("in MotorDriverTB6612FNG::movePWMTwoWay with parameters speed=");
		Serial.print(speed);
		Serial.print(", minInput=");
		Serial.print(minInput);
		Serial.print(", maxInput=");
		Serial.print(maxInput);
		Serial.print("\n");
	#endif
	// value's interval: abs(maxInput - minInput)


	// valuediff: 	abs(speed - minInput)

	//int center =  (maxInput - minInput) /2 + minInput;
	int center =  ((abs(maxInput) + abs(minInput))/2)+minInput;
	int relativeSpeed = abs(speed - center);
	int direction;
	int halfInterval = abs(center-minInput);

	// calculate the direction based on the center
	// center is calculated by the following: (maxInput - minInput) /2 + minInput
	if ( speed < center)
	{
		// speed has lower value than center -> use direction 0

		// as speed following is used:
		// move( 0, map(relativeSpeed, 0, center/2, 0, 255 ) );
		direction = 0;
	}
	else
	{
		// speed has higher value than center -> use direction 1
		//move(1, map(relativeSpeed, 0, center/2, 0, 255 )) ;
		direction = 1;
	}
	// trigger the motor driver
	#if IS_DEBUG_TB6612
		Serial.print("  center=");
		Serial.print(center);
		Serial.print(", relativeSpeed=");
		Serial.print(relativeSpeed);
		Serial.print(", direction=");
		Serial.print(direction);
		Serial.print("\n");
	#endif
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
	#if IS_DEBUG_TB6612
	Serial.print("  calling: move(direction=");
	Serial.print(direction);
	Serial.print(",map(relativeSpeed=");
	Serial.print(relativeSpeed);
	Serial.print(",0,center=");
	Serial.print(center);
	Serial.print(",");
	Serial.print(PWM_MIN);
	Serial.print(",");
	Serial.print(PWM_MAX);
	Serial.println("))");
	//move(direction, map(relativeSpeed, 0, center, PWM_MIN, PWM_MAX )) ;
	#endif
	move(direction, map(relativeSpeed, 0, halfInterval, PWM_MIN, PWM_MAX )) ;

	#if IS_DEBUG_TB6612
		Serial.print("end of MotorDriverTB6612FNG::movePWMTwoWay\n");
	#endif
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
