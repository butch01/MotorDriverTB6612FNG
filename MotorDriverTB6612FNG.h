/*
 * MotorDriverTB6612FNG.h
 *
 *  Created on: 27.03.2016
 *      Author: butch
 */

#ifndef MOTORDRIVERTB6612FNG_H_
#define MOTORDRIVERTB6612FNG_H_

class MotorDriverTB6612FNG {
public:
	MotorDriverTB6612FNG(int pinIn1, int pinIn2, int pinPWM, int pinStdby);
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



private:
	int _pinIn2;
	int _pinIn1;
	int _pinPWM;
	int _pinStdby; // is shared with other port on motor driver


};

#endif /* MOTORDRIVERTB6612FNG_H_ */
