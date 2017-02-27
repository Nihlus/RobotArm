// automatically generated by arduino-cmake
#line 1 "/home/jarl/Programming/RobotArm/RobotArm.ino"
/*
  RobotArm

   TODO
 */

#include <Arduino.h>
#include "AFMotor.h"
#include "ArmCommunication.hpp"

#line 13 "/home/jarl/Programming/RobotArm/cmake-build-debug/RobotArm_RobotArm.ino.cpp"
#include "Arduino.h"

//=== START Forward: /home/jarl/Programming/RobotArm/RobotArm.ino
 void setup() ;
 void setup() ;
 void checkButton(uint8_t pin, int &stateVariable, AF_DCMotor motor, uint8_t direction) ;
 void checkButton(uint8_t pin, int &stateVariable, AF_DCMotor motor, uint8_t direction) ;
 void loop() ;
 void loop() ;
//=== END Forward: /home/jarl/Programming/RobotArm/RobotArm.ino
#line 9 "/home/jarl/Programming/RobotArm/RobotArm.ino"


AF_DCMotor stick(1, MOTOR12_64KHZ);
AF_DCMotor boom(2, MOTOR12_64KHZ);
AF_DCMotor rotator(3, MOTOR34_64KHZ);
AF_DCMotor grip(4, MOTOR34_64KHZ);

const uint8_t pinStickForward = A1;
const uint8_t pinStickBackward = A0;
const uint8_t pinRotationLeft = A2;
const uint8_t pinRotationRight = A3;

void setup()
{
	boom.setSpeed(255);
	rotator.setSpeed(255);

	// See if manual control is enabled
		// If yes,
	Serial.begin(115200);

	pinMode(pinStickForward, INPUT_PULLUP);
	pinMode(pinStickBackward, INPUT_PULLUP);
	pinMode(pinRotationLeft, INPUT_PULLUP);
	pinMode(pinRotationRight, INPUT_PULLUP);

		// If no, wait for connection from controller
}

int joySFLastState = 0;
int joySBLastState = 0;
int joyRLLastState = 0;
int joyRRLastState = 0;

/// <summary>
/// Checks the state of the given button against a known state, and sets an associated
/// joystick button according to it.
/// </summary>
/// <param name="pin">The pin to check.</param>
/// <param name="stateVariable">The state variable for that pin.</param>
/// <param name="button">The associated joystick button.</param>
void checkButton(uint8_t pin, int &stateVariable, AF_DCMotor motor, uint8_t direction)
{
	int currentButtonState = !digitalRead(pin);
	if (currentButtonState != stateVariable)
	{
		if (currentButtonState == 1)
		{
			motor.run(direction);
		}
		else
		{
			motor.run(RELEASE);
		}

		stateVariable = currentButtonState;
	}
}

void loop()
{
	checkButton(pinStickForward, joySFLastState, boom, FORWARD);
	checkButton(pinStickBackward, joySBLastState, boom, BACKWARD);
	checkButton(pinRotationLeft, joyRLLastState, rotator, FORWARD);
	checkButton(pinRotationRight, joyRRLastState, rotator, BACKWARD);
	delay(50);
}
