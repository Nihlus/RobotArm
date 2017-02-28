//
//  RobotArm.ino
//
//  Author:
//       Jarl Gullberg <jarl.gullberg@gmail.com>
//
//  Copyright (c) 2017 Jarl Gullberg
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <Arduino.h>
#include "AFMotor.h"

AF_DCMotor stick(1, MOTOR12_64KHZ);
AF_DCMotor boom(2, MOTOR12_64KHZ);
AF_DCMotor rotator(3, MOTOR34_64KHZ);
AF_DCMotor grip(4, MOTOR34_64KHZ);

const uint8_t pinStickForward = A1;
const uint8_t pinStickBackward = A0;
const uint8_t pinRotationLeft = A2;
const uint8_t pinRotationRight = A3;
const uint8_t pinManualControl = A5;

boolean isManualControl = false;

/*
 * Command protocol enumerations & definitions
 */

/// <summary>
/// Accepted commands which can be sent by controlling software.
/// </summary>
enum class Command : uint8_t
{
	/// <summary>
	/// Move a designated motor in the forward direction.
	/// </summary>
	MoveForward     = 0xF0,

	/// <summary>
	/// Move a designated motor in the backward direction.
	/// </summary>
	MoveBackward    = 0xBA,

	/// <summary>
	/// Stop a designated motor from moving.
	/// </summary>
	Stop            = 0xAB,

	/// <summary>
	/// Checks for or establishes a connection with controlling software.
	/// </summary>
	Connect         = 0xC0
};

/// <summary>
/// Available motors for control.
/// </summary>
enum class Motor : uint8_t
{
	/// <summary>
	/// No motor.
	/// </summary>
	None            = 0,

	/// <summary>
	/// The gripper motor, which is used to grab and hold onto things.
	/// </summary>
	Grip            = 1,

	/// <summary>
	/// The stick motor, which raises the arm.
	/// </summary>
	Stick           = 2,

	/// <summary>
	/// The boom motor, which moves the arm back and forth.
	/// </summary>
	Boom            = 3,

	/// <summary>
	/// The rotator motor, which rotates the arm around.
	/// </summary>
	Rotator         = 4
};

/// <summary>
/// Possible responses which the Arduino can send back to controlling software.
/// </summary>
enum class Status : uint8_t
{
	/// <summary>
	/// Some type of error has occurred.
	/// </summary>
	ErrorOccurred    = 0xE0,

	/// <summary>
	/// Confirms that the last command was executed without problems.
	/// </summary>
	AllClear         = 0xAC
};

/// <summary>
/// Possible errors which the Arduino can report.
/// </summary>
enum class Error : uint8_t
{
	/// <summary>
	/// No error.
	/// </summary>
	None = 0x00,

	/// <summary>
	/// An endstop has been reached.
	/// </summary>
	EndstopReached  = 0xEE,
};

/// <summary>
/// An incoming motion command.
/// </summary>
struct MotionCommand
{
	/// <summary>
	/// The binary signature of the structure.
	/// </summary>
	const uint8_t Signature = 0xFB;

	/// <summary>
	/// The sent command.
	/// </summary>
	Command command;

	/// <summary>
	/// The motor which the command affects.
	/// </summary>
	Motor motor;
};

/// <summary>
/// A response which goes back to the controlling software.
/// </summary>
struct CommandResponse
{
	/// <summary>
	/// The binary signature of the structure.
	/// </summary>
	const uint8_t Signature = 0xFC;

	/// <summary>
	/// The status of the response.
	/// </summary>
	Status status;

	/// <summary>
	/// The error (if any) that occurred)
	/// </summary>
	Error error;

	/// <summary>
	/// The command which this response relates to.
	/// </summary>
	Command command;

	/// <summary>
	/// The motor which this response relates to.
	/// </summary>
	Motor motor;
};

struct ConnectionQuery
{
	/// <summary>
	/// The binary signature of the structure.
	/// </summary>
	const uint8_t Signature = 0xFA;

	const Command connected = Command::Connect;
};

template <typename T> void sendPacket(T* content)
{
	Serial.flush();

	Serial.write(0xFF);
	Serial.write((int)sizeof(T));
	Serial.write((char*)&*content, sizeof(T));
}

void setup()
{
	grip.setSpeed(255);
	stick.setSpeed(255);
	boom.setSpeed(255);
	rotator.setSpeed(255);

	Serial.begin(115200);

	// See if manual control is enabled
	pinMode(pinManualControl, INPUT_PULLUP);
	if (!digitalRead(pinManualControl))
	{
		isManualControl = true;

		// Manual control is enabled, treat pins as joystick input
		pinMode(pinStickForward, INPUT_PULLUP);
		pinMode(pinStickBackward, INPUT_PULLUP);
		pinMode(pinRotationLeft, INPUT_PULLUP);
		pinMode(pinRotationRight, INPUT_PULLUP);
	}
	else
	{
		// Controlling software is expected. Wait for a connection.
		uint8_t serialSignature = 0;
		//while (serialSignature != (uint8_t)Command::Connect)
		while (serialSignature != 'c')
		{
			if (Serial.available() > 0)
			{
				int headByte = Serial.read();
				if (headByte < 0)
				{
					continue;
				}

				serialSignature = (uint8_t)headByte;
			}
		}

		CommandResponse* connectionResponse = new CommandResponse();
		connectionResponse->status = Status::AllClear;
		connectionResponse->error = Error::None;
		connectionResponse->command = Command::Connect;
		connectionResponse->motor = Motor::None;

		sendPacket(connectionResponse);
		delete connectionResponse;
	}
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
	if (isManualControl)
	{
		checkButton(pinStickForward, joySFLastState, boom, FORWARD);
		checkButton(pinStickBackward, joySBLastState, boom, BACKWARD);
		checkButton(pinRotationLeft, joyRLLastState, rotator, FORWARD);
		checkButton(pinRotationRight, joyRRLastState, rotator, BACKWARD);
		delay(50);
	}
	else
	{
		//if (isControllerStillConnected())
		{
			/*
			packet = readPacket();
			switch (packet_>signature)
			{
				case MotionCommand::Signature:
			    {
			        executeMotion((MotionCommand)packet);
		        }
		        case
			}
			 */
		}
	}
}
