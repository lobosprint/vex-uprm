#pragma config(Sensor, in2,    armPot,         sensorPotentiometer)
#pragma config(Sensor, in3,    gripperPot,     sensorPotentiometer)
#pragma config(Motor,  port1,           gripperRight,  tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port6,           armTopLeft,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           armBottomLeft, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           armTopRight,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           armBottomRight, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          gripperLeft,   tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Request 1 for open the gripper and 0 for close the gripper
void gripperAction(int action, int gripperPotInit)
{
	motor[gripperLeft] = 0;
	motor[gripperRight] = 0;
	int gripperPotVal = SensorValue[gripperPot];
	bool move = true;
	if (action == 1)
	{
		//Si la pinza aun no esta abierta del todo o la pinza no puede moverse entonces sigue abriendose
		while ((gripperPotVal - gripperPotInit) < 1000)
		{
			//writeDebugStreamLine("Gripper = %d", (gripperPotVal - gripperPotInit));
			motor[gripperLeft] = 127;
			motor[gripperRight] = 127;
			delay(40);
			gripperPotVal = SensorValue[gripperPot];
		}

		motor[gripperLeft] = 0;
		motor[gripperRight] = 0;
	}
	else if (action == 0)
	{
		//writeDebugStreamLine("Valor del while tiene que ser > 1: %d", (gripperPotVal - gripperPotInit));
		while ((gripperPotVal - gripperPotInit) > 1 && move)
		{
			//writeDebugStreamLine("Valor %d > Valor %d", gripperPotVal, SensorValue[gripperPot] - 50);
			//Si la pinza no puede abrirse m�s entonces se para el movimiento, se aumenta el umbral en 10 que es la oscilaci�n aproximada que tiene el sensar
			if (gripperPotVal > SensorValue[gripperPot] + 50) {
				move = false;
			}
			else
			{
				//writeDebugStreamLine("Gripper  = %d", (gripperPotVal - gripperPotInit));
				gripperPotVal = SensorValue[gripperPot];
				motor[gripperLeft] = -127;
				motor[gripperRight] = -127;
			}
			delay(50);
		}

		motor[gripperLeft] = -30;
		motor[gripperRight] = -30;
	}
}

//Up the arm, open the gripper and down the arm
void armThrow(int armPotInit, int gripperPotInit)
{
	int armPotVal = SensorValue[armPot];
	bool throw = true;
	//up the arm
	while ((armPotVal - armPotInit) < 2200)
	{
		writeDebugStreamLine("Condici�n %d > 2000", (armPotVal - armPotInit));
		motor[armTopLeft] = 127;
		motor[armTopRight] = 127;
		motor[armBottomLeft] = 127;
		motor[armBottomRight] = 127;
		delay(50);
		armPotVal = SensorValue[armPot];

		if((armPotVal - armPotInit) > 1700 && throw)
		{
			//Open gipper
			gripperAction(1,gripperPotInit);
			throw = false;
		}
	}

	//Down the arm
	while ((armPotVal - armPotInit) > 200)
	{
		writeDebugStreamLine("Condici�n %d < 200", (armPotVal - armPotInit));
		motor[armTopLeft] = -127;
		motor[armTopRight] = -127;
		motor[armBottomLeft] = -127;
		motor[armBottomRight] = -127;
		delay(50);
		armPotVal = SensorValue[armPot];
	}

	motor[armTopLeft] = 0;
	motor[armTopRight] = 0;
	motor[armBottomLeft] = 0;
	motor[armBottomRight] = 0;
}


task main()
{
	int gripperPotVal = 0, armPotVal = 0, gripperPotInit = 0, armPotInit = 0;
	bool init = true;
	while (1)
	{
		if (init)
		{
			clearDebugStream();
			gripperPotInit = SensorValue[gripperPot];
			armPotInit = SensorValue[armPot];
			init = false;
		}

		gripperAction(1,gripperPotInit);
		delay(2000);
		gripperAction(0,gripperPotInit);
		//armThrow(armPotInit,gripperPotInit);
		//delay(2000);
		//gripperAction(0,gripperPotInit);
		//armThrow(armPotInit,gripperPotInit);


		delay(100000);
	}
}
