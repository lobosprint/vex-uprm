#ifndef AutoFunctionsMotorGripper.h
#define AutoFunctionsMotorGripper.h
#pragma systemFile
//*********************************************************************************************
//			Includes
//*********************************************************************************************

#include "../BNSLib.h";

//*********************************************************************************************
//			Gripper Functions
//*********************************************************************************************

//Action = 1 -> Open
//Action = 0 -> Close
//Action = 2 -> Init
//Range: 800 back of robot, 3700 closed
void gripperAction(int action)
{
	if(action==1)
	{
		while(SensorValue[gripperPot]>2000)
		{
			motor[gripper] = -127;
		}
		motor[gripper]=0;
	}
	else if(action==2)
	{
		while(SensorValue[gripperPot]<1300)
		{
			motor[gripper] = 127;
		}
		motor[gripper]=0;
	}
	else
	{
		clearTimer(T2);
		while(SensorValue[gripperPot]<3000 && time1(T2)<1000)
		{
			motor[gripper] = 127;
		}
		motor[gripper] = 50;
	}
}

int gripperPotInit = 0;
void moveGripperTo(int value)
{
	int curr = SensorValue[gripperPot]-gripperPotInit;
	if(	curr < value )
	{
		while(curr < value)
		{
			motor[gripper]=127;
			curr = SensorValue[gripperPot]-gripperPotInit;
			writeDebugStreamLine("%f<%f", curr, value);
		}
	}
	else
	{
		while(curr > value)
		{
			motor[gripper]=-127;
			curr = SensorValue[gripperPot]-gripperPotInit;
		}
	}
	motor[gripper]=0;
}


//*********************************************************************************************
//			Base Functions
//*********************************************************************************************

// This variables for the ecnoders and counter fore the PID.
int encoderL, encoderR, counter;

// Function to move the robot to the front or back.
void moveBase(int speed)
{
	motor[rbBase]=speed;
	motor[rfBase]=speed;
	motor[lbBase]=speed;
	motor[lfBase]=speed;
	motor[rmBase]=speed;
	motor[lmBase]=speed;
}

// Funtion to correct the aligment of the robot when move to the front or back.
void moveBase(int speed, int offset)
{
	motor[rbBase]=speed + offset;
	motor[rmBase]=speed + offset;
	motor[rfBase]=speed + offset;
	motor[lbBase]=speed - offset;
	motor[lfBase]=speed - offset;
	motor[lmBase]=speed - offset;
}

// Rotate the base clockwise or counter-clockwise.
void rotateBase(int speed)
{
	motor[rbBase]=speed;
	motor[rfBase]=speed;
	motor[rmBase]=speed;
	motor[lbBase]=-speed;
	motor[lfBase]=-speed;
	motor[lmBase]=-speed;
}

// Stop moving the robot.
void stopBase()
{
	motor[rbBase]=0;
	motor[rfBase]=0;
	motor[lbBase]=0;
	motor[lfBase]=0;
	motor[rmBase]=0;
	motor[lmBase]=0;
}

// Convert the distance of ticks (the value of the encoder) in inches.
float ticksToInches(int ticks)
{
	float inches = ticks*((2.75*PI)/360.0)/0.9;
	return inches;
}

float inchesToTicks(int inches)
{
	float ticks = inches*(360.0/(2.75*PI))*0.9;
	return ticks;
}

// Move the base to the front. Request the distance for know how much need to move,
// the variable time is the maximun time for wait to the movement to
// prevent keep going infinitely the task if the robot can�t move and
// finally, the variable factor to reduce the velocity of the robot.
// Note: The distance in inches.
void moveBaseWithFactor(int distance, int time, float factor){
	distance = inchesToTicks(distance);
	writeDebugStreamLine("Start moveBaseFront");
	writeDebugStreamLine("Target distance = %d", distance);
	int encoderAvg = 0;
	encoderR = 0;
	encoderL = 0;
	float initialGyro = SensorValue[gyro], actualGyro = initialGyro;
	writeDebugStreamLine("Initial Gyro Position = %f", initialGyro);
	int startEncoderValueR = SensorValue[encR];
	int startEncoderValueL = SensorValue[encL];
	bool atPos=false;
	float pidMovResult;
	//float pidStraightResult;
	counter = 0;

	PID pidMovement;
	//PID pidStraight;
	PIDInit(&pidMovement, 0.15, .1, 0.25); // Set P, I, and D constants
	//PIDInit(&pidStraight, 2, 0, 0.3);//Set constants for driving straight

	clearTimer(T1);
	int timer = T1;
	while(!atPos && timer < time){
		encoderR = (SensorValue[encR] - startEncoderValueR)*-1;//Reversing
		encoderL = SensorValue[encL] - startEncoderValueL;
		encoderAvg = (encoderR+encoderL)/2;
		writeDebugStreamLine("encR = %d\tencL = %d", encoderR, encoderL);
		pidMovResult = PIDCompute(&pidMovement, distance - encoderAvg);
		//pidStraightResult = PIDCompute(&pidStraight, initialGyro - actualGyro);
		//writeDebugStreamLine("Actual Gyro Position = %f   PID = %f", actualGyro, pidStraightResult);
		moveBase(pidMovResult*factor);
		if (abs(encoderAvg-distance)<2)
			counter++;
		if (counter >= 4)
			atPos = true;
		timer = time1[T1];
		wait1Msec(25);
		actualGyro = SensorValue[gyro];
	}
	moveBase(0);
	writeDebugStreamLine("Encoder at position = %d", encoderAvg);
}
//For going slow at first
// Move the base to the front. Request the distance for know how much need to move,
// the variable time is the maximun time for wait to the movement to
// prevent keep going infinitely the task if the robot can�t move and
// finally, the variable factor to reduce the velocity of the robot.
// Note: The distance in inches.
void moveBaseWithFactor(int distance, int time, float factor, int factor2){
	distance = inchesToTicks(distance);
	writeDebugStreamLine("Start moveBaseFront");
	writeDebugStreamLine("Target distance = %d", distance);
	int encoderAvg = 0;
	encoderR = 0;
	encoderL = 0;
	float initialGyro = SensorValue[gyro], actualGyro = initialGyro;
	writeDebugStreamLine("Initial Gyro Position = %f", initialGyro);
	int startEncoderValueR = SensorValue[encR];
	int startEncoderValueL = SensorValue[encL];
	bool atPos=false;
	float pidMovResult;
	//float pidStraightResult;
	counter = 0;

	PID pidMovement;
	//PID pidStraight;
	PIDInit(&pidMovement, 0.10, 0.2, 0.25); // Set P, I, and D constants //0.15,0.1,0.25
	//PIDInit(&pidStraight, 2, 0, 0.3);//Set constants for driving straight

	float grow = 0;
	clearTimer(T1);
	int timer = T1;
	while(!atPos && timer < time){
		encoderR = (SensorValue[encR] - startEncoderValueR)*-1;//Reversing
		encoderL = SensorValue[encL] - startEncoderValueL;
		encoderAvg = (encoderR+encoderL)/2;
		writeDebugStreamLine("encR = %d\tencL = %d", encoderR, encoderL);
		pidMovResult = PIDCompute(&pidMovement, distance - encoderAvg);
		//pidStraightResult = PIDCompute(&pidStraight, initialGyro - actualGyro);
		//writeDebugStreamLine("Actual Gyro Position = %f   PID = %f", actualGyro, pidStraightResult);
		moveBase(pidMovResult*factor*grow);
		if (abs(encoderAvg-distance)<2)
			counter++;
		if (counter >= 4)
			atPos = true;
		timer = time1[T1];
		wait1Msec(25);
		actualGyro = SensorValue[gyro];
		grow+=0.01;
	}
	moveBase(0);
	writeDebugStreamLine("Encoder at position = %d", encoderAvg);
}

// Move the base to the back. Request the distance for know how much need to move,
// the variable time is the maximun time for wait to the movement to
// prevent keep going infinitely the task if the robot can�t move and
// finally, the variable factor to reduce the velocity of the robot.
// Note: The distance in inches.
void moveBaseBack(int distance, int time, int slowFactor)
{
	distance = inchesToTicks(distance);

	writeDebugStreamLine("Start moveBaseFront");
	writeDebugStreamLine("Target distance = %d", distance);

	int encoderAvg = 0;
	encoderR = 0;
	encoderL = 0;
	int startEncoderValueR = SensorValue[encR];
	int startEncoderValueL = SensorValue[encL];
	bool atPos=false;
	float pidMovResult, pidStraightResult;
	counter = 0;

	PID pidMovement;
	//PID pidStraight;
	PIDInit(&pidMovement, 0.15, 0, 0.018); // Set P, I, and D constants
	//PIDInit(&pidStraight, 0.5, 0, 0);//Set constants for driving straight

	clearTimer(T1);
	int timer = T1;

	while(!atPos && timer < time)
	{
		encoderR = abs(SensorValue[encR] - startEncoderValueR);
		encoderL = abs(SensorValue[encL] - startEncoderValueL);
		writeDebugStreamLine("encoder derecho = %d", encoderR);
		writeDebugStreamLine("encoder izquierdo = %d", encoderL);
		encoderAvg = (encoderR+encoderL)/2;
		pidMovResult = PIDCompute(&pidMovement, distance - encoderAvg);
		//pidStraightResult = PIDCompute(&pidStraight, encoderL - encoderR);
		pidMovResult = pidMovResult >  127 ? 127 : pidMovResult;

		moveBase(-pidMovResult);

		if (abs(encoderAvg-distance)<2)
			counter++;
		if (counter >= 4)
			atPos = true;
		timer = time1[T1];
		wait1Msec(25);
	}
	stopBase();
	writeDebugStreamLine("Encoder at position = %d", encoderAvg);
}

// Default values for configure the gyro.
float gyroAngle;
int offsetAngle = 90;

// Configure the default values of the gyro.
void setOffsetAngle (int angle){
	offsetAngle = angle;
}

// Task for get the value of the gyro
task getGyro {
	while(1){
		gyroAngle = (SensorValue[gyro]/10.0 + offsetAngle)%360;//Offset can be changed with function.
		//writeDebugStreamLine("gyro = %d", gyroAngle);
		wait1Msec(25);																	 //Default is 90.
	}
}

// Rotate the base to one side. The variable time is the maximun time for wait to the rotation to
// prevent keep going infinitely the task if the robot can�t rotate.
// The variable angle is for the finally wanted angle.
void rotateToAngle(float targetAngle, int time){
	writeDebugStreamLine("Start rotateToAngle");
	writeDebugStreamLine("Target angle = %f", targetAngle);
	bool atGyro=false;
	float pidGyroResult;
	counter = 0;

	PID pidGyro;
	PIDInit(&pidGyro, 0.2, 0, 0.3); // Set P, I, and D constants

	clearTimer(T1);
	int timer = time1[T1];
	while(!atGyro && timer < time){
		pidGyroResult = PIDCompute(&pidGyro, targetAngle - gyroAngle);
		rotateBase(11*pidGyroResult);
		if (abs(gyroAngle-targetAngle)<0.1)
			counter++;
		if (counter > 3)
			atGyro = true;
		timer = time1[T1];
		wait1Msec(20);
	}
	stopBase();
}

// Rotate the base to one side. The variable time is the maximun time for wait to the rotation to
// prevent keep going infinitely the task if the robot can�t rotate.
// The variable angle is for the amount of angle to rotate.
void rotateThisAngle(float angle, int time){
	writeDebugStreamLine("Start rotateThisAngle");
	writeDebugStreamLine("Target angle = %f", angle);
	bool atGyro = false;
	float pidGyroResult;
	counter = 0;

	PID pidGyro;
	PIDInit(&pidGyro, 0.3, 0, 0.4); // Set P, I, and D constants

	angle = (gyroAngle + angle)%360;

	clearTimer(T1);
	int timer = time1[T1];
	while(!atGyro && timer < time){
		pidGyroResult = PIDCompute(&pidGyro, angle - gyroAngle);
		rotateBase(15*pidGyroResult);
		if (abs(gyroAngle-angle)<2)
			counter++;
		if (counter > 3)
			atGyro = true;
		timer = time1[T1];
		wait1Msec(20);
	}
	stopBase();
}

//*********************************************************************************************
//			Arm Functions
//*********************************************************************************************

void setTower(int speed)
{
	motor[riTower] = speed;
	motor[liTower] = speed;
	motor[yTower] = speed;
}

// Move the arm up and down. The amximun up is the value 2200 and the maximum down is the value 0.
// Height => [0 | 2000]
int armPotInit = 0;
void moveArmTo(int height)
{
	int currHeight = -1*(SensorValue[armPot]-armPotInit);
	if(	currHeight < height )
	{
		while(currHeight < height)
		{
			setTower(127);
			currHeight = -1*(SensorValue[armPot]-armPotInit);
			writeDebugStreamLine("%f<%f", currHeight, height);
		}
	}
	else
	{
		while(currHeight > height)
		{
			setTower(-127);
			currHeight = -1*(SensorValue[armPot]-armPotInit);
		}
	}
	setTower(0);
}

//Up the arm, open the gripper and down the arm
void armThrow()
{
	int armPotVal = SensorValue[armPot];
	bool throw = true;
	//up the arm
	while (-(armPotVal - armPotInit) < 2200)
	{
		writeDebugStreamLine("Condici�n %d > 2000", (armPotVal - armPotInit));
		setTower(127);
		delay(50);
		armPotVal = SensorValue[armPot];

		if(-(armPotVal - armPotInit) > 2000 && throw)
		{
			//Open gipper
			gripperAction(1);
			throw = false;
		}
	}

	//Down the arm
	while (-(armPotVal - armPotInit) > 5)
	{
		writeDebugStreamLine("Condici�n %d < 5", (armPotVal - armPotInit));
		setTower(-127);
		delay(50);
		armPotVal = SensorValue[armPot];
	}

	setTower(0);
}

// Move the robot to the back while throwing objects. Parameter
// height will move the arm up to that point, then the arm will
// go up while the base moves back, the base will stop and the
// gripper will open. Finally the arm will go back down.
//
//Parameter: height - Pre-Movement heigth in pot value units
void armThrowWhileMoving(int height)
{
	moveArmTo(height);
	int armPotVal = SensorValue[armPot];
	bool throw = true;
	//up the arm
	moveBase(-127);
	while (-(armPotVal - armPotInit) < 2200)
	{
		writeDebugStreamLine("Condici�n %d > 2000", (armPotVal - armPotInit));
		setTower(127);
		delay(50);
		armPotVal = SensorValue[armPot];

		if(-(armPotVal - armPotInit) > 2000 && throw)
		{
			//Open gipper
			gripperAction(1);
			throw = false;
		}
	}
	stopBase();
	delay(500);
	//Down the arm
	while (-(armPotVal - armPotInit) > 5)
	{
		writeDebugStreamLine("Condici�n %d < 5", (armPotVal - armPotInit));
		setTower(-127);
		delay(50);
		armPotVal = SensorValue[armPot];
	}

	setTower(0);
}

// Move the robot to the back while throwing objects. Parameter
// height will move the arm up to that point, then the arm will
// go up while the base moves back, the base will stop and the
// gripper will open. Finally the arm will go back down.
//
//Parameter: height - Pre-Movement heigth in pot value units
//Parameter: distance - Full distance to fence
void armThrowWhileMoving(int height, float distance)
{
	int ticks = inchesToTicks(distance);
	int preEncR = SensorValue[encR];
	int preEncL = SensorValue[encL];
	int error = ticks;
	float prop = 50;
	int armPotVal = SensorValue[armPot];
	bool throw = true;
	//up the arm
	moveArmTo(height);
	while ((-(armPotVal - armPotInit) < 2200) && error>0)
	{
		error = SensorValue[encR]+SensorValue[encL]-preEncR-preEncL;
		moveBase(error*prop*-1);
		writeDebugStreamLine("Condici�n %d > 2000", (armPotVal - armPotInit));
		setTower(127);
		delay(50);
		armPotVal = SensorValue[armPot];

		if(-(armPotVal - armPotInit) > 2000 && throw)
		{
			//Open gipper
			gripperAction(1);
			throw = false;
		}
	}
	stopBase();
	delay(400);
	//Down the arm
	while (-(armPotVal - armPotInit) > 5)
	{
		writeDebugStreamLine("Condici�n %d < 5", (armPotVal - armPotInit));
		setTower(-127);
		delay(50);
		armPotVal = SensorValue[armPot];
	}

	setTower(0);
}

//*********************************************************************************************
//			Routines
//*********************************************************************************************

void auto1() //100% Risk
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: right 		                                       */
	/*		  Start angle: 90                                                      */
	/*---------------------------------------------------------------------------*/
	setOffsetAngle(90);

	//Drops preload right fence
	gripperAction(0);
	moveArmTo(1900);
	setTower(20);
	moveBaseWithFactor(4, 1000, 1);
	rotateToAngle(70, 800);
	moveBaseWithFactor(18, 2000, 1);
	rotateToAngle(95,800);
	moveBaseWithFactor(18, 1000, 1);
	gripperAction(1);

	//Push right fence objects
	moveBaseBack(10, 1000, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(10,1000,1);

	//Picks up cube
	setTower(-127);
	moveBaseBack(23,2000,1);
	moveArmTo(5);
	rotateToAngle(180, 1000);
	moveBaseWithFactor(20, 1000, 1);
	gripperAction(0);

	//Drops cube center fence
	moveArmTo(1900);
	setTower(20);
	moveBaseWithFactor(12,500,1);
	rotateToAngle(95, 1000);
	moveBaseWithFactor(25, 2000, 1);
	gripperAction(1);

	//Push fence objects center fence
	moveBaseBack(15, 1000, 1);
	moveArmTo(1500);
	setTower(20);
	moveBaseWithFactor(10,1000,1);

	//Picks up 2-3 back stars
	moveBaseBack(10,1000,1);
	rotateToAngle(273, 1500);
	moveArmTo(50);
	moveBaseWithFactor(20,1500,0.7);
	gripperAction(0);

	//Drops stars left fence
	moveBaseBack(10,1000,1);
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(180,1000);
	moveBaseWithFactor(30,1500,1);
	rotateToAngle(95,1000);
	moveBaseWithFactor(20,1000,1);
	gripperAction(1);

	//Push left fence objects
	moveBaseBack(15, 1000, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(15,1000,1);
}

void auto2() //0 Risk
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: left                                             */
	/* 		  Start angle: 0	                                                     */
	/*---------------------------------------------------------------------------*/

	setOffsetAngle(0);

	//Pick up 3 back stars plus preload
	moveGripperTo(6);
	moveArmTo(500);
	moveGripperTo(1550);
	moveArmTo(5);
	moveBaseWithFactor(40, 2500, 0.7);
	gripperAction(0);

	//Drop stars right fence
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(25,500);
	moveBaseWithFactor(25,1500,1);
	rotateToAngle(90, 1500);
	moveBaseWithFactor(35, 2000, 1);
	gripperAction(1);

	//Push right fence objects
	moveBaseBack(10, 1000, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(10,1000,1);

	//Picks up cube
	moveBaseBack(23,1500,1);
	moveArmTo(50);
	rotateToAngle(180, 1000);
	moveBaseWithFactor(20, 1000, 1);
	gripperAction(0);

	//Drops cube right fence
	moveArmTo(1900);
	setTower(20);
	moveBaseBack(12,500,1);
	rotateToAngle(90, 1000);
	moveBaseWithFactor(25, 2000, 1);
	gripperAction(1);

	////Sweep
	//moveBaseBack(23,2000,1);
	//moveArmTo(50);
	//rotateToAngle(180, 1000);
	//moveBaseWithFactor(80, 1000, 1);
	//gripperAction(0);

	////Drop to the left fence
	//moveArmTo(1900);
	//setTower(20);
	//rotateToAngle(90, 1000);
	//moveBaseWithFactor(25, 2000, 1);
	//gripperAction(1);

}

void auto2_mirror() //0 Risk
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: Right                                            */
	/* 		  Start angle: 180	                                                   */
	/*---------------------------------------------------------------------------*/

	setOffsetAngle(180);

	//Pick up 3 back stars plus preload
	moveGripperTo(6);
	moveArmTo(500);
	moveGripperTo(1550);
	moveArmTo(5);
	moveBaseWithFactor(40, 2500, 0.7);
	gripperAction(0);

	//Drop stars left fence
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(155,500);
	moveBaseWithFactor(30,1500,1);
	rotateToAngle(90, 1500);
	moveBaseWithFactor(35, 2000, 1);
	gripperAction(1);

	//Push left fence objects
	moveBaseBack(10, 1000, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(10,1000,1);

	//Picks up cube
	moveBaseBack(23,1500,1);
	moveArmTo(50);
	rotateToAngle(0, 1000);
	moveBaseWithFactor(20, 1000, 1);
	gripperAction(0);

	//Drops cube left fence
	moveArmTo(1900);
	setTower(20);
	moveBaseBack(12,500,1);
	rotateToAngle(90, 1000);
	moveBaseWithFactor(25, 2000, 1);
	gripperAction(1);

	////Sweep
	//moveBaseBack(23,2000,1);
	//moveArmTo(50);
	//rotateToAngle(0, 1000);
	//moveBaseWithFactor(80, 1000, 1);
	//gripperAction(0);

	////Drop to the left fence
	//moveArmTo(1900);
	//setTower(20);
	//rotateToAngle(90, 1000);
	//moveBaseWithFactor(155, 2000, 1);
	//gripperAction(1);

}

void auto3() //Cube first
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: left                                             */
	/* 		  Start angle: 90	                                                     */
	/*---------------------------------------------------------------------------*/

	setOffsetAngle(90);

	//Pick cube and preload
	moveBaseWithFactor(18, 1000, 1);
	rotateToAngle(0, 800);
	moveBaseWithFactor(10, 800, 1);
	gripperAction(0);
	delay(500);

	//Drop through center fence
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(0,500);
	moveBaseWithFactor(13, 1000, 1);
	rotateToAngle(90, 1000);
	moveBaseWithFactor(23, 1000, 1);
	gripperAction(1);

	//Push stars from center fence
	moveBaseBack(10, 1000, 1);
	moveArmTo(1500);
	setTower(20);
	moveBaseWithFactor(10,1000,1);

	//Pick 3 stars from back (drift)
	moveBaseBack(10,500,1);
	rotateToAngle(42, 1500);
	moveBaseBack(44, 2000,1);
	rotateToAngle(0, 800);
	moveGripperTo(1550);
	moveArmTo(5);
	moveBaseWithFactor(40, 2500, 0.7);
	gripperAction(0);

	//Drop stars right fence
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(25,500);
	moveBaseWithFactor(25,1500,1);
	rotateToAngle(90, 1500);
	moveBaseWithFactor(30, 1500, 1);
	gripperAction(1);

	//Push right fence objects
	moveBaseBack(10, 1000, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(10,1000,1);

	////Sweep
	//moveBaseBack(23,2000,1);
	//moveArmTo(50);
	//rotateToAngle(180, 1000);
	//moveBaseWithFactor(80, 1000, 1);
	//gripperAction(0);

	////Drop through left fence
	//moveArmTo(1900);
	//setTower(20);
	//rotateToAngle(90, 1000);
	//moveBaseWithFactor(25, 2000, 1);
	//gripperAction(1);

}

void auto4() //0 Risk 4
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: Right                                            */
	/* 		  Start angle: 180	                                                   */
	/*---------------------------------------------------------------------------*/

	setOffsetAngle(180);

	//Pick up 3 back stars plus preload
	moveGripperTo(6);
	moveArmTo(500);
	moveGripperTo(1650);
	moveArmTo(5);
	moveBaseWithFactor(38, 2500, 0.7);
	gripperAction(0);

	//Drop stars left fence
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(155,500);
	moveBaseWithFactor(30,1500,1);
	rotateToAngle(90, 1500);
	moveBaseWithFactor(30, 2000, 1);
	gripperAction(1);

	//Push left fence objects
	moveBaseBack(10, 1000, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(10,1000,1);

	//Picks up cube
	moveBaseBack(23,1500,1);
	moveArmTo(50);
	rotateToAngle(0, 1000);
	moveBaseWithFactor(18, 800, 1);
	gripperAction(0);

	//Drops cube right fence
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(0, 500);
	moveBaseWithFactor(38,2000,1);
	rotateToAngle(93, 1500);
	moveBaseWithFactor(28, 2000, 1);
	gripperAction(1);

	//Push stars from right fence
	moveBaseBack(10, 700, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(10,700,1);

	////Sweep
	//moveBaseBack(23,2000,1);
	//moveArmTo(50);
	//rotateToAngle(0, 1000);
	//moveBaseWithFactor(80, 1000, 1);
	//gripperAction(0);

	////Drop to the left fence
	//moveArmTo(1900);
	//setTower(20);
	//rotateToAngle(90, 1000);
	//moveBaseWithFactor(155, 2000, 1);
	//gripperAction(1);

}

void programmingSkills()
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: right                                            */
	/* 		  Start angle: 90	                                                     */
	/*---------------------------------------------------------------------------*/

	setOffsetAngle(90);

	//Preloads to left fence
	rotateToAngle(110, 500);
	gripperAction(2);
	moveArmTo(1900);
	setTower(50);
	delay(200);
	gripperAction(0);
	delay(750);
	//moveBaseWithFactor(55, 2500, 0.5, 70);
	moveBaseWithFactor(55, 1200, 1);
	gripperAction(1);

	//Push objects from left fence
	moveBaseBack(10, 700, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(10,700,1);

	//Picks up cube
	moveBaseBack(23,1000,1);
	moveArmTo(5);
	rotateToAngle(0, 1000);
	moveBaseWithFactor(20, 1000, 1);
	gripperAction(0);

	//Drops cube left fence
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(0, 500);
	moveBaseWithFactor(38,2000,1);
	rotateToAngle(92, 1000);
	moveBaseWithFactor(25, 1000, 1);
	gripperAction(1);

	//Push left fence objects
	moveBaseBack(15, 1000, 1);
	moveArmTo(1700);
	setTower(20);
	moveBaseWithFactor(15,1000,1);

	//Picks up hanging star
	moveBaseBack(10,1000,1);
	rotateToAngle(-70, 1500);
	moveGripperTo(1400);
	moveArmTo(5);
	moveBaseWithFactor(18,1000,1);
	gripperAction(0);

	//Drops hanging star
	moveBaseBack(10,1000,1);
	moveArmTo(1900);
	setTower(20);
	rotateToAngle(100,1500);
	moveBaseWithFactor(32,1500,1);
	gripperAction(1);

	//Pick stars from front
	setTower(-127);
	moveBaseBack(19,1000,1);
	moveArmTo(5);
	rotateToAngle(180, 1000);
	moveBaseWithFactor(25, 1000, 1);
	rotateToAngle(90, 1000);
	moveBaseWithFactor(10, 500, 1);
	gripperAction(0);

	//Drop through center
	moveBaseBack(15, 500, 1);
	moveArmTo(1900);
	setTower(20);
	moveBaseWithFactor(25, 1000, 1);
	gripperAction(1);

	//Push stars from center
	moveBaseBack(10, 1000, 1);
	moveArmTo(1500);
	setTower(20);
	moveBaseWithFactor(10,1000,1);

	//Hang
	rotateToAngle(90,500);
	moveArmTo(1700);
	setTower(20);
	moveBaseBack(15, 500, 1);
	rotateToAngle(0, 1000);
	moveBaseWithFactor(25, 2000, 1);
	rotateToAngle(-45, 800);
	//Si se empieza de esquina antes del tubo
	//gripperAction(0);
	//moveArmTo(1700);
	//gripperAction(1);
	moveBaseWithFactor(23,1000,1);
	rotateToAngle(-50,500);
	moveBaseWithFactor(5,500,1);
	moveArmTo(1650);
	setTower(15);
	gripperAction(0);
	moveBaseBack(10,800,1);
	moveBaseWithFactor(10,800,1);
	delay(250);
	moveBase(127);
	moveArmTo(100);
	while(1)
	{
		setTower(-127);
		moveBase(0);
		motor[gripper]=-127;
	}
}

//*********************************************************************************************
//			LCD Functions
//*********************************************************************************************

string mainBattery, secondBattery;

//Wait for Press--------------------------------------------------
void waitForPress()
{
	while(nLCDButtons == 0 && bIfiRobotDisabled)
	{
		wait1Msec(5);
	}
}
//----------------------------------------------------------------

//Wait for Release------------------------------------------------
void waitForRelease()
{
	while(nLCDButtons != 0)
	{
		wait1Msec(5);
	}
}
//----------------------------------------------------------------

void preautonomous_selector()
{
	const short leftButton = 1;
	const short centerButton = 2;
	const short rightButton = 4;

	//------------- Beginning of User Interface Code ---------------
	//Clear LCD
	clearLCDLine(0);
	clearLCDLine(1);

	//Loop while robot is disabled
	while(bIfiRobotDisabled)
	{
		//Display Battery Levels On Top Row
		displayLCDString(0, 0, "    B1: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0);
		displayNextLCDString(mainBattery);
		displayNextLCDString("          ");
		//Switch case that allows the user to choose from 8 different options
		switch(count){
		case 0:
			displayLCDCenteredString(1, "< Auto 1 >");
			waitForPress();
			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count = 6;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 1:
			displayLCDCenteredString(1, "< Auto 2 >");
			waitForPress();
			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 2:
			displayLCDCenteredString(1, "< Auto 2_M >");
			waitForPress();
			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 3:
			displayLCDCenteredString(1, "< Auto 3 >");
			waitForPress();
			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 4:
			displayLCDCenteredString(1, "< Auto 4 >");
			waitForPress();
			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 5:
			displayLCDCenteredString(1, "< Prog Skills >");
			waitForPress();
			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 6:
			displayLCDCenteredString(1, "< No Auto >");
			waitForPress();
			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count = 0;
			}
			break;
		default:
			count = 0;
			break;
		}
	}
}

void autonomous_selector(int auto_selection)
{
	//------------- Beginning of Robot Movement Code ---------------
	//Clear LCD
	clearLCDLine(0);
	clearLCDLine(1);
	//runs the user choice
	if(auto_selection==0)
	{
		displayLCDCenteredString(0, "Auto 1");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		auto1();
	}
	else if(auto_selection == 1)
	{
		//If count = 1, run the code correspoinding with choice 2
		displayLCDCenteredString(0, "Auto 2");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		auto2();
	}
	else if(auto_selection == 2)
	{
		//If count = 2, run the code correspoinding with choice 3
		displayLCDCenteredString(0, "Auto 2_M");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		auto2_mirror();
	}
	else if(auto_selection == 3)
	{
		//If count = 3, run the code correspoinding with choice 4
		displayLCDCenteredString(0, "Auto 3");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		auto3();
	}
	else if(auto_selection == 4)
	{
		displayLCDCenteredString(0, "Auto 4");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		auto4();
	}
	else if(auto_selection == 5)
	{
		displayLCDCenteredString(0, "Prog Skills");
		wait1Msec(100);
		programmingSkills();
	}
	else if(auto_selection == 6)
	{
		displayLCDCenteredString(0, "No auto");
		wait1Msec(100);
		//NoAuto
	}
	else
	{
		displayLCDCenteredString(0, "No valid choice");
		displayLCDCenteredString(1, "was made!");
	}
	//------------- End of Robot Movement Code -----------------------
}

void display_battery_levels()
{
	//Display Battery Levels
	displayLCDString(0, 0, "B1:");
	sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0);
	displayNextLCDString(mainBattery);
	displayNextLCDString("  B2:");
	sprintf(secondBattery, "%1.2f%c", SensorValue[Expander]/280.0);
	displayNextLCDString(secondBattery);
	displayLCDCenteredString(1, "Dory");
}

//*********************************************************************************************
//			Other Functions
//*********************************************************************************************

// Initialize BNS library and sensors.
void init()
{
	BNS();

	//Initialize Gyro
	SensorType[gyro] = sensorNone;
	wait1Msec(1000);
	SensorType[gyro] = sensorGyro;
	wait1Msec(2000);
	//End Initialize Gyro

	SensorValue[encR] = 0;
	SensorValue[encL] = 0;

	startTask(getGyro);

	clearDebugStream();
	armPotInit = SensorValue[armPot];
	gripperPotInit = SensorValue[gripperPot];
}

// Testing for the potentiometer and the encoder.
void test_pot_and_enc()
{
	while(1)
	{
		writeDebugStreamLine("armPot = %f", SensorValue[armPot]-armPotInit);
		writeDebugStreamLine("gripperPot = %f", SensorValue[gripperPot]-gripperPotInit);
		writeDebugStreamLine("gyro = %f", gyroAngle);
		writeDebugStreamLine("encR = %f, encL = %f", SensorValue[encR], SensorValue[encL]);
		delay(200);
	}
}

void check_all_motors()
{
	int i;
	for(i=0;i<10;i++)
	{
		motor[i]=100;
		delay(1000);
		motor[i]=0;
	}
}

// User control code
void user_control()
{
	while(1)
	{
		display_battery_levels();

		motor[rfBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[rmBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[rbBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[lfBase]= vexRT[Ch3] + vexRT[Ch4];
		motor[lmBase]= vexRT[Ch3] + vexRT[Ch4];
		motor[lbBase]= vexRT[Ch3] + vexRT[Ch4];

		setTower(vexRT[Ch2]);

		//New motor gripper prog
		if(vexRT[Btn6U])
		{
			motor[gripper] = 127;
		}
		else if (vexRT[Btn6D])
		{
			motor[gripper] = -127;
		}
		else if (vexRT[Btn5U])
		{
			motor[gripper] = 30;
		}
		else if(vexRT[Btn5D])
		{
			motor[gripper] = -30;
		}
		else
		{
			motor[gripper] = 0;
		}

		////Experimental pneumatic gripper
		//if(vexRT[Btn6U])
		//{
		//	SensorValue[gripperR] = 0;
		//	SensorValue[gripperL] = 0;
		//}
		//else if (vexRT[Btn6D])
		//{
		//	SensorValue[gripperR] = 1;
		//	SensorValue[gripperL] = 1;
		//}

		////Tested motor gripper
		//if(vexRT[Btn6U])
		//{
		//	if(SensorValue[in1]>2900 && SensorValue[in1]<3700)
		//		motor[gripper]=50;
		//	else
		//		motor[gripper]=127;
		//}
		//else if(vexRT[Btn6D])
		//{
		//	motor[gripper] = -127;
		//}
		//else
		//{
		//	motor[gripper]=0;
		//}
	}
}
#endif
