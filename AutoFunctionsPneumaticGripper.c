#pragma config(Sensor, in1,    armPot,         sensorPotentiometer)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  gripperRight,   sensorDigitalOut)
#pragma config(Sensor, dgtl2,  gripperLeft,    sensorDigitalOut)
#pragma config(Sensor, dgtl5,  encP,           sensorQuadEncoder)
#pragma config(Sensor, dgtl8,  encL,           sensorQuadEncoder)
#pragma config(Sensor, dgtl11, encR,           sensorQuadEncoder)
#pragma config(Motor,  port1,           rfBase,        tmotorVex393HighSpeed_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           riTower,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           rbBase,        tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           rmBase,        tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           roTower,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           lbBase,        tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           lmBase,        tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           loTower,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           liTower,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          lfBase,        tmotorVex393HighSpeed_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//r = right, l = left, b = back, m = middle, f = front, i = inner, o = outer

//*********************************************************************************************
//			Includes
//*********************************************************************************************

#include "BNSLib.h";

//*********************************************************************************************
//			Gripper Functions
//*********************************************************************************************

//Action = 1 -> Open
//Action = 0 -> Close
void gripperAction(bool action)
{
	if(action)
	{
		SensorValue[gripperLeft] = 0;
		SensorValue[gripperRight] = 0;
	}
	else
	{
		SensorValue[gripperLeft] = 1;
		SensorValue[gripperRight] = 1;
	}
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
	motor[rmBase]=speed;
	motor[rfBase]=speed;
	motor[lbBase]=speed;
	motor[lmBase]=speed;
	motor[lfBase]=speed;
}

// Funtion to correct the aligment of the robot when move to the front or back.
void moveBase(int speed, int offset)
{
	motor[rbBase]=speed + offset;
	motor[rmBase]=speed + offset;
	motor[rfBase]=speed + offset;
	motor[lbBase]=speed - offset;
	motor[lmBase]=speed - offset;
	motor[lfBase]=speed - offset;
}

// Rotate the base clockwise or counter-clockwise.
void rotateBase(int speed)
{
	motor[rbBase]=speed;
	motor[rmBase]=speed;
	motor[rfBase]=speed;
	motor[lbBase]=-speed;
	motor[lmBase]=-speed;
	motor[lfBase]=-speed;
}

// Stop moving the robot.
void stopBase()
{
	motor[rbBase]=0;
	motor[rmBase]=0;
	motor[rfBase]=0;
	motor[lbBase]=0;
	motor[lmBase]=0;
	motor[lfBase]=0;
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
	PIDInit(&pidMovement, 0.5, .1, 0.25); // Set P, I, and D constants
	//PIDInit(&pidStraight, 2, 0, 0.3);//Set constants for driving straight

	clearTimer(T1);
	int timer = T1;
	while(!atPos && timer < time){
		encoderR = (SensorValue[encR] - startEncoderValueR);
		encoderL = SensorValue[encL] - startEncoderValueL;
		encoderAvg = (encoderR+encoderL)/2;
		writeDebugStreamLine("encR = %d\tencL = %d", encoderR, encoderL);
		pidMovResult = PIDCompute(&pidMovement, distance - encoderAvg);
		//pidStraightResult = PIDCompute(&pidStraight, initialGyro - actualGyro);
		//writeDebugStreamLine("Actual Gyro Position = %f   PID = %f", actualGyro, pidStraightResult);
		moveBase(pidMovResult);
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
	PIDInit(&pidMovement, 0.2, 0, 0.018); // Set P, I, and D constants
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
		gyroAngle = (SensorValue[gyro]/10.0 + offsetAngle);//Offset can be changed with function.
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
	PIDInit(&pidGyro, 0.1, 0, 0.3); // Set P, I, and D constants

	clearTimer(T1);
	int timer = time1[T1];
	while(!atGyro && timer < time){
		pidGyroResult = PIDCompute(&pidGyro, targetAngle - gyroAngle);
		rotateBase(18*pidGyroResult);
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
	motor[roTower] = speed;
	motor[liTower] = speed;
	motor[loTower] = speed;
}

// Move the arm up and down. The amximun up is the value 2200 and the maximum down is the value 0.
// Height => [0 | 2200]
int armPotInit = 0;
void moveArmTo(int height)
{
	int currHeight = abs(SensorValue[armPot]-armPotInit);
	if(	abs(currHeight) < height )
	{
		while(currHeight < height)
		{
			setTower(127);
			currHeight = abs(SensorValue[armPot]-armPotInit);
			writeDebugStreamLine("%f<%f", currHeight, height);
		}
	}
	else
	{
		while(currHeight > height)
		{
			setTower(-127);
			currHeight = abs(SensorValue[armPot]-armPotInit);
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
	while (abs(armPotVal - armPotInit) < 1660)
	{
		writeDebugStreamLine("Condici�n %d < 1660", (armPotVal - armPotInit));
		setTower(127);
		delay(5);
		armPotVal = SensorValue[armPot];
	}

	gripperAction(1);
	delay(200);
	//Down the arm
	while (abs(armPotVal - armPotInit) > 5)
	{
		writeDebugStreamLine("Condici�n %d > 5", (armPotVal - armPotInit));
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
void armThrowWhileMoving(int time, int distance)
{
		moveBaseBack(distance, time, 1);
		armThrow();
}

//*********************************************************************************************
//			Other Functions
//*********************************************************************************************

// Inicialice BNS library and sensors.
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
}

// Testing for the potenciometer and the ecnoder.
void test_pot_and_enc()
{
	writeDebugStreamLine("armPot = %f", SensorValue[armPot]-armPotInit);
	writeDebugStreamLine("encR = %f, encL = %f, encP = %f", SensorValue[encR], SensorValue[encL], SensorValue[encP]);
	delay(200);
}

//Test motors
void test_motors()
{
	//Tower
	motor[riTower] = 127;
	writeDebugStreamLine("Motor: riTower");
	delay(500);
	motor[riTower] = 0;

	delay(2000);

	motor[roTower] = 127;
	writeDebugStreamLine("Motor: roTower");
	delay(500);
	motor[roTower] = 0;

	delay(2000);

	motor[liTower] = 127;
	writeDebugStreamLine("Motor: liTower");
	delay(500);
	motor[liTower] = 0;

	delay(2000);

	motor[loTower] = 127;
	writeDebugStreamLine("Motor: loTower");
	delay(500);
	motor[loTower] = 0;

	delay(2000);

	//Base
	motor[rbBase] = 127;
	writeDebugStreamLine("Motor: rbBase");
	delay(500);
	motor[rbBase] = 0;

	delay(2000);

	motor[rmBase] = 127;
	writeDebugStreamLine("Motor: rmBase");
	delay(500);
	motor[rmBase] = 0;

	delay(2000);

	motor[rfBase] = 127;
	writeDebugStreamLine("Motor: rfBase");
	delay(500);
	motor[rfBase] = 0;

	delay(2000);

	motor[lbBase] = 127;
	writeDebugStreamLine("Motor: lbBase");
	delay(500);
	motor[lbBase] = 0;

	delay(2000);

	motor[lmBase] = 127;
	writeDebugStreamLine("Motor: lmBase");
	delay(500);
	motor[lmBase] = 0;

	delay(2000);

	motor[lfBase] = 127;
	writeDebugStreamLine("Motor: lfBase");
	delay(500);
	motor[lfBase] = 0;
}

//Test gripper
void test_gripper()
{
	writeDebugStreamLine("Gripper: close");
	gripperAction(0);
	delay(2000);
	writeDebugStreamLine("Gripper: open");
	gripperAction(1);
	delay(2000);
}


// User control code
void userControl()
{
	while(1)
	{
		motor[rbBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[rmBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[rfBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[lbBase]= vexRT[Ch3] + vexRT[Ch4];
		motor[lmBase]= vexRT[Ch3] + vexRT[Ch4];
		motor[lfBase]= vexRT[Ch3] + vexRT[Ch4];

		setTower(vexRT[Ch2]);

		if(vexRT[Btn6U])
		{
			SensorValue[gripperLeft] = 1;
			SensorValue[gripperRight] = 1;
		}
		if(vexRT[Btn6D])
		{
			SensorValue[gripperLeft] = 0;
			SensorValue[gripperRight] = 0;
		}
	}
}

// User control code for Marcos test
void marcosControl()
{
	while(1)
	{
		motor[rbBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[rmBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[rfBase]= vexRT[Ch3] - vexRT[Ch4];
		motor[lbBase]= vexRT[Ch3] + vexRT[Ch4];
		motor[lmBase]= vexRT[Ch3] + vexRT[Ch4];
		motor[lfBase]= vexRT[Ch3] + vexRT[Ch4];

		setTower(vexRT[Ch2]);

		if(vexRT[Btn6U])
		{
			SensorValue[gripperLeft] = 1;
			SensorValue[gripperRight] = 1;
		}
		if(vexRT[Btn5U])
		{
			SensorValue[gripperLeft] = 0;
			SensorValue[gripperRight] = 0;
		}
	}
}

//*********************************************************************************************
//			Routines
//*********************************************************************************************

void auto1()
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: right 				                                   */
	/*        Oriented: left 				 						                                 */
	/*---------------------------------------------------------------------------*/
	sleep(1000);

	//Configure angle for the left orientation of the robot
	setOffsetAngle(180);

	//Configure gripper
	//gripperAction(0);
	//delay(2000);

	gripperAction(1);
	//**delay(300);

	//Take the stars
	moveBaseWithFactor(62, 2200, 1);

	//Close the gripper
	gripperAction(0);

	moveBaseBack(10,500,1);


	//Up the arm more than 12 inches
	moveArmTo(850);
	setTower(20);
	//Rotate the robot backwards to the center line
	rotateToAngle(270, 1200);

	//Throw the stars
	armThrowWhileMoving(1100,42);

	//Turn the robot in the way of the cube
	rotateToAngle(343, 1200);

	//Go to take the cube
	moveBaseWithFactor(26, 1200, 1);

	//Close the gripper
	gripperAction(0);
	moveArmTo(300);
	setTower(20);

	//Rotate the robot backwards to the center line
	rotateToAngle(270, 1000);

	//Throw the cube
	armThrowWhileMoving(1000, 18);

	//Turn the robot to the right side
	rotateToAngle(359, 1000);

	//Go to the end of the field
	moveBaseWithFactor(23, 1100, 1);

	//Turn the robot backwards to the center line
	rotateToAngle(310, 1000);

	//Go to the end of the field
	moveBaseWithFactor(15, 1000, 1);

	//Close the gripper
	gripperAction(0);
	rotateToAngle(280, 1000);

	//Throw the star and the objects taked in the way
	armThrowWhileMoving(1100,22);

		sleep(50000);

	//Turn the robot 70� to the right side
	rotateToAngle(250, 2000);

	//Move to take possibles objects
	moveBaseWithFactor(72, 5000, 0);

	//Close
	gripperAction(0);

	//Throw elements taked during way
	armThrowWhileMoving(1200, 40);

}

void auto2()
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: left 				                                   */
	/*        Oriented: front 				 						                                 */
	/*---------------------------------------------------------------------------*/
	//Configure angle
	setOffsetAngle(90);

	//Go to the cube
	moveBaseWithFactor(72,2000,1);
	rotateThisAngle(180,500);
	moveBaseWithFactor(36,1000,1);
	rotateThisAngle(270,500);
	moveBaseWithFactor(36,1000,1);

	//Take the cube
	gripperAction(0);

	//Throw the cube
	armThrowWhileMoving(1200, 40);

	//Take stars
	moveBaseWithFactor(60,1500,1);
	rotateThisAngle(180,500);
	moveBaseWithFactor(74,1500,1);
	rotateThisAngle(270,500);

	//Throw stars
	armThrowWhileMoving(2200, 72);

	//Go to the last star
	rotateThisAngle(180,500);
	moveBaseWithFactor(60,1500,1);
	rotateThisAngle(270,500);
	moveBaseWithFactor(60,1500,1);

	//Throw the last star
	armThrowWhileMoving(2200, 48);
}

void auto3()
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: left 				                                   	 */
	/*        Oriented: front 				 						                               */
	/*---------------------------------------------------------------------------*/

	//Configure angle
	setOffsetAngle(90);

	//Take the cube
	moveBaseWithFactor(36,2000,1);
	rotateThisAngle(180,500);
	moveBaseWithFactor(36,1000,1);
	gripperAction(0);

	//Throw the cube
	rotateThisAngle(270,500);
	armThrowWhileMoving(2200, 36);

	//Take the stars
	rotateThisAngle(180,500);
	moveBaseWithFactor(48,1000,1);
	rotateThisAngle(270,500);
	moveBaseWithFactor(48,1000,1);
	rotateThisAngle(360,500);
	moveBaseWithFactor(120,3000,1);
	gripperAction(0);

	//Throw the stars
	rotateThisAngle(270,500);
	armThrowWhileMoving(2200, 36);
}

void auto4()
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: right 				                                   */
	/*        Oriented: front 				 						                                 */
	/*---------------------------------------------------------------------------*/
	//Configure angle
	setOffsetAngle(90);

	//Take the cube
	moveBaseWithFactor(36,2000,1);
	rotateThisAngle(0,500);
	moveBaseWithFactor(74,1000,1);
	gripperAction(0);

	//Throw the cube
	rotateThisAngle(270,500);
	armThrowWhileMoving(2200, 36);

	//Take the stars
	moveBaseWithFactor(48,1000,1);
	rotateThisAngle(180,500);
	moveBaseWithFactor(120,1000,1);
	rotateThisAngle(270,500);

	//Throw the stars
	armThrowWhileMoving(2200, 36);
}


//*********************************************************************************************
//			Main Task
//*********************************************************************************************

task main()
{
	//init();

	userControl();

	//while(true)
	//{
//test_gripper();
	//test_motors();


	//	test_pot_and_enc();
	//	delay(50);
	//}

	//test_gripper();

	//gripperAction(0);
	//armThrowWhileMoving(1300);
	//auto1();
//test_gripper();
}
