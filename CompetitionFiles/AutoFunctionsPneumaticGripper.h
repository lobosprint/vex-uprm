#ifndef AutoFunctionsPneumaticGripper.h
#define AutoFunctionsPneumaticGripper.h
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
	while (abs(armPotVal - armPotInit) < 1580)
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
//			Routines
//*********************************************************************************************

void auto1()
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: right 				                                   */
	/*        Oriented: left 				 						                                 */
	/*---------------------------------------------------------------------------*/

	//Configure angle for the left orientation of the robot
	setOffsetAngle(180);

	//Configure gripper
	//gripperAction(0);
	//delay(2000);

	gripperAction(1);

	//Take the stars
	moveBaseWithFactor(62, 2200, 1);

	//Close the gripper
	gripperAction(0);

	//Up the arm
	moveArmTo(300);
	setTower(20);

	moveBaseBack(5,250,1);

	//Up the arm more than 12 inches
	moveArmTo(800);
	setTower(20);
	//Rotate the robot backwards to the center line
	rotateToAngle(270, 1200);

	//Throw the stars
	armThrowWhileMoving(1100,42);

	//Turn the robot in the way of the cube
	rotateToAngle(343, 1000);

	//Go to take the cube
	moveBaseWithFactor(15, 890, 1);

	//Close the gripper and up the arm
	gripperAction(0);
	moveArmTo(350);
	setTower(20);

	//Rotate the robot backwards to the center line
	rotateToAngle(270, 1000);

	//Throw the cube
	armThrowWhileMoving(1000, 15);

	//Turn the robot to the right side
	moveArmTo(0);
	rotateToAngle(359, 1300);

	//Go to the end of the field
	moveBaseWithFactor(24, 2000, 1);

	//Turn the robot backwards to the center line
	rotateToAngle(300, 850);

	//Go to the end of the field
	moveBaseWithFactor(20, 1000, 1);

	//Close the gripper and up the arm
	gripperAction(0);
	moveArmTo(350);
	setTower(20);

	rotateToAngle(270, 1000);

	//Throw the star and the objects taked in the way
	armThrowWhileMoving(1100,24);

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
	gripperAction(0);

	//Go to the cube
	moveBaseWithFactor(16,1000,1);
	rotateToAngle(0,1000);
	gripperAction(1);
	moveBaseWithFactor(18,1200,1);
	gripperAction(0);

	//Move the cube
	moveArmTo(300);
	setTower(20);
	moveBaseBack(33,1600,1);
	delay(600);
	//Throw the cube
	rotateToAngle(-90,1000);
	armThrowWhileMoving(1600, 25);
	setOffsetAngle(270);

	//Go to take the first star
	moveBaseWithFactor(20,1200,1);
	gripperAction(0);

	//Up the arm for rotate
	moveArmTo(300);
	setTower(20);
	rotateToAngle(295,700);
	moveBaseWithFactor(8,1000,1);
	rotateToAngle(360,1000);
	moveArmTo(0);
	gripperAction(1);

	//Take more stars
	moveBaseWithFactor(78,3300,1);
	gripperAction(0);
	moveArmTo(800);
	setTower(20);
	rotateToAngle(270,1800);

	//Throw the stars
	armThrowWhileMoving(2200, 32);

		sleep(10000000);
}

void auto3()
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: left 				                                   	 */
	/*        Oriented: right 				 						                               */
	/*---------------------------------------------------------------------------*/


	//Configure angle for the left orientation of the robot
	setOffsetAngle(0);

	//Configure gripper
	//gripperAction(0);
	//delay(2000);

	gripperAction(1);

	//Take the stars
	moveBaseWithFactor(62, 2200, 1);

	//Close the gripper
	gripperAction(0);

	//Up the arm
	moveArmTo(300);
	setTower(20);

	moveBaseBack(10,500,1);

	//Up the arm more than 12 inches
	moveArmTo(800);
	setTower(20);
	//Rotate the robot backwards to the center line
	rotateToAngle(-90, 1200);

	//Throw the stars
	armThrowWhileMoving(1100,42);

	//Turn the robot in the way of the cube
	rotateToAngle(-165, 1200);

	//Go to take the cube
	moveBaseWithFactor(15, 890, 1);

	//Close the gripper and up the arm
	gripperAction(0);
	moveArmTo(350);
	setTower(20);

	//Rotate the robot backwards to the center line
	rotateToAngle(-90, 1000);

	//Throw the cube
	armThrowWhileMoving(1000, 15);

	//Turn the robot to the right side
	moveArmTo(0);
	rotateToAngle(-180, 1300);

	//Go to the end of the field
	moveBaseWithFactor(24, 2000, 1);

	//Turn the robot backwards to the center line
	rotateToAngle(-130, 850);

	//Go to the end of the field
	moveBaseWithFactor(20, 1000, 1);

	//Close the gripper and up the arm
	gripperAction(0);
	moveArmTo(350);
	setTower(20);

	rotateToAngle(-90, 1000);

	//Throw the star and the objects taked in the way
	armThrowWhileMoving(1100,24);

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

void auto4()
{
	/*---------------------------------------------------------------------------*/
	/*        Field start side: right 				                                   */
	/*        Oriented: front 				 						                                 */
	/*---------------------------------------------------------------------------*/
	//Configure angle
	setOffsetAngle(90);
	gripperAction(0);

	//Take the cube
	moveBaseWithFactor(16,1000,1);
	rotateToAngle(180, 1200);
	gripperAction(1);
	moveBaseWithFactor(25,1200,1);
	gripperAction(0);

	//Move the cube
	moveArmTo(700);
	setTower(20);
	moveBaseWithFactor(36,1200,1);

	//Throw the cube
	rotateToAngle(275,1000);
	armThrowWhileMoving(1000, 16);

	//Go to take the first star
	moveBaseWithFactor(20,1200,1);
	gripperAction(0);

	//Up the arm for rotate
	moveArmTo(300);
	setTower(20);
	rotateToAngle(295,700);
	moveBaseWithFactor(8,1000,1);
	rotateToAngle(360,1000);
	moveArmTo(0);

	gripperAction(1);

	//Take more stars
	moveBaseWithFactor(78,3300,1);
	gripperAction(0);
	moveArmTo(800);
	setTower(20);
	rotateToAngle(270,1800);

	//Throw the stars
	armThrowWhileMoving(2200, 32);

		sleep(10000000);
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
		case 3:
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
		case 4:
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
				count++;
			}
			break;
		case 5:
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
		displayLCDCenteredString(0, "Auto 3");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		auto3();
	}
	else if(auto_selection == 3)
	{
		//If count = 3, run the code correspoinding with choice 4
		displayLCDCenteredString(0, "Auto 4");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		auto4();
	}
	else if(auto_selection == 4)
	{
		displayLCDCenteredString(0, "No Auto");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		//No Auto
	}
	else if(auto_selection == 5)
	{
		displayLCDCenteredString(0, "No auto");
		wait1Msec(100);
		//No auto
	}
	else if(auto_selection == 6)
	{
		displayLCDCenteredString(0, "No Auto");
		wait1Msec(100);
		//No Auto
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
	displayLCDString(0, 0, "    B1: ");
	sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0);
	displayNextLCDString(mainBattery);
	displayNextLCDString("          ");
	displayLCDCenteredString(1, "Pnemo-XX");
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

void test_gyro()
{
	writeDebugStreamLine("Gyro: %d", gyroAngle);
}

// User control code
void userControl()
{
	while(1)
	{
		display_battery_levels();

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
#endif
