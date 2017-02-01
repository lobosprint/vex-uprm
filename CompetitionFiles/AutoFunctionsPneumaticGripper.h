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
// prevent keep going infinitely the task if the robot can´t move and
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
		encoderR = (SensorValue[encR] - startEncoderValueR)*-1;//Reversing
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
// prevent keep going infinitely the task if the robot can´t move and
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
		gyroAngle = (SensorValue[gyro]/10.0 + offsetAngle)%360;//Offset can be changed with function.
		//writeDebugStreamLine("gyro = %d", gyroAngle);
		wait1Msec(25);																	 //Default is 90.
	}
}

// Rotate the base to one side. The variable time is the maximun time for wait to the rotation to
// prevent keep going infinitely the task if the robot can´t rotate.
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
		rotateBase(12*pidGyroResult);
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
// prevent keep going infinitely the task if the robot can´t rotate.
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
		writeDebugStreamLine("Condición %d > 2000", (armPotVal - armPotInit));
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
		writeDebugStreamLine("Condición %d < 5", (armPotVal - armPotInit));
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
		writeDebugStreamLine("Condición %d > 2000", (armPotVal - armPotInit));
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
		writeDebugStreamLine("Condición %d < 5", (armPotVal - armPotInit));
		motor(-127);
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
		writeDebugStreamLine("Condición %d > 2000", (armPotVal - armPotInit));
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
		writeDebugStreamLine("Condición %d < 5", (armPotVal - armPotInit));
		setTower(-127);
		delay(50);
		armPotVal = SensorValue[armPot];
	}
	setTower(0);
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
			displayLCDCenteredString(1, "< Auto Red 1 >");
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
			displayLCDCenteredString(1, "< Auto Red 2 >");
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
			displayLCDCenteredString(1, "< Auto Blue 1 >");
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
			displayLCDCenteredString(1, "< Auto Blue 2 >");
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
			displayLCDCenteredString(1, "< Prog Skill >");
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
			displayLCDCenteredString(1, "< Auto QCC2 >");
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
		displayLCDCenteredString(0, "Auto Red 1");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		//autonomous_red_1();
	}
	else if(auto_selection == 1)
	{
		//If count = 1, run the code correspoinding with choice 2
		displayLCDCenteredString(0, "Auto Red 2");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		//autonomous_red_2();
	}
	else if(auto_selection == 2)
	{
		//If count = 2, run the code correspoinding with choice 3
		displayLCDCenteredString(0, "Auto Blue 1");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		//autonomous_blue_1();
	}
	else if(auto_selection == 3)
	{
		//If count = 3, run the code correspoinding with choice 4
		displayLCDCenteredString(0, "Auto Blue 2");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		//autonomous_blue_2();
	}
	else if(auto_selection == 4)
	{
		displayLCDCenteredString(0, "Prog Skills");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		//prog_skill();
	}
	else if(auto_selection == 5)
	{
		displayLCDCenteredString(0, "No auto");
		wait1Msec(100);
		//No auto
	}
	else if(auto_selection == 6)
	{
		displayLCDCenteredString(0, "Auto QCC2");
		wait1Msec(100);
		//autonomous_qcc2();
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
	displayLCDCenteredString(1, "Dory");
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
	writeDebugStreamLine("encR = %f, encL = %f", SensorValue[encR], SensorValue[encL]);
	delay(200);
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
#endif
