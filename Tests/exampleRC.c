#pragma config(Motor,  port1,           gripperRight,  tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port10,          gripperLeft,   tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	while(1)
	{
		if(vexRT(Btn5U))
		{
			motor[gripperLeft] = 127;
			motor[gripperRight] = 127;
		}
		else if(vexRT(Btn6U))
		{
			motor[gripperLeft] = -127;
			motor[gripperRight] = -127;
		}
		else
		{
			motor[gripperLeft] = 0;
			motor[gripperRight] = 0;
		}
	}
}