#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Motor,  mtr_S1_C1_1,     motorD,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motorE,        tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)

#include "JoystickDriver.c"

task main()
{

		int eroare = 5; //erori joystick
		while(true)
{

getJoystickSettings(joystick);

if(abs(joystick.joy1_y2) > eroare) // pentru valoare analog mai mare decat eroarea
{
motor[motorD] = joystick.joy1_y2; // analogul da valoarea motorului D
}
else
{
motor[motorD] = 0; // motor D oprit
}


if(abs(joystick.joy1_y1) > eroare) // daca valoare absoluta analog e mai mare ca eroarea
{
motor[motorE] = joystick.joy1_y1; // motor E primeste valoarea datei analog joystick
}
else
{
motor[motorE] = 0; // motor E e oprit
}

int a = ServoValue[servo1];
int b = ServoValue[servo2];

if(joy1Btn(1)) // daca buton 3 e apasat
{
a = a + 5;
servo[servo1] = a;
}

if (joy1Btn(2))
{
	a = a+ 35;
	servo[servo1] = a;
}

if(joy1Btn(3)) // daca Buton 1 e apasat
{
a = a - 5;
}
servo[servo1] = a;

if (joy1Btn(9))
{
	b = b + 5;
	servo[servo2] = b;
}
if (joy1Btn(10))
{
	b = b - 5;
	servo[servo2] = b;
}

wait1Msec (50);
}
}
