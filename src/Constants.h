/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
enum PS4 {
	Square = 1, //
	Cross = 2, //
	Circle = 3, //
	Triangle = 4, //
	L1 = 5, //
	R1 = 6, //
	L2 = 7, //
	R2 = 8, //
	Share = 9, //
	Options = 10, //
	L3 = 11, //
	R3 = 12, //
	PS = 13, //
	Pad = 14, //
	PSLeftStickRight = 0, //
	PSLeftStickDown = 1, //
	PSRightStickRight = 2, //
	L2In = 3, //start at -1 and goes to 1      (L2In+1)/2 0 to 100%
	R2In = 4,  //start at -1 and goes to 1
	PSRightStickDown = 5  //
//POV up is 0 none is -1
};

enum XB1 { //TODO get XBox mapping
	A = 1, //
	B = 2, //
	X = 3, //
	Y = 4, //
	LB = 5, //
	RB = 6, //
	View = 7, //hi
	Menu = 8, //
	LS = 9, //
	RS = 10, //
	XBLeftStickRight = 0, //
	XBLeftStickDown = 1, //
	LIn = 2, //0 to 1
	RIn = 3, //0 to 1
	XBRightStickRight = 4, //
	XBRightStickDown = 5 //
//POV up is 0 none is -1
};

enum Attack {
	Right = 0, //
	Down = 1, //
	ReverseThrottle = 2 //
};

enum kPDP {
	DBLeft = 13, //
	DBLeft2 = 12, //
	DBRight = 3, //
	DBRight2 = 2, //
	Claw = 14, //
	ClawLeft = 11, //
	ClawRight = 4, //
	Elevator1 = 15, //
	Elevator2 = 0, //
	Elevator3 = 1, //
};
