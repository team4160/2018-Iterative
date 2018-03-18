/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

enum PS4 {
	Square = 1,
	Cross = 2,
	Circle = 3,
	Triangle = 4,
	L1 = 5,
	R1 = 6,
	L2 = 7,
	R2 = 8,
	Share = 9,
	Options = 10,
	L3 = 11,
	R3 = 12,
	PS = 13,
	Pad = 14,
	PSLeftStickRight = 0,
	PSLeftStickDown = 1,
	PSRightStickRight = 2,
	PSRightStickDown = 5,
	L2In = 3,
	R2In = 4
};

enum XB1 { //TODO get XBox mapping
	A = 1,
	X = 3,
	B = 3,
	Y = 4,
	LB = 5,
	RB = 6,
	View = 9,
	Menu = 10,
	LS = 11,
	RS = 12,
	XB = 13,
	XBLeftStickRight = 0,
	XBLeftStickDown = 1,
	XBRightStickRight = 2,
	XBRightStickDown = 5,
	LIn = 3,
	RIn = 4
};

enum Attack {
	Up = 1, //TODO find axis numbers
	Right = 2,
	Throttle = 4
};
