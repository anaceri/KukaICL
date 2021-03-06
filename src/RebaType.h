#pragma once


enum StopCondT{
	DEFINED_POSITION = 0,
	OPTIMIZEGP = 1,
	OTHERS = 2
};

enum ActionResultT{
	NOACTIVE = 0,
	RUNNING = 1,
	SUCCESSFUL = 2,
	UNEXPECTED = 3
};

enum RobotNameT{
	kuka_left,
	kuka_right
};

enum MotionDirT{
	UP = 0,
	DOWN = 1,
	BACK = 2,
	FORWARD = 3,
	APPROACH = 4,
	AWAY = 5,
	LEFT = 6,
	RIGHT = 7,
	RXP = 8,
	RXN = 9,
	RYP = 10,
	RYN = 11,
	RZP = 12,
	RZN = 13,
	STOP = 14,
    NOCONTROL = 15,
    VISGUIDE = 16
};

enum ForceCtrlT{
    FTCTRL = 0,
    TACCTRL = 1
};

enum RobotModeT{
    NormalMode = 0,
    PsudoGravityCompensation = 1
};

enum ContactPositionT{
    eff,
    ct,
    eff_tacform
} ;

// class ctrlpara{
// public:
// 	double kp;
// 	double ki;
// 	double kd;
// 	ctrlpara(){kp = 0.0; ki = 0.0; kd = 0.0;};
// };
// 
// class kukapara{
// public:
// 	double axis_stiffness[7];
// 	double axis_damping[7];
// 	kukapara(){
// 		for(int i = 0; i < 7; i++){
// 			axis_damping[i] = 0.2;axis_stiffness[i] = 500;}
// 	}
// };




