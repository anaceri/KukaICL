#pragma once
#include <Eigen/Dense>

class ctrlpara{
public:
    ctrlpara(){kp = 0.0; ki = 0.0; kd = 0.0;}
    double kp;
    double ki;
    double kd;
};

class taskctrlpara{
public:
    ctrlpara p_ctrlpara_x;
    ctrlpara p_ctrlpara_y;
    ctrlpara p_ctrlpara_z;
    ctrlpara p_ctrlpara_roll;
    ctrlpara p_ctrlpara_pitch;
    ctrlpara p_ctrlpara_yaw;
    ctrlpara p_ctrlpara_roll2;
    ctrlpara p_ctrlpara_pitch2;
    ctrlpara p_ctrlpara_yaw2;
    Eigen::MatrixXd kpp;
    Eigen::MatrixXd kpi;
    Eigen::MatrixXd kpd;
    Eigen::MatrixXd kop;
    Eigen::MatrixXd tsm;
    Eigen::MatrixXd ttjkm;
    Eigen::MatrixXd vsm;
    Eigen::MatrixXd vtjkm;
    Eigen::MatrixXd psm;
};

class kukapara{
public:
	double axis_stiffness[7];
	double axis_damping[7];
	kukapara(){
		for(int i = 0; i < 7; i++){
			axis_damping[i] = 0.2;axis_stiffness[i] = 500;}
	}
};

class stiffpara{
public:
    double axis_stiffness[7];
    double axis_damping[7];
    stiffpara(){
        for(int i = 0; i < 7; i++){
            axis_damping[i] = 0.2;axis_stiffness[i] = 500;}
    }
};
