#pragma once

#include <Eigen/Dense>
#include "Robot.h"
#include "msgcontenttype.h"
#include <map>
#include "Util.h"
#include <utility> //for std::pair

class RobotState{
public:
    RobotState();
    RobotState(Robot *);
    void updated(Robot *);
    Eigen::Vector3d EstCtcPosition_Ref(Robot *, myrmex_msg&);
    Eigen::Vector3d EstCtcPosition_KUKAPALM(Robot *, myrmex_msg&);
    Eigen::Vector3d EstCtcPosition_KUKAFINGER();
    Eigen::Vector3d EstRobotEefLVel_Ref(Robot *);
    std::pair<Eigen::Vector3d,Eigen::Vector3d> EstRobotEefRVel_InitF(Robot *, bool&);
    Eigen::Vector3d EstCtcNormalVector_Ref(Robot *, myrmex_msg&);
    Eigen::Vector3d EstRobotEefAcc_Ref(Robot *r);

    //old rebacode endeffector pose
	Eigen::Vector3d position;
    Eigen::Vector3d position_tacfoam;
	Eigen::Vector3d orientation;
	Eigen::Matrix3d eigen_orientation;
	Eigen::VectorXd JntPosition;
    Eigen::Vector3d ctposition;
	bool contactflag;
	double gq;

    //state of whole kinematics chain
    std::map<int, std::string> jntnum2name;
    std::map<std::string, Eigen::Vector3d> robot_position;
    std::map<std::string, Eigen::Matrix3d> robot_orien;
    Eigen::Vector3d eef_vel_g;
    Eigen::Vector3d eef_acc_g;
    Eigen::Vector3d eef_position_lastT;
    Eigen::Vector3d eef_vel_g_lastT;
    Eigen::Vector3d eef_acc_g_lastT;
    Eigen::Matrix3d eef_orientation_lastT;
    Eigen::VectorXd JntPosition_act;
    double JntPosition_mea[7];
    Eigen::Vector3d contact_position;
    Eigen::Vector3d contact_nv;

    //rotation angle rate estimation
    Eigen::Vector3d omega;
    double rate_old;
    double theta_old;


};

class EnvState{
public:
    EnvState(){
        marker_p.setZero();
        marker_o.setZero();
    }
    //marker position
    Eigen::Vector3d marker_p;
    Eigen::Matrix3d marker_o;
};
