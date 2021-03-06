#pragma once

#include <iostream>
#include <deque>
#include <time.h> //for program running test(cpu consuming time)
#include <sys/time.h>//for program running test(realtime consuming test)
#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <utility>

extern Eigen::Vector3d euler2axisangle(Eigen::Vector3d la,Eigen::Matrix3d tm);
extern Eigen::Vector3d g_euler2axisangle(Eigen::Vector3d la,Eigen::Matrix3d tm);
extern Eigen::Vector3d tm2axisangle(Eigen::Matrix3d tm);
extern std::pair<Eigen::Vector3d,double>  tm2axisangle_4(Eigen::Matrix3d,bool&);
extern Eigen::Matrix3d g_euler2tm(Eigen::Vector3d la,Eigen::Matrix3d tm);
extern Eigen::Matrix3d euler2tm(Eigen::Vector3d la,Eigen::Matrix3d tm);
extern double _smooth_filter(std::deque<double> t);
extern Eigen::Matrix3d GetSkrewFromVector(Eigen::Vector3d vec);
extern long long timeval_diff(struct timeval *difference, struct timeval *end_time, struct timeval *start_time);
extern Eigen::Matrix3d AlignVec(Eigen::Vector3d cur, Eigen::Vector3d des);
extern Eigen::Vector3d kdl2eigen_position(const KDL::Frame& f);
extern Eigen::Matrix3d kdl2eigen_orien(const KDL::Frame& f);
extern void global2local(Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d &);
extern std::pair<Eigen::Vector3d,double> omega_transform(std::pair<Eigen::Vector3d,Eigen::Vector3d>,Eigen::Matrix3d);
