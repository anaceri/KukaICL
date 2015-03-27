#ifndef TACSERVOTASK_H
#define TACSERVOTASK_H

#include "task.h"

class TacServoTask : public Task
{
public:
    TacServoTask(TACTaskNameT taskname);
    Eigen::Vector3d get_desired_p_eigen() {return desired_p_eigen;}
    Eigen::Vector3d get_initial_p_eigen() {return initial_p_eigen;}
    Eigen::Matrix3d get_desired_o_eigen(){return desired_o_eigen;}
    Eigen::Vector3d get_desired_o_ax(){return desired_o_ax;}
    void set_desired_p_eigen(Eigen::Vector3d p) {desired_p_eigen =  p;}
    void set_initial_p_eigen(Eigen::Vector3d p) {}
    void set_desired_o_eigen(Eigen::Matrix3d o_eigen){desired_o_eigen = o_eigen;}
    void set_desired_o_ax(Eigen::Vector3d o_ax){desired_o_ax = o_ax;}
    void set_desired_cp_myrmex(double *);
    void set_desired_cf_myrmex(double);
    void set_desired_cf_kuka(double){}
    void get_desired_cp_myrmex(double *cp){cp[0] = desired_cp_myrmex[0];cp[1] = desired_cp_myrmex[1];}
    void get_desired_cf_myrmex(double& cf_myrmex){cf_myrmex = desired_cf_myrmex;}
    void get_desired_cf_kuka(double& cf_kuka){cf_kuka = desired_cf_kuka;}
    void switchtotask(TACTaskNameT taskname);
private:
    double desired_cp_myrmex[2];
    double desired_cf_myrmex;
    double desired_cf_kuka;
};

#endif // TACSERVOTASK_H
