#ifndef PROACTCONTROLLER_H
#define PROACTCONTROLLER_H
#include "actcontroller.h"
#include "Robot.h"
#include "kukaselfctrltask.h"


class ProActController : public ActController
{
public:
    ProActController(ParameterManager &p);
    //hold the object in its current pose
    void update_robot_reference(Robot *);
    //move the object to the desired reference designed by Task
    void update_robot_reference(Robot *, Task *);
    void update_robot_reference(Robot *, Task *,myrmex_msg *){}
    void update_controller_para(Eigen::Vector3d&,PROTaskNameT);
    void update_controller_para(std::pair<Eigen::Vector3d,double>&,PROTaskNameT);
    void update_controller_para_stiffness();
    void updateTacServoCtrlParam(TACTaskNameT){}
    void updateProServoCtrlParam(PROTaskNameT tnt);
    void set_pm(ParameterManager &p);
    void get_desired_lv(Robot *, Task *);
    void get_desired_lv(Robot *, Task *, myrmex_msg *){}
    void get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov);
    void set_init_TM(Eigen::Matrix3d tm) {m_init_tm = tm;}
private:
    void get_joint_position();
    void set_eff_command(Eigen::Vector3d p, Eigen::Vector3d o);
    void set_eff_command(Eigen::Vector3d p, Eigen::Matrix3d o);
    void initProServoCtrlParam(PROTaskNameT);
    //!select matrix
    std::map<PROTaskNameT, Eigen::MatrixXd> psm;
    //!pose kp parameter
    std::map<PROTaskNameT, Eigen::MatrixXd> Kpp;
    //!pose kp parameter
    std::map<PROTaskNameT, Eigen::Matrix3d> Kop;
    Eigen::Vector3d llv_pro,lov_pro;
    Eigen::VectorXd lv_pro;
};

#endif // PROACTCONTROLLER_H
