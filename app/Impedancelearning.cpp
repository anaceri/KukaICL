

/*
 ==============================================================================================
 Name        : ICLCartlmpTest.cpp
 Authors     : Qiang Li & Abdeldjallil naceri
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Human stifness learning experiment using KUKA-lwr and ICL for a visual feedback
 ==============================================================================================
 */


#include <ICLQt/Common.h>
#include <ICLUtils/Random.h>
#include <ICLUtils/FPSLimiter.h>



#include <ICLGeom/Scene.h>
#include <ICLGeom/ComplexCoordinateFrameSceneObject.h>
//#include <ICLUtils/Mutex.h>


#include <VisTool.h>

#include <iostream>
#include <thread>
#include <unistd.h>
#include <termios.h>

#include "ComOkc.h"
#include "KukaLwr.h"
#include "Robot.h"
#include "proactcontroller.h"
#include "kukaselfctrltask.h"
#include "tacservocontroller.h"
#include "tacservotask.h"
//#include "CtrlParam.h"
#include "Timer.h"
#include <fstream>
#include <sstream>
#include "Util.h"
#include "RobotState.h"
#include "Fun.hpp"
 #include "TemporalSmoothingFilter.h"

std::ofstream datafile;
ComOkc *com_okc;
Robot *kuka_lwr;
ActController *ac;
Task *task;
TaskNameT taskname;
ParameterManager* pm;
RobotState *kuka_lwr_rs;

GUI gui;
VisTool *visTool = 0;
Fun shuffel_v;
int counter_t;
Vec filtered_force;
TemporalSmoothingFilter<Vec>* cf_filter;
Eigen::Vector3d f_desired,t_desired;

#ifdef DJALLIL_CONF
#define newP_x -0.3
#define newP_y 0.6
#define newP_z 0.50

#define newO_x 0.0
#define newO_y 0.0
#define newO_z 0.0;
#else
#define newP_x 0.2
#define newP_y 0.3
#define newP_z 0.30
#define newO_x 0.0
#define newO_y -M_PI/2;
#define newO_z 0.0;
#endif

#define SAMPLEFREQUENCE 4

enum session{
    SESSION1 = 0,
    SESSION2 = 1,
    SESSION3 = 2
};

#define trial_s1 40
#define trial_s2 40
#define trial_s3 4

//int numOftrials1[trial_s1];
//int numOftrials2[trial_s2];
//int numOftrials3[trial_s3];
int numOftrials1[trial_s1] = {1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8};

int numOftrials2[trial_s2] = {1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2};

int numOftrials3[trial_s3] = {1,2,1,2};

session sval;
char inp;

RobotModeT rmt;

bool stiffflag,startflag,visflag;

double t_t;
Eigen::VectorXd cp_stiff,cp_damping,extft;

int trialcounter;


void set_stiff_extf(){
    cp_stiff[0] = 0;
    cp_stiff[1] = 200;
    cp_stiff[2] = 2000;
    cp_stiff[3] = 200;
    cp_stiff[4] = 200;
    cp_stiff[5] = 200;
    cp_damping[0] = 0.7;
    cp_damping[1] = 0.7;
    cp_damping[2] = 0.7;
    cp_damping[3] = 0.7;
    cp_damping[4] = 0.7;
    cp_damping[5] = 0.7;

    kuka_lwr->update_robot_cp_stiffness(cp_stiff,cp_damping);
    kuka_lwr->update_robot_cp_exttcpft(extft);
}


Point32f curr(10,30);
Eigen::Vector3d initP;
string pathdata = "../Data/";
string filename = "default";

void start_cb(){
    //    Eigen::VectorXd cp_stiff,cp_damping,extft;
//    com_okc->start_brake();
    //    switch_stiff_cb1();
//    kuka_lwr->switch2cpcontrol();
//    com_okc->release_brake();
    ostringstream FileName;
    filename = pa("-name").as<string>();

    stiffflag = true;
    visflag  = true;
    visTool->setPropertyValue("pos.curr",curr);
    initP.setZero();
    initP = kuka_lwr_rs->robot_position["eef"];
    std::cout<<"session = " << sval <<std::endl;
    switch (sval){
    case SESSION1:
        if(trialcounter>=trial_s1){
            trialcounter = -1;
            std::cout<<"\n"<<std::endl;
            std::cout<<"End of session 1"<<std::endl;
        }
        FileName << pathdata << filename << "_se1_" << trialcounter << ".dat";
        datafile.open(FileName.str().c_str(), ofstream::out);
        counter_t = numOftrials1[trialcounter++];
        std::cout<<"counter = " << counter_t <<std::endl;
        std::cout<<"trial = " << trialcounter <<std::endl;
        break;
    case SESSION2:
        if(trialcounter>=trial_s2){
            trialcounter = -1;
            std::cout<<"\n"<<std::endl;
            std::cout<<"End of session 2"<<std::endl;
        }
        FileName << pathdata << filename << "_se2_" << trialcounter << ".dat";
        datafile.open(FileName.str().c_str(), ofstream::out);
        counter_t = numOftrials2[trialcounter++];
        break;
    }

    startflag = true;
}

void stop_cb(){
    extft[0] = 0;
    extft[1] = 0;
    set_stiff_extf();
    datafile.close();
    startflag = false;
    stiffflag = false;
}

void session1_cb(void){
    trialcounter = 0;
    sval = SESSION1;
    int j = 1;

    // Do random generation routine
    shuffel_v.dealvi(trial_s1,numOftrials1);
    //for (i = 1; i<=trial_s1; i++)
    //  std::cout<< numOftrials1[i] << " " <<std::endl;


}

void session2_cb(){
    trialcounter = 0;
    sval = SESSION2;
    shuffel_v.dealvi(trial_s2,numOftrials2);

}

void session3_cb(void){
    trialcounter = 0;
    sval = SESSION3;
    shuffel_v.dealvi(trial_s3,numOftrials3);

}


void sjnt_cb(void){
    com_okc->start_brake();
    stiffflag = false;
    kuka_lwr->switch2jntcontrol();
    com_okc->release_brake();
}

void calib_cb(void){
    kuka_lwr->calibForce(1000);
}

void run_ctrl(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
        //kuka_lwr->update_robot_stiffness(pm);
        Eigen::Vector3d vel;
        vel.setZero();
        kuka_lwr->get_joint_position_act();
        kuka_lwr->get_joint_position_mea();
        kuka_lwr_rs->updated(kuka_lwr);
        kuka_lwr->update_robot_state();
        vel = kuka_lwr->get_cur_vel();
        if(stiffflag ==true){
            //record data here
            Eigen::Vector3d tmp_p,task_p;
            Eigen::Vector3d f_est,t_est;
            tmp_p.setZero();
            task_p.setZero();
            f_est.setZero();
            t_est.setZero();

	    
	     kuka_lwr->getTcpFtCalib(f_est);
        filtered_force = cf_filter->push(Vec(f_est(0),f_est(1),f_est(2),1));
        //std::cout<<"force are "<<f_est(0)<<","<<f_est(1)<<","<<f_est(2)<<std::endl;
        for(int i = 0; i < 3; i++){
            f_est(i) = filtered_force[i];
//             t_est(i) = 0;
        }
	    
//             kuka_lwr->get_eef_ft(f_est,t_est);
            tmp_p = kuka_lwr_rs->robot_position["eef"]-initP;
            datafile<<tmp_p[0]<< "\t";
            datafile<<tmp_p[1]<< "\t";
            datafile<<tmp_p[2]<< "\t";

            datafile<<f_est[0]<< "\t";
            datafile<<f_est[1]<< "\t";
            datafile<<f_est[2]<< "\t";
            datafile<<t_est[0]<< "\t";
            datafile<<t_est[1]<< "\t";
            datafile<<t_est[2]<< "\t";

            datafile<<extft[0]<< "\t";
            datafile<<extft[1]<< "\t";
            datafile<<extft[2]<< "\t";
            datafile<<t_desired[0]<< "\t";
            datafile<<t_desired[1]<< "\t";
            datafile<<t_desired[2]<< "\t";
            datafile<<counter_t<<std::endl;
            //add session 3 sin perturbation force
            //std::cout<<"session value is "<<sval<<std::endl;
            //std::cout<<"counter value is "<<counter_t<<std::endl;
            switch (sval){
            case SESSION1:
                switch (counter_t){
                case 1:
                    extft[0] = 20*sin(0/4.0);
                    extft[1] = 20*cos(0/4.0);
                    std::cout<<"session 1  1"<<std::endl;
                    break;
                case 2:
                    extft[0] = 20*sin(M_PI/4.0);
                    extft[1] = 20*cos(M_PI/4.0);
                    std::cout<<"session 1  2"<<std::endl;
                    break;
                case 3:
                    extft[0] = 20*sin(2*M_PI/4.0);
                    extft[1] = 20*cos(2*M_PI/4.0);
                    std::cout<<"session 1  3"<<std::endl;
                    break;
                case 4:
                    extft[0] = 20*sin(3.0*M_PI/4.0);
                    extft[1] = 20*cos(3.0*M_PI/4.0);
                    std::cout<<"session 1  4"<<std::endl;
                    break;
                case 5:
                    extft[0] = 20*sin(4.0*M_PI/4.0);
                    extft[1] = 20*cos(4.0*M_PI/4.0);
                    std::cout<<"session 1  5"<<std::endl;
                    break;
                case 6:
                    extft[0] = 20*sin(5.0*M_PI/4.0);
                    extft[1] = 20*cos(5.0*M_PI/4.0);
                    std::cout<<"session 1  6"<<std::endl;
                    break;
                case 7:
                    extft[0] = 20*sin(6.0*M_PI/4.0);
                    extft[1] = 20*cos(6.0*M_PI/4.0);
                    std::cout<<"session 1  7"<<std::endl;
                    break;
                case 8:
                    extft[0] = 20*sin(7.0*M_PI/4.0);
                    extft[1] = 20*cos(7.0*M_PI/4.0);
                    std::cout<<"session 1  8"<<std::endl;
                    break;
                default:
                    extft[0] = 0;
                    extft[1] = 0;
                    break;
                }
                break;
            case SESSION2:
                switch (counter_t){
                case 1:
                    extft[0] = 0;
                    extft[1] = 50;
                    std::cout<<"session 2  1"<<std::endl;
                    break;
                case 2:
                    extft[0] = 0;
                    extft[1] = -50;
                    std::cout<<"session 2  2"<<std::endl;
                    break;
                default:
                    extft[0] = 0;
                    extft[1] = 0;
                    break;
                }
                break;
            }

            if(sval==SESSION3){
                t_t = t_t + 0.01;
                extft[1] = 20*sin(t_t);
                if(t_t>6.28){t_t = 0.0;}
            }

//            std::cout<<"tmp is "<<tmp_p<<std::endl;
//            std::cout<<"tmp norm is "<<tmp_p.norm()<<std::endl;
            if(tmp_p.norm()>0.02){
//                std::cout<<"right position"<<std::endl;
                set_stiff_extf();
            }else{
//                std::cout<<"wong position"<<std::endl;
                extft[0] = 0;
                extft[1] = 0;
                set_stiff_extf();
            }
            std::cout<<"local vel "<<vel(0)<<","<<vel(1)<<","<<vel(2)<<std::endl;
            std::cout<<"del distance "<<(double)pa("-g",0)-curr.x;
            if(kuka_lwr->isTaskFinish(vel,(double)pa("-g",0)-curr.x)){
                stop_cb();
            }

        }
        //using all kinds of controllers to update the reference
        if(task->mt == JOINTS)
            ac->update_robot_reference(kuka_lwr,task);
        ac->llv.setZero();
        ac->lov.setZero();
        //use CBF to compute the desired joint angle rate
        kuka_lwr->update_cbf_controller();
        kuka_lwr->set_joint_command(rmt);
        com_okc->controller_update = true;

    }
}


void run_vis(){
//  if(!pa("-run-test")) {
//    Thread::msleep(1000);
//    return;
//  }
  if(visflag == true){
      Eigen::Vector3d tmp_p;
      tmp_p.setZero();
      tmp_p = kuka_lwr_rs->robot_position["eef"]-initP;
//      std::cout<<"p error"<<tmp_p<<std::endl;
//      std::cout<<"init p"<<initP<<std::endl;
        curr.x = (tmp_p[0])*1000*((double)pa("-g",0)-(double)pa("-s",0)) \
                /(float)gui["tbar"];
        curr.y = (double)pa("-g",1) + (tmp_p[1])*1000*((double)pa("-g",0)-(double)pa("-s",0))/(-2.0) \
                /(float)gui["tbar"];
//        std::cout<<"x,y "<<curr.x<<","<<curr.y<<std::endl;
//      curr.x += 1;
//      curr.y += gaussRandom(0,2);
//      if(curr.y > 80) curr.y *= 0.9;
//      if(curr.y < 20) curr.y *= 1.1;
      visTool->setPropertyValue("pos.curr",curr);
  }
  static FPSLimiter limiter(30);
  limiter.wait();
}

void init(){
//    if(!pa("-no-robot")){}
    sval = SESSION1;
    pm = new ParameterManager("right_arm_param.xml");
    com_okc = new ComOkc(kuka_right,OKC_HOST,OKC_PORT,CART_IMP);
    com_okc->connect();
    kuka_lwr = new KukaLwr(kuka_right,*com_okc);
    kuka_lwr_rs = new RobotState(kuka_lwr);
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = newP_x;
    p(1) = newP_y;
    p(2) = newP_z;

    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    kuka_lwr->setAxisStiffnessDamping(ac->pm.stiff_ctrlpara.axis_stiffness, \
                                      ac->pm.stiff_ctrlpara.axis_damping);
    rmt = NormalMode;
    //tHello.setSingleShot(false);
    //tHello.setInterval(Timer::Interval(SAMPLEFREQUENCE));
    //tHello.start(true);
    stiffflag = false;
    startflag = false;
    t_t = 0.0;
    cp_stiff.setZero(6);
    cp_damping.setZero(6);
    extft.setZero(6);
    trialcounter = 0;
    gui << VBox().handle("box")
        <<(VBox()
	<< Button("ForceCalib").handle("calib_task")
         << Button("SessionI").handle("session1_task")
         << Button("SessionII").handle("session2_task")
         << Button("SessionIII").handle("session3_task")
//         << Button("sJNT").handle("sjnt_task")
         << Button("start").handle("start_task")
         << Button("stop").handle("stop_task")
         << Slider(0,400,400,false).label("bar_move(mm)").handle("tbar")

         )
        <<  Show();
    gui["calib_task"].registerCallback(utils::function(calib_cb));
    gui["session1_task"].registerCallback(utils::function(session1_cb));
    gui["session2_task"].registerCallback(utils::function(session2_cb));
    gui["session3_task"].registerCallback(utils::function(session3_cb));
//    gui["sjnt_task"].registerCallback(utils::function(sjnt_cb));
    gui["start_task"].registerCallback(utils::function(start_cb));
    gui["stop_task"].registerCallback(utils::function(stop_cb));
    visTool = new VisTool;
    BoxHandle box = gui["box"];
    box.add(visTool->getRootWidget(),"myVisTool");

    float maxX = pa("-v",0), maxY = pa("-v",1);

    if(pa("-props")){
      visTool->loadProperties(pa("-props"));
    }

    visTool->setPropertyValue("viewport.maxX",maxX);
    visTool->setPropertyValue("viewport.maxY",maxY);

    visTool->setPropertyValue("pos.left",Point32f(pa("-s",0), pa("-s",1)));
    visTool->setPropertyValue("pos.right",Point32f(pa("-g",0), pa("-g",1)));

    visTool->setPropertyValue("margin",pa("-margin").as<float>());

    visTool->setPropertyValue("sizes.trace",2);
    gui.getRootWidget()->setGeometry(QRect(pa("-geom",0),
                                           pa("-geom",1),
                                           pa("-geom",2),
                                           pa("-geom",3)));
//    if(pa("-record")){
//      visTool->getWidget()->startRecording(*pa("-record",0), *pa("-record",1));
//    }
    std::cout<<"string is "<<pa("-name").as<string>()<<std::endl;
    visflag = false;
    cf_filter = new TemporalSmoothingFilter<Vec>(5,Average,Vec(0,0,0,0));
    f_desired.setZero();
    t_desired.setZero();

}

int main(int n, char **args){
    return ICLApp(n,args,"-viewport|-v(w=100,h=100) "
                  "-start-pos|-s(x=10,y=50) -goal-pos|-g(x=90,y=50) "
                  "-margin|-m(margin=30) -props(xmlfile) -record(device,info) "
                  "-window-geomery|-geom(x=100,y=100,w=800,h=400) "
                  "-name(dd=txt) ",init,run_ctrl,run_vis).exec();
}

