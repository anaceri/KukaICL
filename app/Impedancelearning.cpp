

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
#include <fstream>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>

#include "ComOkc.h"
#include "KukaLwr.h"
#include "Robot.h"
#include "proactcontroller.h"
#include "kukaselfctrltask.h"
#include "tacservocontroller.h"
#include "tacservotask.h"
//#include "CtrlParam.h"
#include "Timer.h"
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

#define BEEP1    0
#define BEEP2    1
#define SOUND_NUMBER  2

#define SAMPLEFREQUENCE 4

enum session{
    SESSION1 = 0,
    SESSION2 = 1,
    SESSION3 = 2,
    SESSION0 = 3
};

#define trial_s0 10
#define trial_s1 40
#define trial_s2 120
#define trial_s3 40
// change this based on the gender: male=1000, female=500
#define beta 1000
#define forceDisp 40
#define deltatPert 0.6

//int numOftrials1[trial_s1];
//int numOftrials2[trial_s2];
//int numOftrials3[trial_s3];
int numOftrials1[trial_s1] = {1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8};

//int numOftrials2[trial_s2] = {1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2};

int numOftrials3[trial_s3] = {1,2,3,4,5,6,7,8,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9};

session sval;
char inp;

RobotModeT rmt;

bool stiffflag,startflag, visflag;

double t_t;
Eigen::VectorXd cp_stiff,cp_damping,extft;

int trialcounter;
float pertcounter;
float bipcounter;
float randval;
bool startAdd;
bool bipflag;

//choose which axis you wanna investigate!
int indx;
int indy;
int indz;

// audio function
Mix_Chunk *sounds[SOUND_NUMBER];
void playSound( Mix_Chunk *sound ) {

      if (sound == NULL)
      {
        printf("Error: No sound\n");
        return;
      }
      int channel = Mix_PlayChannel(-1, sound, 0);
}

void set_stiff_extf(){
    cp_stiff[0] = 0;
    cp_stiff[1] = 200;
    cp_stiff[2] = 200;
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


Point32f curr(10,0);
Point32f currPause(0,0);
Eigen::Vector3d initP;int j = 1;

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
    visflag   = true;
    startflag = true;
    if(!visflag){std::cout << "alo mais c alo ..." << std::endl;}
    visTool->setPropertyValue("pos.curr",curr);
    initP.setZero();
    initP = kuka_lwr_rs->robot_position["eef"];
    randval   = rand() % 3 + 2;
    randval   = randval * 0.1;
    pertcounter = 0;
    startAdd = false;
    bipflag = true;

    switch (sval){
    case SESSION0:
        if(trialcounter>=trial_s0){
            trialcounter = -1;
            std::cout<<"\n"<<std::endl;
            std::cout<<"End of session 0"<<std::endl;
        }
        FileName << pathdata << filename << "_se0_" << trialcounter << ".dat";
        datafile.open(FileName.str().c_str(), ofstream::out);
        counter_t = numOftrials1[trialcounter++];
        std::cout<<"trial = " << trialcounter <<std::endl;
        break;
    case SESSION1:
        if(trialcounter>=trial_s1){
            trialcounter = -1;
            std::cout<<"\n"<<std::endl;
            std::cout<<"End of session 1"<<std::endl;
        }
        FileName << pathdata << filename << "_se1_" << trialcounter << ".dat";
        datafile.open(FileName.str().c_str(), ofstream::out);
        counter_t = numOftrials1[trialcounter++];
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
        counter_t = 1;
        trialcounter++;
        std::cout<<"trial = " << trialcounter <<std::endl;
        break;
    case SESSION3:
        if(trialcounter>=trial_s3){
            trialcounter = -1;
            std::cout<<"\n"<<std::endl;
            std::cout<<"End of session 3"<<std::endl;
        }
        FileName << pathdata << filename << "_se3_" << trialcounter << ".dat";
        datafile.open(FileName.str().c_str(), ofstream::out);
        counter_t = numOftrials3[trialcounter++];
        //std::cout<<"count_t is "<<counter_t<<std::endl;
        std::cout<<"trial = " << trialcounter <<std::endl;
        //std::cout<<"sval = " << sval <<std::endl;
        break;
    default:
        trialcounter = -1;
    }

}

void stop_cb(){

    extft[0] = 0;
    extft[1] = 0;
    extft[2] = 0;
    set_stiff_extf();
    datafile.close();
    startflag = false;
    stiffflag = false;
    visflag   = false;
    //std::cout << "stopped properly" << std::endl;
    visTool->setPropertyValue("pos.curr",curr);
    curr.x = 100;
}

void session0_cb(void){
    trialcounter = 0;
    sval = SESSION0;
    std::cout<<"callback session 0"<<std::endl;
    // only movement
}

void session1_cb(void){
    trialcounter = 0;
    sval = SESSION1;
    int j = 1;

    // Do random generation routine
    shuffel_v.dealvi(trial_s1,numOftrials1);
    //for (i = 1; i<=trial_s1; i++)
    //std::cout<< numOftrials1[i] << " " <<std::endl;


}

void session2_cb(){
    trialcounter = 0;
    sval = SESSION2;
    //shuffel_v.dealvi(trial_s2,numOftrials2);

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
//        std::cout<<"vel is "<<vel<<std::endl;
        if(stiffflag ==true){
            //record data here
            Eigen::Vector3d tmp_p,task_p;
            Eigen::Vector3d f_est,t_est;
            tmp_p.setZero();
            task_p.setZero();
            f_est.setZero();
            t_est.setZero();

//            std::cout<<"force are before filtering "<<f_est(0)<<","<<f_est(1)<<","<<f_est(2)<<std::endl;
            kuka_lwr->getTcpFtCalib(f_est);
            filtered_force = cf_filter->push(Vec(f_est(0),f_est(1),f_est(2),1));
//            std::cout<<"force are "<<f_est(0)<<","<<f_est(1)<<","<<f_est(2)<<std::endl;
            for(int i = 0; i < 3; i++){
                f_est(i) = filtered_force[i];
                //             t_est(i) = 0;
            }
//            std::cout<<"force are new "<<f_est(0)<<","<<f_est(1)<<","<<f_est(2)<<std::endl;

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
            //std::cout<<"session value in run is "<<sval<<std::endl;
            //std::cout<<"counter value in run is "<<counter_t<<std::endl;
            switch (sval){
            case SESSION0:
                if ((bipflag) && (startAdd)) {
                    //play bip audio
                    playSound(sounds[BEEP1]);
                    bipcounter = 0;
                    bipflag = false;
                }
                //std::cout<<"bipcounter = " << bipcounter <<std::endl;
                //std::cout << "vel1 = " << vel[0]<<","<< vel[1]<<","<< vel[2]<<std::endl;
                //std::cout<<"bipcounter = " << bipcounter <<std::endl;
                if((bipcounter >= 0.6) && (bipcounter < 0.61)){
                    //play bip audio
                    playSound(sounds[BEEP2]);
                }
                break;

            case SESSION1:
                if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                    //std::cout<<"randval = " << randval <<std::endl;
                    switch (counter_t){
                    case 1:
                        extft[indy] = forceDisp*sin(0/4.0);
                        extft[indz] = forceDisp*cos(0/4.0);
                        //std::cout<<"session 1  1"<<std::endl;
                        break;
                    case 2:
                        extft[indy] = forceDisp*sin(M_PI/4.0);
                        extft[indz] = forceDisp*cos(M_PI/4.0);
                        //std::cout<<"session 1  2"<<std::endl;
                        break;
                    case 3:
                        extft[indy] = forceDisp*sin(2*M_PI/4.0);
                        extft[indz] = forceDisp*cos(2*M_PI/4.0);
                        //std::cout<<"session 1  3"<<std::endl;
                        break;
                    case 4:
                        extft[indy] = forceDisp*sin(3.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(3.0*M_PI/4.0);
                        //std::cout<<"session 1  4"<<std::endl;
                        break;
                    case 5:
                        extft[indy] = forceDisp*sin(4.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(4.0*M_PI/4.0);
                        //std::cout<<"session 1  5"<<std::endl;
                        break;
                    case 6:
                        extft[indy] = forceDisp*sin(5.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(5.0*M_PI/4.0);
                        //std::cout<<"session 1  6"<<std::endl;
                        break;
                    case 7:
                        extft[indy] = forceDisp*sin(6.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(6.0*M_PI/4.0);
                        //std::cout<<"session 1  7"<<std::endl;
                        break;
                    case 8:
                        extft[indy] = forceDisp*sin(7.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(7.0*M_PI/4.0);
                        //std::cout<<"session 1  8"<<std::endl;
                        break;
                    default:
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                        break;
                    }
                } else {
                    extft[indx] = 0;
                    extft[indy] = 0;
                    extft[indz] = 0;
                }
                break;
            case SESSION2:
                switch (counter_t){
                case 1:
                    extft[indx] = 0;
                    extft[indz] = 0;
                    if ((abs(tmp_p[indy]) <= 0.1) && (abs(tmp_p[indz]) <= 0.1)){
                        extft[indy] = beta * tmp_p[indy];
                        //extft[indz] = beta * tmp_p[indz];
                    }
                    else{
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    //std::cout<<"err = " << extft[1] <<std::endl;
                    break;
                case 2:
                    extft[0] = 0;
                    if (abs(tmp_p[1]) <= 0.1)
                        extft[indy] = beta * tmp_p[1];
                    else
                        extft[indy] = 0;
                    //std::cout<<"err = " <<  extft[1] <<std::endl;
                    break;
                default:
                    extft[indx] = 0;
                    extft[indy] = 0;
                    extft[indz] = 0;
                    break;
                }
                break;
            case SESSION3:
//                std::cout <<  "conterout = " << counter_t << std::endl;
                switch (counter_t){
//                std::cout <<  "conterin = " << counter_t << std::endl;
                extft[indx] = 0;
                case 1:
                    if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                        extft[indy] = forceDisp*sin(0/4.0);
                        extft[indz] = forceDisp*cos(0/4.0);
                        //std::cout<<"session 3  1"<<std::endl;
                    } else {
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                case 2:
                    if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                        extft[indy] = forceDisp*sin(M_PI/4.0);
                        extft[indz] = forceDisp*cos(M_PI/4.0);
                        //std::cout<<"session 3  2"<<std::endl;
                    } else {
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                case 3:
                    if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                        extft[indy] = forceDisp*sin(2*M_PI/4.0);
                        extft[indz] = forceDisp*cos(2*M_PI/4.0);
                        //std::cout<<"session 3  3"<<std::endl;
                    } else {
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                case 4:
                    if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                        extft[indy] = forceDisp*sin(3.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(3.0*M_PI/4.0);
                        //std::cout<<"session 3  4"<<std::endl;
                    } else {
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                case 5:
                    if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                        extft[indy] = forceDisp*sin(4.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(4.0*M_PI/4.0);
                        //std::cout<<"session 3  5"<<std::endl;
                    } else {
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                case 6:
                    if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                        extft[indy] = forceDisp*sin(5.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(5.0*M_PI/4.0);
                        //std::cout<<"session 3  6"<<std::endl;
                    } else {
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                case 7:
                    if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                        extft[indy] = forceDisp*sin(6.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(6.0*M_PI/4.0);
                        //std::cout<<"session 3  7"<<std::endl;
                    } else {
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                case 8:
                    if ((pertcounter > randval) && (pertcounter < randval + deltatPert )){
                        extft[indy] = forceDisp*sin(7.0*M_PI/4.0);
                        extft[indz] = forceDisp*cos(7.0*M_PI/4.0);
                        //std::cout<<"session 3  8"<<std::endl;
                    } else {
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                case 9:
                    extft[indx] = 0;
                    extft[indz] = 0;
                    if ((abs(tmp_p[indy]) <= 0.1) && (abs(tmp_p[indz]) <= 0.1)){
                        extft[indy] = beta * tmp_p[indy];
                        //extft[indz] = beta * tmp_p[indz];
                    }
                    else{
                        extft[indx] = 0;
                        extft[indy] = 0;
                        extft[indz] = 0;
                    }
                    break;
                default:
                    extft[indx] = 0;
                    extft[indy] = 0;
                    extft[indz] = 0;
                    break;
                }
//                std::cout<<"now in session 3 branch"<<std::endl;
                break;
            default:
                std::cout<<"this is the task default case"<<std::endl;
            }
//            std::cout<<"now just outside of session 3 branch"<<std::endl;
            /*if(sval==SESSION3){
                t_t = t_t + 0.01;
                extft[1] = 20*sin(t_t);
                if(t_t>6.28){t_t = 0.0;}
            }*/

//                        std::cout<<"tmp is "<<tmp_p<<std::endl;
//                        std::cout<<"tmp norm is "<<tmp_p.norm()<<std::endl;
            if(tmp_p.norm()>0.02){
                //                std::cout<<"right position"<<std::endl;
                set_stiff_extf();
            }else{
                //                std::cout<<"wong position"<<std::endl;
                extft[0] = 0;
                extft[1] = 0;
                extft[2] = 0;
                set_stiff_extf();
            }
//            std::cout<<"local vel "<<vel(0)<<","<<vel(1)<<","<<vel(2)<<std::endl;
            //std::cout<<"del distance "<<(double)pa("-g",0)-curr.x;
            if((kuka_lwr->isTaskStart(vel,(double)pa("-s",0)-curr.x))){
                startAdd = true;
            }

            if(startAdd){
                pertcounter = pertcounter + kuka_lwr->gettimecycle();
                bipcounter  = bipcounter  + kuka_lwr->gettimecycle();
            }

            if((kuka_lwr->isTaskFinish(vel,(double)pa("-g",0)-curr.x)) || (abs(tmp_p[1]) > 0.1)){
                stop_cb();
            }

            if((vel.norm() < 0.4) || (vel.norm() > 0.8))
                visTool->setPropertyValue("colors.curr",Color(200,0,0));
            else
                visTool->setPropertyValue("colors.curr",Color(0,200,20));
        }
        else{

            //visTool->setPropertyValue("colors.curr",Color(0,0,0));
            //visTool->setPropertyValue("pos.curr",currPause);
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
    static float angle = 90;
    VisualizationDescription d;
    d.color(0,100,255,255);
    d.textangle(90);
    d.fontsize(35);
    d.text(900,10,"Trial = " + str(trialcounter));

    if(trialcounter==0){
        d.fontsize(40);
        d.textangle(++angle);
        d.text(500,150,"End of session!");
    }

    visTool->addCustomVisualization(d);
    if(visflag == true){
        Eigen::Vector3d tmp_p;
        tmp_p.setZero();
        tmp_p = kuka_lwr_rs->robot_position["eef"]-initP;
        //std::cout<<"p error"<<tmp_p<<std::endl;
        //std::cout<<"init p"<<initP<<std::endl;
        curr.x = (tmp_p[0])*1000*((double)pa("-g",0)-(double)pa("-s",0)) \
                /(float)gui["tbar"];
        curr.y = (double)pa("-g",1) + (tmp_p[1])*1000*((double)pa("-g",0)-(double)pa("-s",0))/(-2.0) \
                /(float)gui["tbar"];
        //      std::cout<<"x,y "<<curr.x<<","<<curr.y<<std::endl;
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
    indx = 0;
    indy = 1;
    indz = 2;
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
    pertcounter = 0;
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
           << Button("Session0").handle("session0_task")
           << Button("SessionI").handle("session1_task")
           << Button("SessionII").handle("session2_task")
           << Button("SessionIII").handle("session3_task")
           //         << Button("sJNT").handle("sjnt_task")
           << Button("start").handle("start_task")
           << Button("stop").handle("stop_task")
           << Slider(0,320,320,false).label("bar_move(mm)").handle("tbar")

           )
       <<  Show();
    gui["calib_task"].registerCallback(utils::function(calib_cb));
    gui["session0_task"].registerCallback(utils::function(session0_cb));
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

    // Init variables for the audio

    int audio_rate = 22050;		//Frequency of audio playback
    Uint16 audio_format = AUDIO_S16SYS; 	//Format of the audio we're playing
    int audio_channels = 1;		  //2 channels = stereo
    int audio_buffers = 4096;		//Size of the audio buffers in memory
    if( Mix_OpenAudio(audio_rate, audio_format, audio_channels, audio_buffers)!= 0 )
    {
        printf( "Unable to initialize audio: %s\n", Mix_GetError());
        exit(1);
    }
    // Load sounds
    string pathsound = "../sound/";
    ostringstream  path;
    path.str("");
    path << pathsound << "beep-07.wav";
    sounds[BEEP1] = Mix_LoadWAV(path.str().c_str());
    path.str("");
    path << pathsound << "beep-08b.wav";
    sounds[BEEP2] = Mix_LoadWAV(path.str().c_str());
    path.str("");

}

int main(int n, char **args){
    return ICLApp(n,args,"-viewport|-v(w=100,h=100) "
                  "-start-pos|-s(x=10,y=50) -goal-pos|-g(x=90,y=50) "
                  "-margin|-m(margin=30) -props(xmlfile) -record(device,info) "
                  "-window-geomery|-geom(x=100,y=100,w=800,h=400) "
                  "-name(dd=txt) ",init,run_ctrl,run_vis).exec();
}

