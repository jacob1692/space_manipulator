#ifndef DATADEFINE_H
#define DATADEFINE_H

//=== Windows ===//
//#include <windows.h>
//#include <mmsystem.h>
//=== SpDyn C++ ===//
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/spd.h"
//=== Others ===//
#include "spacedyn_integration/dataflag.h"
#include <math.h>        //

//===================//
//=== Define Data ===//
//===================//

//=== Step time and Simulation time ===//
//const double STEPTIME = 0.0001;	//[s] original value is 0.0001
//const double FINISHTIME = 10;	//[s]

////=== Set Animation's Interval Timers =====================//
////===(30fps=33.33ms, 20fps=50ms, 15fp = 66msec(no good))===//
//const int ANIMATION_INTERVAL = 60;   // [ms]

////=== Change order of input data(Joystick) ===//
//const float CHANGE_ORDER = 0.01;    //

//== tip position's degree of freedom ==//
//#define TIPPOSDEGF 3

////=== MOTOR Parameter ===//
//const float M_GEARRATIO_1 =  158;
//const float M_GEARRATIO_JEM =  1000;
//const float M_INERTIA_1   =  0.4e-4;
//const float M_INERTIA_JEM   =  0.5e-4;
//const float M_STIFFNESS_1 =  200;
//const float M_STIFFNESS_JEM =  20000;
//const float M_DAMPING_1   =  1;
//const float M_DAMPING_JEM   =  1200;

//const double kp = 1.0;
//const double kd = 0.5;

const int e_num = 1; // éËêÊÇÃêî

// time
//const double e_time = 8;  // end time



#endif// DATADEFINE_H
