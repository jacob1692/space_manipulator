#include "spacedyn_integration/dataext.h"

MODEL m;
MODEL mt; //! model of the target
FLAG flag;
//
double *Gravity = matrix_get(6,1);
double *chaser_A0 = matrix_get(3,3);
double *target_A0 = matrix_get(3,3);
double *ORI_e = matrix_get(3,3); // [9(=3Å~3)][e_num]
double *POS_e = matrix_get(e_num,3); // [e_num][3]
//double *des_q = matrix_get(8,1); // 8-1 joints; q[0] null
//double *des_qd = matrix_get(8,1);
//double *Q0_rad = matrix_get(3,1); // m.A0 in RPY
//double *Q0_deg = matrix_get(3,1);
//double *Qe_rad = matrix_get(3,1); // m.ORI_e in RPY
//double *Qe_deg = matrix_get(3,1);
double *q_deg = matrix_get(8,1);
double *q_vel = matrix_get(8,1);

//! time

double d_time;

//! flag target

bool flag_TARGET_;

//! FLAG FOR INITIATING THE HYBRID POSITION-IMPEDANCE CONTROL

bool flag_ENABLE_HYBCTRL_; 

//! IMPULSE BASED VARIABLE IMPEDANCE CONTROL

	int _linknum_chaser = 8; //! Used for constructing the global matrixes of the dynamic modelling;

	double *xh  = matrix_get(6,1); //! Linear - Angular Position of the end-effector
	double *dxh = matrix_get(6,1); //! Linear - Angular Velocity of the end effector
	double *ddxh= matrix_get(6,1); //! Linear - Angular Acceleration of the end effector
	
	
	double *Nd = matrix_get(6,1); //! Reference Position  Attitude with Inertial Frame 
	double *Nvd = matrix_get(6,1); //! Reference Velocity Attitude with Inertial Frame 
	double *Nad = matrix_get(6,1); //! Reference Acceleration Attitude Acceleration with respect to the inertial reference frame 

	double Idn;//! Desired Normal Direction Impulse
	double Dampd; //! Desired Damping Ratio
	double Wdn; //! Desired Angular Velocity
	double *tMi=matrix_get(6,6); //! Desired Impedance Mass (R6x6)
	double *tDdi=matrix_get(6,6);//! Desired Impedance Damping Coefficient (R6x6)
	double *tKdi=matrix_get(6,6); //! Desired Impedance Stiffness Coefficient  (R6x6)

	double tmi; //! Desired Impedance Mass
	double tddi;//! Desired Impedance Damping Coefficient
	double tkdi; //! Desired Impedance Stiffness Coefficient 

	//! Contact caracteristics
	bool flag_IN_CONTACT_;
	//int  iflag_NO_CONTACT_;
	bool flag_CONTACT0_;
	//! Relative motion

	double v_0; //! initial 1D relative velocity during contact = relative velocity of the compliant wrist
	double y_0; //! initial 1D relative position during contact = relative displacement of the compliant wrist

	//! Compliant Wrist
	double kw; //! Stiffness of compliant wrist
	double dw; //! Damping coefficient of compliant wrist

	//! Kinematics and dynamics  

	double *dxgh=matrix_get(6,1); //! linear-angular velocity from the CoG of the robot to the hand

	double *GJe=matrix_get(6,_linknum_chaser-1); //! Generalized Jacobian for the end effector [6xLINKNUM-1]
	double *GJe_prev=matrix_get(6,_linknum_chaser-1); //! previous iteration
	double *dGJe=matrix_get(6,_linknum_chaser-1); //! Time Derivative Generalized Jacobian for the end effector [6xLINKNUM-1]

	double *Je=matrix_get(6,_linknum_chaser-1); //! Jacobian matrix for the end effector
	double *GH=matrix_get(_linknum_chaser-1,_linknum_chaser-1); //! Generalized Inertia Matrix
	double *Gc=matrix_get(6,1); //! Generalized non-linear velocity term



//! HYBRID VARIABLE IMPEDANCE - POSITION CONTROL

	
	//! PID 
	double *kP = matrix_get(6,6);  
	double *kD = matrix_get(6,6);
	double *kI = matrix_get(6,6);
	
	double *tRp = matrix_get(3,1);

	//! Selection matrixes for hybrid control
	double *S_imp = matrix_get(6,6);
	double *S_pos = matrix_get(5,6);

	//! Reference trajectories

	double *tNd = matrix_get(6,1); //! Reference Position  Attitude
	double *tNvd = matrix_get(6,1); //! Reference Velocity  Attitude Velocity
	double *tNad = matrix_get(6,1); //! Reference Acceleration  Attitude Accceleration

	double *tRI=matrix_get(6,6);
	double *tCI=matrix_get(3,3);


	//! Kinematics and dynamics  

	double *tdxghI=matrix_get(6,1); //! dxgh measured from the target frame of reference
	double *tdxghI_prev=matrix_get(6,1); //! last measurement of tdxghI
	double *tddxghI=matrix_get(6,1); //! xgh acceleration measured from the target frame of reference 

	double* tdxhr=matrix_get(6,1); //! Linear - Angular Velocity from the influence of the angular velocity of the target
	double* tdxhr_prev=matrix_get(6,1); //! Previous Iteration tdxhr
	double* tddxhr=matrix_get(6,1); //! acceleration for txhr

	double *tdxhI=matrix_get(6,1); //! total linear-angular velocity of the hand measured from the target

	//! Transformation to the target frame of reference

	double *tGJeI=matrix_get(6,_linknum_chaser-1); //! Generalized Jacobian for the end effector measured in the target frame of reference
	double *tGJeI_prev=matrix_get(6,_linknum_chaser-1); //! J* from previous iteration
	double *tdGJeI=matrix_get(6,_linknum_chaser-1); //! d(J*)ij / d_time


	double *tGHI=matrix_get(_linknum_chaser-1,_linknum_chaser-1);  //! Generalized Inertia Matrix measured in the target inertial frame of reference
	double *tGcI=matrix_get(6,1); //! Generalized non-linear velocity term measured from the target reference frame

//! ============ CONTROL WITH INCERTIA UNCERTAINTY =======//

	//! Estimation errors:
	double Em; //! Mass Estimation Error
	double EI; //! Inertia Estimation Error

	double ttest; //! time of estimation of inertia
