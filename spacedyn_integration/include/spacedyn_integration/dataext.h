#ifndef DATAEXT_H
#define DATAEXT_H

#include "spacedyn_integration/datadefine.h"

//=== GLOBAL ===//
extern MODEL m;
extern MODEL mt; // model of the target;
extern FLAG flag;
// watanabe
extern double *Gravity;
extern double *chaser_A0;
extern double *target_A0;
extern double *ORI_e;
extern double *POS_e;
//extern double *des_q;
//extern double *des_qd;
//extern double *Q0_rad;
//extern double *Q0_deg;
//extern double *Qe_rad;
//extern double *Qe_deg;
extern double *q_deg;
extern double *q_vel;

extern double d_time;

extern bool flag_ENABLE_HYBCTRL_;
extern bool flag_TARGET_;
    
//!======IMPULSE BASED - VARIABLE IMPEDANCE CONTROL =========================//

extern int _linknum_chaser;

//! Hand Position, Velocity and Acceleration

extern double *xh; //! Linear - Angular Position of the end-effector
extern double *dxh; //! Linear - Angular Velocity of the end effector
extern double *ddxh; //! Linear - Angular Acceleration of the end effector

//! Reference Trajectories

extern double *Nd; //! Reference Position  Attitude with Inertial Frame
extern double *Nvd; //! Reference Velocity Attitude with Inertial Frame 
extern double *Nad; //! Reference Acceleration Attitude Acceleration with respect to the inertial reference frame 

//! Contact caracteristics   

extern double Idn; //! Desired normal direction impulse 
extern double Dampd; //! Desired Damping Ratio (!= Damping Coefficient)
extern double Wdn; //! Desired Angular Frequency

extern double *tDdi;//! Desired Damping Coefficient R6X6
extern double *tKdi; //! Desired Stiffness Coefficient  R6X6
extern double *tMi; //! Desired Impedance Mass R6X6

extern double tmi; //! Desired Impedance Mass measured from target.
extern double tddi;//! Desired Damping Coefficient measured from target.
extern double tkdi; //! Desired Stiffness Coefficient measured from target.

extern bool flag_IN_CONTACT_;
//extern int  iflag_NO_CONTACT_;
extern bool flag_CONTACT0_; //! flag for detecting
 
//! Relative motion
extern double v_0; //! Initial 1D linear relative velocity
extern double y_0; //! Initial 1D linear relative position
	
   //! Compliant Wrist
 
extern double kw; //! stiffness of compliant wrist
extern double dw; //! damping coefficient of compliant wrist


//! Kinematics and Dynamics 

extern double *dxgh; //! Linear - Angular Velocity from the COG to the Hand

extern double *GJe; //! Generalized Jacobian for the end effector
extern double *GJe_prev;
extern double *dGJe; //! Time derivative of Generalized Jacobian

// extern double *tau; //! This is implemented in the model_chaser.def
extern double *Je; //! Jacobian of the end effector
extern double *GH; //! Generalized Inertia Matrix 
extern double *Gc; //! Generalized non-linear velocity term


//! ===== HYBRID VARIABLE IMPEDANCE - POSITION CONTROL======================= //

	 
	//! PID

	extern double *kP;
	extern double *kD;
	extern double *kI; 

	extern double *tRp; //! Contact Point in the target measured in Frame t

	//! Selection matrixes for hybrid control

	extern double *S_imp;
	extern double *S_pos;
    

	//! Reference trajectories
	extern double *tNd; //! Reference Position  Attitude with respect to the target
	extern double *tNvd; //! Reference Velocity  Attitude Velocity with respect to the target
	extern double *tNad; //! Reference Acceleration  Attitude Acceleration with respect to the target 

	extern double *tRI; //! Transformation Matrix 6x6 from Inertia to Target
	extern double *tCI; //! Cosine Direction Matrix from EI to Et
			    //!tRi = [tCI 03x3 ]   tCI: is the cosine direction cosine matrix from EI to Et
			    //!      [03x3 txCI]
	
	//! Kinematics and Dynamics 

	extern double *tdxghI; //! dxgh measured from the target frame of reference
	extern double *tdxghI_prev; //! last measurement of tdxghI
	extern double *tddxghI; //! xgh acceleration measured from the target frame of reference

	extern double *tdxhr; //! Linear - Angular Velocity from the influence of the angular velocity of the target
	extern double *tdxhr_prev; //! Previous Iteration tdxhr
	extern double *tddxhr; //! acceleration for txhr

	extern double *tdxhI; //! Total Linear - Angular Velocity of the hand measured from the target; 
	
	
	extern double *tGJeI; //! Generalized Jacobian for the end effector measured transformed to the target frame of reference.
	extern double *tGJeI_prev; //! Last measurement of tGJeI
	extern double *tdGJeI; //! Time Derivative of tGJeI
	
	extern double *tGHI; //! Generalized Inertia Matrix transformed to the target frame of reference 
	extern double *tGcI; //! Generalized non-linear velocity term measured from the target frame of reference

//! ============ CONTROL WITH INCERTIA UNCERTAINTY =======//
    
    //! Estimation Errors    	
extern double Em; //! Error Ratio of the Mass
extern double EI; //! Error Ratio of Moment of Inertia
extern double ttest; //! time of estimation of intertia


#endif // DATAEXT_H
