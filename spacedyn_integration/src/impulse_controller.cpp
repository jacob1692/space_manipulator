#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include <stdlib.h>
#include <ros/package.h>
#include "spacedyn_integration/chaser_controller.h"
#include <vector>

#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

namespace gazebo {

		
	void GazeboRosSPDChaserController::ImpParamLoad(sdf::ElementPtr _sdf)
	{
		//! COUT THE KW AND DW RETRIEVED FROM THE COMPLIANT WRIST PLUGIN
		
		ROS_INFO("Stiffness of the Compliant Wrist : %f ", kw); //! See dataext.cpp
		ROS_INFO("Damping of the Compliant Wrist: %f", dw);
		
		
		//! kP Parsing
		std::stringstream stream;
		if (_sdf->HasElement("kP")){   //! Extract kP from SDF e.g diag ("_4000_0_0_0_0_4000_" )
		//	cout<<"DEBUG: I found kP"<<endl;
			std::string kP_s;
			_sdf->GetElement("kP")->GetValue()->Get<std::string>(kP_s);	 //! Retrieve the string
			stream.str(kP_s.c_str());  //! turn the string into a stream
			if( !stream )
			{
			  cerr << "Cannot read kP diagonal" << endl; //! In case the conversion didn't happen
			  //exit(1);
			}
			
		//	ROS_WARN("this is KP diagonal, %s",kP_s.c_str());
			for (unsigned int i=0;i<6;i++){
				stream >> kP[0+7*i];  //! parse the element of the string into the array element
				//cout<<kP[0+7*i]<<endl;
			}
		 }
		 
		else{
			kP[0]=4000;kP[0+7*5]=4000; //!diag(4000 0 0 0 0 4000)
			}
		 
		stream=std::stringstream();
		//! kD Parsing
		
		if (_sdf->HasElement("kD")){    //! Extract kP from SDF e.g diag ("_100_0_0_0_0_100_" )
			std::string kD_s;
			_sdf->GetElement("kD")->GetValue()->Get<std::string>(kD_s);	
			stream.str(kD_s.c_str());
			if( !stream )
			{
			  cerr << "Cannot read kD diagonal" << endl;
			  //exit(1);
			}
			for (unsigned int i=0;i<6;++i){
				stream >> kD[0+7*i];
			}
		 }
		 else{
			kD[0]=100;kD[0+7*5]=100; //!diag(100 0 0 0 0 100)
			}
		stream=std::stringstream();
		
		//! kI[0][0]=0;kI[5][5]=0; //!diag(0 0 0 0 0 0)
		
		//! kI Parsing
		
		if (_sdf->HasElement("kI")){    //! Extract kI from SDF e.g diag ("_0_0_0_0_0_0_" )
			std::string kI_s;
			_sdf->GetElement("kI")->GetValue()->Get<std::string>(kI_s);	
			stream.str(kI_s.c_str());
			if( !stream )
			{
			  cerr << "Cannot read kI diagonal" << endl;
			  //exit(1);
			}
			for (unsigned int i=0;i<6;++i){ 
				stream >> kI[0+7*i];
			}
		 }
		stream=std::stringstream();
		//! else{
			//! kI[0][0]=0;kI[5][5]=0; //!diag(0 0 0 0 0 0)
		//!	}
		//! Damping Ratio Parsing
		if (_sdf->HasElement("dampD")){   //! Extract Dampd from SDF e.g "1.0"
			_sdf->GetElement("dampD")->GetValue()->Get(Dampd);	
		 }
		else {
			Dampd=1.0;
		}
		//! Desired Normal Impulse Parsing
		if (_sdf->HasElement("ImpDN")){   //! Extract Idn from SDF e.g "0.1"
			_sdf->GetElement("ImpDN")->GetValue()->Get(Idn);	
		 }
		else{
			Idn=0.1;
		}
		
		//! Desired Impedance Mass
		if (_sdf->HasElement("tmi")){   //! Desired Impedance Mass
			_sdf->GetElement("tmi")->GetValue()->Get(tmi);	
		 }
		else{ 
			tmi=200;
			}
		for (unsigned int i = 0; i<3; ++i){
		tMi[0+7*i]=tmi;     
		
		//!tMi=[tmi 0   0   0 0 0];
		//!        0  tmi 0   0 0 0]
		//!        0   0  tmi 0 0 0]
		//!        0   0  0   0 0 0]  
		//!        0   0  0   0 0 0]  
		//!        0   0  0   0 0 0]  

		
		}
		
		//tddi//! TO BE DETERMINED BY THE ALGORITHM!!!
		//		tDdi[0][0]=tddi;     //!tDixx=200;
		
		//tkdi//! TO BE DETERMINED BY THE ALGORITHM!!!
		//		tKdi[0][0]=tkdi;
		
		
		//! ========================== HYBRID CONTROL ============================//
		
		//! Set the control selection matrixes
		S_imp[0]=1; //! Si=[1 0 0 0 0 0 ]
		for (int i=0;i<5;++i){
			S_pos[1+(6+1)*i]=1; 
		//! Sp= 0 1 0 0 0 0; 
		//!     0 0 1 0 0 0; 
		//!     0 0 0 1 0 0;
		//!     0 0 0 0 1 0;
		//!     0 0 0 0 0 1
		}
		
		//! tNd Parsing
		
		
		
		if (_sdf->HasElement("tNd")){   //! Extract tNd from SDF
			std::string tNd_s;
			_sdf->GetElement("tNd")->GetValue()->Get<std::string>(tNd_s);	
			stream.str(tNd_s.c_str()); 
			if( !stream )
			{
			  cerr << "Cannot read tNd" << endl;
			  //exit(1);
			}
			for (unsigned int i=0;i<6;++i){ //! "_-0.16_0_0_0_0_0_"
				stream >> tNd[i];
			}
		 }
		 else{
			 tNd[0]=-0.16;
			 }
		stream=std::stringstream();
		
		
		//! tNvd Parsing
		
		//! tNd[0]=0.0; tNd[1]=0.0; tNd[2]=0.0; tNd[3]=0.0; tNd[4]=0.0; tNd[5]=0.0;
		
		if (_sdf->HasElement("tNvd")){   //! Extract tNvd from SDF
			std::string tNvd_s;
			_sdf->GetElement("tNvd")->GetValue()->Get<std::string>(tNvd_s);	
			stream.str(tNvd_s.c_str());
			if( !stream )
			{
			  cerr << "Cannot read tNvd" << endl;
			  //exit(1);
			}
			for (unsigned int i=0;i<6;++i){ //! "_0_0_0_0_0_0_"
				stream >> tNvd[i];
			}
		 }
		stream=std::stringstream();
		 
		 //! tNad Parsing
		
		//! tNvd[0]=0.0; tNvd[1]=0.0; tNvd[2]=0.0; tNvd[3]=0.0; tNvd[4]=0.0; tNvd[5]=0.0;
		
		if (_sdf->HasElement("tNad")){   //! Extract tNad from SDF
			std::string tNad_s;
			_sdf->GetElement("tNad")->GetValue()->Get<std::string>(tNad_s);	
			stream.str(tNad_s.c_str());
			if( !stream )
			{
			  cerr << "Cannot read tNad" << endl;
			  //exit(1);
			}
			for (unsigned int i=0;i<6;++i){ //! "_0_0_0_0_0_0_"
				stream >> tNad[i];
			}
		 }
		stream=std::stringstream();
		 //! tRp Parsing
		
		if (_sdf->HasElement("tRp")){   //! Extract tRp from SDF
			std::string tRp_s;
			_sdf->GetElement("tRp")->GetValue()->Get<std::string>(tRp_s);	
			stream.str(tRp_s.c_str());
			if( !stream )
			{
			  cerr << "Cannot read tRp" << endl;
			  //exit(1);
			}
			for (unsigned int i=0;i<3;++i){ //! "_-0.15_0_0 "
				stream >> tRp[i];
			}
		 }
		 else{
			tRp[0]=-0.15; tRp[1]=0.0; tRp[2]=0.0; 
		}
		stream=std::stringstream();
		
		//! Mass estimation error
		if (_sdf->HasElement("Em")){  
			_sdf->GetElement("Em")->GetValue()->Get(Em);	
		 }
		else{ 
			Em=0.5; 
			}
			
		//! Inertia estimation error
		if (_sdf->HasElement("EI")){  
			_sdf->GetElement("EI")->GetValue()->Get(EI);	
		 }
		else{ 
			EI=0.5;
			}
		
		//! Time of estimation of Inertia
		if (_sdf->HasElement("ttest")){  
			_sdf->GetElement("ttest")->GetValue()->Get(ttest);	
		 }
		else{ 
			ttest=0.05; //![s]
			}
		
	////DEBUG matrix_print(5,6,S_pos);
		//matrix_print(6,6,kP);
		//matErix_print(6,6,kD);
		//matrix_print(6,6,kI);
		//cout<<"dampD: "<<Dampd<<endl;
		//cout<<"ImpDN: "<<Idn<<endl;
		//cout<<"tmi: "<<tmi<<endl;
		//vector_print(6,tNd);
		//vector_print(6,tNvd);
		//vector_print(6,tNad);
		//vector_print(3,tRp);
		//cout<<"Em: "<<Em<<endl;
		//cout<<"EI: "<<EI<<endl;
		//cout<<"ttest: "<<ttest<<endl;
		//}
	}		
	
	void GazeboRosSPDChaserController::ImpParamCalc(double _Idn, double _dampD, double _tv0, double _ty0, double _tmi,  double _kw, double _dw, double& _Wdn, double& _tddi, double& _tkdi) 
	//! Determination of impedance parameters based on DampdD(Desired Damping Ratio)  tV0(relative velocity measure from target) tY0 (relative position measured from target) and tmi (desired impedance mass 1D; tmi> kw/wdn2,dw/2*DampdD*wdn)  
	{	
		
		//! Desired Normal Direction Impulse: _Idn.
		//! Desired Damping Ratio: _dampD.
		
		
		if (_tv0>=0){
			//! Checking of the Desired Damping Ratio
			if (_dampD<1){
				 ROS_INFO("The desired damping ratio should be greater or equal to 1, changed to 1 ");
				_dampD=1.0;
				}
			//! Desired Angular Frenquency
			double sqr1_temp = _kw * ( std::pow(_kw,2) ) * ( std::pow(_ty0,2) ) + _kw * _tv0 * ( _dw * _ty0 + _dampD );//! Avoid Square Root of Negative Numbers 
			if (sqr1_temp>=0) {
				_Wdn= ( _kw*_dampD*_ty0 + std::sqrt ( sqr1_temp ))/ ( _dw*_ty0 + _Idn );
			}
			else {
				ROS_ERROR("ERROR: negative square root in _Wdn with _v0 >= 0");
				_Wdn=0.0;
				}
		}
		
		if (_tv0<0){
			double sqrtest= - ( _tv0 / ( _kw * std::pow( _ty0,2 ) ) ) * ( _dw * _ty0 + _Idn );
			if (sqrtest>=0) {
				double dampTemp = std::sqrt ( sqrtest );
				//! Checking of the Desired Damping Ratio
				if (_dampD<dampTemp){ //! Eq. 57 of the English Text
					 ROS_INFO("The desired damping ratio should be greater or equal to  %f , changed to %f ", dampTemp, dampTemp);
					_dampD=dampTemp;
				}
			}
			else
				ROS_ERROR("ERROR: negative square root in damping ratio with tv0<0");
			
			
			//! Desired Angular Frenquency
			double sqr2_temp = _kw*( std::pow(_kw,2) ) * ( std::pow(_ty0,2) ) + _kw*_tv0* ( _dw * _ty0 + _dampD );
			ROS_INFO("sqrt for wdn: %f ",sqr2_temp);
			if (sqr2_temp>=0){
				_Wdn= ( _kw *_dampD *_ty0 + std::sqrt ( sqr2_temp ) ) / ( _dw * _ty0 + _dampD ); 
				if (_Wdn <= 0){
					_Wdn= ( _kw * _dampD * _ty0 - std::sqrt ( sqr2_temp ) ) / ( _dw*_ty0 + _dampD ); 
				}
			}
			else {
				ROS_ERROR("ERROR: negative square root in _Wdn with _v0 < 0");
				_Wdn=0.0;
			}
		}
		
		if (_Wdn < 0){
			ROS_ERROR("Attention: _Wdn is negative! check your implementation");
		}
		
		//! Desired Impedance Mass: _tmi
		
		//! Checking of the Desired Impedance Mass
			
			//! TBD
			
		//! Desired Damping and Stiffness Coefficient
		
		_tddi= 2 * _dampD * _tmi * _Wdn - _dw;
		_tkdi= _tmi * std::pow ( _Wdn,2 ) - _kw;
		
		for (unsigned int i = 0; i<3; ++i){
			tDdi[0+7*i]=tddi;
			tKdi[0+7*i]=tkdi;     
		}
		//tDdi[0]=_tddi; //! Pos X in Global R6x6 viscous damping coefficient
		//tKdi[0]=_tkdi; //! Pos X in Global R6x6 stiffness coefficient
	}
	
	void GazeboRosSPDChaserController::ImpulseController(MODEL& m_, MODEL& mt_, physics::ModelPtr _model)
	{
		static int times=0; //! flag to make parameter calculation only at the beginning of the contact
		static int times2=0;
		//! Example of multiplication:  matrix_mult( 6, 6, 6, m.Xup_I[i], m.jXc[i], Xc_I );
		if (flag_CONTACT0_){ //! TO REMEMBER: CHANGE TO THE POINT OF THE ALGORITHM WHERE THE CONTACT STARTS
			
			//! Set the position, velocity and acceleration as reference the target at the begining of the contact as the reference 
			
			// matrix_cpy(6,1,xh,); 
			
			if (times==0){ // !
				v_0= - this->d1_->GetVelocity(9);//! Velocity of the compliant wrist
				y_0= - this->d1_->GetAngle(9).Radian(); //! Position of the 
				ROS_INFO("velocity of contact: %f", v_0);
				ROS_INFO("relative position of contact: %f", y_0);
				GazeboRosSPDChaserController::ImpParamCalc(Idn,Dampd,v_0,y_0,tmi,kw,dw,Wdn,tddi,tkdi); //! Determination of _tddi and _tdki
				times=1;
			}	
			//! Calculate the linear-angular velocity from the CoG of the robot to the hand
			calc_dxgh(m,dxgh);
			//! Calculating the Generalized Jacobian
			calc_GJe(m,1,GJe);//! Calculation of the Generalized Jacobian Matrix for the End Effector
			
			//! Calculating the generalized inertia matrix
			calc_GH(m,GH);		
			//! Calculating the generalized non_linear velocity term
			calc_Gc(m,Gravity,Gc);
			if (times2==0){
				ROS_INFO("The generalized jacobian matrix is \n");
				matrix_print(6,m.LINKNUM-1,GJe);
				ROS_INFO("\n The generalized inertia matrix is \n");
				matrix_print(m.LINKNUM-1,m.LINKNUM-1,GH);
				times2=1;
				}
			
			//! Calculating the J*^-1 pseudoinverse and transpose J*'
			double *GJe_inv = matrix_get(_linknum_chaser-1,6);
			double *GJe_trans= matrix_get(_linknum_chaser-1,6);
			double *GJe_devt= matrix_get(6,_linknum_chaser-1);
			
			matrix_pinv(6,_linknum_chaser-1,GJe,GJe_inv); //! J*^-1
			matrix_trans(6,_linknum_chaser-1,GJe,GJe_trans); //! J*'
			matrix_scale(_linknum_chaser-1, 6,-1,GJe_trans,GJe_trans);//! -J*'
			matrix_devt(6,_linknum_chaser-1,GJe,GJe_prev,d_time,GJe_devt); //! dJ*/dt
			
			//! Calculating the M^-1 pseudoinverse of the mass
			double *tMi_pinv = matrix_get(6,6); 
			matrix_pinv(6,6,tMi,tMi_pinv); //! M^-1  
			
			//! Constructing the Fh matrix out of Fe and Te
			double *Fh = matrix_get(6,1);
			
			//! Calculating Delta
			
			double *delta_Fh=matrix_get(6,1);
			double *delta_xh=matrix_get(6,1);
			double *delta_dxh=matrix_get(6,1);
			
			math::Vector3 xh_rot (-this->d1_->GetAngle(9).Radian(),0.0,0.0);
			xh_rot  = this->arm3_->GetWorldPose().rot.RotateVector(xh_rot); //! use dhx from the compliant wrist
			
			math::Vector3 dxh_rot (-this->d1_->GetVelocity(9),0.0,0.0);
			dxh_rot  = this->arm3_->GetWorldPose().rot.RotateVector(dxh_rot); //! use ddhx from the compliant wrist
		
			
			for (unsigned int i = 0; i<3; ++i){
				Fh[i]=m.Fe[m.E_NUM][i];
				Fh[3+i]=m.Te[m.E_NUM][i];
				
				//! Fh= [Fe 3x1]
				//!		[Te 3x1]
			}
			
			////! Calculating joint torques!
			
			//matrix_cpy(m.LINKNUM-1,1,Gc,m.Tau); //! Tau= c*
			
			//double *tmp1 = matrix_get(m.LINKNUM-1,1);
			
			//matrix_mult(linknum_chaser-1, 6, 1, GJe_trans, Fh, tmp1); //! tmp1 = -J*'.Fh
			
			////! Calculating acceleration ddxh - tDdi.delta_dxh<< 
			//double *tmp2 = matrix_get(m.LINKNUM-1,1);
			
			
			
			//delete [] *tmp2;
			//delete [] *tmp1;
			
			
			delete [] delta_dxh;
			delete [] delta_xh;
			delete [] Fh;
			delete [] tMi_pinv;
			delete [] GJe_trans;
			delete [] GJe_inv;
			delete [] GJe_devt;
			
			matrix_cpy(6,_linknum_chaser-1,GJe,GJe_prev); //! copy last iteration of GJe_prev
		}
		
		
		
	
		/* ==========================HYBRID CONTROL ================================================================/
		calc_tdxghI(dxgh, mt, tdxghI); //! calculated from the target frame of reference
		
		//! Calculating the transformation tRI
		
		//! Here you leverage the parallel thread of the model of the target that maps the orientation and position of the base into the model mt.
		matrix_cpy(3,3,mt.A0,tCI);//! Copy ICt
		matrix_trans(3,3,tCI,tCI); //! Convert and Obtain tCI
			
		matrix_cpy_sub(6,6,1,3,1,3,tCI,tRI); 
		matrix_cpy_sub(6,6,4,6,4,6,tCI,tRI); //! Obtain tRI; 
		matrix_trans(6,6,tRI,tRI) //! Obtain tRI'
					//! tRI
					//! [tCI 03x3]
					//! [03x3 tcI] 
		//! Calculate the linear and angular velocity from the influence of the target angular velocity
			calc_tdxhr(m, mt, tCI, tdxhr);
		
		//! Transformation to the target frame of reference 
			matrix_mult(6,6,LINKNUM-1,tRI,GJe,tGJeI); //! Calculation of tGJeI = tRI'.GJe; 
		//! Calculate the velocity of the hand with respect to the target
				matrix_cpy(6,1,tdxhr,tdxhI); //! tdxhI= tdxhr
				matrix_add(6,1,tdxhI,tdxghI); //! tdxhI= tdxghI + tdxhr
				double *tmp1;
				tmp1=matrix_get(6,1);
				matrix_mult(6,6,1,tGJeI,m.dq,tmp1); //! tmp1= tJ*.dq
				matrix_add(6,1,tmp1,tdxhI, tdxhI); //! tdxhI= tJ*.dq tdxghI + tdxhr	
				delete[] tmp1;		
		//! Calculate the derivative of the Jacobian with respect to the target
				matrix_devt(6 , LINKNUM-1, GJe , GJe_prev, d_time, dGJe);
				
		
		
		
		// matrix_mult(LINKNUM-1,LINKNUM-1,LINKNUM-1,tXI,GH,tGHI) //! Calculation of tGHI = tXI'.GH To be calculated 
		// matrix_mult(LINKNUM-1,LINKNUM-1,1,tYI,Gc,tGcI); //! Calculation of tGcI= tYI'.Gc. To be calculated 
		
		*/
		
		}
		
		
}
// --- EOF ---

