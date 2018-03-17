//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
// Function : calc_tdxghI( MODEL, ans )
//            calculate the linear-angular velocity from the COM to the end effector measured from the target
//
//            ans = (6 x 1) matrix
//
// Jacob [2017.2]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

void calc_tdxghI(double *tRI, double *dxgh, MODEL &mt, double *ans){
		
		//! Construct the matrix of the  linear- angular velocity of the target
		
		double *dxt;
		dxt=matrix_get(6,1);
		for (unsigned int i=0; i<6; ++i){
			dxt[i] = mt.v0[i]; //! linear velocity of the target
			dxt[i+3]=mt.w0[i]; //! angular velocity of the target
			}
		double *tmp1;
		tmp1=matrix_get(6,1);
		matrix_sub(6,1,dxgh,dxt,tmp1); //! tmp1= dxgh - dxt
		
		matrix_mult(6,6,1,tRI,tmp1,ans); //! tdxghI=tRI.(dxgh - dxt) Page 13 English Document
		
		delete[] tmp1;
		delete[] dxt;
}

// === EOF ===
