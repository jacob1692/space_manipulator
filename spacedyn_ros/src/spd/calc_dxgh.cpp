//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
// Function : calc_dxgh( MODEL, ans )
//            calculate the linear-angular velocity from the COM to the end effector
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

void calc_dxgh(MODEL &m, double *ans){

	
	double *HH;
	double *PLb;
	double *Hb, *Jb;
	
	PLb=matrix_get( 6 , 1); //! [Linear Momentum 3x1 ; Angular Momentum 3x1 ]
	
	HH = matrix_get( (6+m.LINKNUM-1), (6+m.LINKNUM-1) );
	Hb = matrix_get( 6, 6 );

	Jb = matrix_get( 6, 6 );
	
	calc_Jb( m, m.E_NUM, Jb );	

	calc_hh( m, HH );
	matrix_ext_sub( (6+m.LINKNUM-1), (6+m.LINKNUM-1), 1, 6, 1, 6, HH, Hb );
		
	matrix_inv( 6, Hb, Hb );
	matrix_mult( 6, 6, 1, Hb, PLb, PLb );
	matrix_mult( 6, 6, 1, Jb, PLb, ans );
	
	
	delete [] HH;
	delete [] PLb;
	delete [] Hb;
	delete [] Jb;
	
}

// === EOF ===
