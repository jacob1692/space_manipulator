//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
// Function : calc_Gc( MODEL, ans )
//            calculate generalized non-linear velocity dependent term
//
//            ans = (LINKNUM-1 x 1) matrix
//
// Jacob Hernandez [2017.2]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

void calc_Gc(MODEL &m, double * Gravity, double *ans){
	
	double *CC;
	double *HH;
	double *cm, *Hbm_t, *Hb, *cb;

	CC = matrix_get ( ( 6 + m.LINKNUM-1 ) , 1);
	HH = matrix_get( (6+m.LINKNUM-1), (6+m.LINKNUM-1) );
	Hb = matrix_get( 6, 6 );
	
	Hbm_t = matrix_get( m.LINKNUM-1, 6 );
	cm = matrix_get( m.LINKNUM-1, 1 );
	cb = matrix_get( 6, 1 );
	

	
	calc_hh( m, HH );
	
	matrix_ext_sub( (6+m.LINKNUM-1), (6+m.LINKNUM-1), 1, 6, 1, 6, HH, Hb );
	matrix_ext_sub( (6+m.LINKNUM-1), (6+m.LINKNUM-1), 7, 6+m.LINKNUM-1, 1, 6, HH, Hbm_t );
	
	
	calc_C(m,Gravity,CC); 
	
	//matrix_Z( (6 + m.LINKNUM-1) , 1, CC);
	
	matrix_ext_sub( (6+m.LINKNUM-1), 1, 1, 6, 1, 1, CC, cb );
	matrix_ext_sub( (6+m.LINKNUM-1), 1, 7, 6+m.LINKNUM-1 , 1, 1, CC, cm );
		
	matrix_inv( 6, Hb, Hb );
	matrix_mult( 6, 6, 1, Hb, cb, cb ); //! tmp1= Hb¨-1 *cb;
	double *tmp2;
	tmp2=matrix_get(m.LINKNUM-1,1);
	matrix_mult( m.LINKNUM-1, 6, 1, Hbm_t, cb, tmp2 ); //! tmp2= Hbm_t*tmp1
	matrix_sub( m.LINKNUM-1, 1, cm, tmp2, ans ); //! ans= cm - tmp2
	
	delete [] tmp2;
	delete [] CC;
	delete [] HH;
	delete [] Hb;
	delete [] cm;
	delete [] Hbm_t;
	delete [] cb;	
	
}

// === EOF ===
