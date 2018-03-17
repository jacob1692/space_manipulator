//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
// Function : calc_GH( MODEL, ans )
//            calculate generalized inertia matrix
//
//            ans = (LINKNUM-1 x LINKNUM-1) matrix
//
// Jacob [2017.2]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

void calc_GH(MODEL &m, double *ans){

	double *HH;
	double *Hb, *Hbm, *Hbm_t, *Hm;
	
	HH = matrix_get( (6+m.LINKNUM-1), (6+m.LINKNUM-1) );
	Hb = matrix_get( 6, 6 );
	Hbm = matrix_get( 6, m.LINKNUM-1 );
	
	Hbm_t = matrix_get( m.LINKNUM-1, 6 );
	Hm = matrix_get( m.LINKNUM-1, m.LINKNUM-1 );

	calc_hh( m, HH );
	matrix_ext_sub( (6+m.LINKNUM-1), (6+m.LINKNUM-1), 1, 6, 1, 6, HH, Hb );
	matrix_ext_sub( (6+m.LINKNUM-1), (6+m.LINKNUM-1), 1, 6, 7, 6+m.LINKNUM-1, HH, Hbm );
	
	matrix_ext_sub( (6+m.LINKNUM-1), (6+m.LINKNUM-1), 7, 6+m.LINKNUM-1, 1, 6, HH, Hbm_t );
	matrix_ext_sub( (6+m.LINKNUM-1), (6+m.LINKNUM-1), 7, 6+m.LINKNUM-1, 7, 6+m.LINKNUM-1, HH, Hm );
	
		
	matrix_inv( 6, Hb, Hb );
	matrix_mult( 6, 6, m.LINKNUM-1, Hb, Hbm, Hbm ); //! tmp1= Hb¨-1 *Hbm;
	double *tmp2;
	tmp2=matrix_get(m.LINKNUM-1,m.LINKNUM-1);
	matrix_mult( m.LINKNUM-1, 6, m.LINKNUM-1, Hbm_t, Hbm, tmp2 ); //! tmp2= Hbm_t*tmp1
	matrix_sub( m.LINKNUM-1, m.LINKNUM-1, Hm, tmp2, ans ); //! ans= Hm - tmp2
	
	delete [] HH;
	delete [] Hb;
	delete [] Hbm;
	delete [] Hbm_t;
	delete [] Hm;
	delete [] tmp2;
	
}

// === EOF ===
