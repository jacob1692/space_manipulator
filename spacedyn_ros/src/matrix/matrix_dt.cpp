//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
// Function : matrix_dt(int m, int n, double *a, double *ans )
//            calculate the influence vector of the target angular velocity to the vecloity of the hand
//  
//            ans = (m x n) matrix
//
// Jacob [2017.2]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

void matrix_dt(int m, int n, double *a, double *ans ){

	
	double *wtilde;
	double *tCI_t;
	double *tmp1;
	
	wtilde = matrix_get(3,3);
	tCI_t = matrix_get(3,3);
	tmp1 = matrix_get(3,1);
	
	vector_tilde3( 3, mt.w0, wtilde );
	matrix_scale( 3, 3, -1, wtilde, wtilde ); //! Calculate -wt~ 
	
	matrix_trans( 3, 3, tCI, tCI_t);
	
	for (unsigned int i=0;i<3,++i){
		tmp1[i] = m.POS_e[i] - mt.POS0[i]; //! Calculate (Irh - Irt)
		}
	
	matrix_mult( 3, 3, 1, tCI_t, tmp1, tmp1 ); //! tmp1= iCt'.(Irh - Irt)
	
	matrix_mult( 3, 3, 1, wtilde , tmp1 , tmp1 ); //! tmp1 = -wt~.iCt'.(Irh - Irt)
	
	matrix_cpy_sub( 6, 1, 1, 3, 1, 1, tmp1, ans ); //! ans= [-wt~.iCt'.(Irh - Irt)]
											 //!      [         0 3x1       ]		

	
	delete [] wtilde;
	delete [] tCI_t;
	delete [] tmp1;
}

// === EOF ===
