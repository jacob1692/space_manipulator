//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : calc_L0( MODEL, L0 )
//            
//            calculate the linear and angular momentum around 
//	      the center of mass of the base
//
// Output : Lb : ( 6 x 1 )
//
// s.abiko [2007.5]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

void calc_L0( MODEL &m, double *Lb){
	
  int i;
  Vector6 *f, *v; 
  Matrix6 Xup_T;  
  double *tmp, *tmp1, *tmp2, *tmp3, *tmp4, *Rg, *P0, *L0;

  f  = new Vector6[m.LINKNUM];
  v  = new Vector6[m.LINKNUM];

  tmp  = new Vector6;
  tmp1 = new Vector3;
  tmp2 = new Matrix6;
  tmp3 = new Vector6;
  tmp4 = new Matrix3;

  P0 = new Vector3;
  L0 = new Vector3;

  
  Rg = new Vector3;

  matrix_trans( 6, 6, m.Xup[0], Xup_T );

  for(i=0;i<3;i++){
    tmp[i] = m.v0[i];
    tmp[i+3] = m.w0[i];
  }

  matrix_mult( 6, 6, 1, Xup_T, tmp, v[0]);
  matrix_mult( 6, 6, 1, m.Isp[0], v[0], f[0] );

  
  if( m.LINKNUM != 1 ){
    for( i=1;i<m.LINKNUM;i++){
      
      matrix_trans( 6, 6, m.Xup[i], Xup_T );

      matrix_mult( 6, 6, 1, Xup_T, v[m.BB[i]], tmp );
      matrix_scale( 6, 1, m.qd[i], m.S[i], tmp3 );
      matrix_add( 6, 1, tmp, tmp3, v[i] );
      
      matrix_mult( 6, 6, 1, m.Isp[i], v[i], f[i] );
    }
    
    for( i=m.LINKNUM-1; i>0; i-- ){   
    
      matrix_mult( 6, 6, 1, m.Xup[i], f[i], tmp );
      matrix_add( 6, 1, f[m.BB[i]], tmp, f[m.BB[i]] );
      
    }
  }
  matrix_mult( 6, 6, 1, m.Xup[0], f[0], f[0]);
  
  
  for(i=0;i<3;i++){
    P0[i] = f[0][i];
    L0[i] = f[0][i+3];
    
    Lb[i] = f[0][i];
    Lb[i+3] = f[0][i+3];
  }
  
  //matrix_print( 6, 1, Lg );
  
  delete [] f;
  delete [] v;
  
  delete [] tmp;
  delete [] tmp1;
  delete [] tmp2;
  delete [] tmp3;
  delete [] tmp4;
  
  delete [] Rg;
  delete [] P0;
  delete [] L0;
  
}

// === EOF ===
