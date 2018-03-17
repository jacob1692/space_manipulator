//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : calc_SS( MODEL, SS )
//            calculate children links connectivity
//
// Output : SS ( LINKNUM x LINKNUM ) matrix
//
// s.abiko [2007.5]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

// ---- extract children links ----
void calc_SS(MODEL &m, double *SS){

  int i, j;

  matrix_I(m.LINKNUM,SS);
  matrix_scale(m.LINKNUM,m.LINKNUM,-1,SS,SS);

  for(j=1;j<m.LINKNUM;j++){
    i =  m.BB[j];
    SS[m.LINKNUM*i+j] = 1.0;
  }
  //matrix_print(m.LINKNUM,m.LINKNUM,SS);
}

// === EOF === 

