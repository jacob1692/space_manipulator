//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : int_eu( MODEL )
//            calculate position and orientation of each link
//
// s.abiko [2007.5]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

void int_eu(MODEL &m, double d_time, double m.A0, double m.POS0, double m.q, double m.w0, double m.v0, double m.qd){

  if( m.LINKNUM != 1){
    matrix_scale( m.LINKNUM, 1, d_time, m.qdd, tmp_qd );
    matrix_add( m.LINKNUM, 1, m.qd, tmp_qd, m.qd );

    matrix_scale( m.LINKNUM, 1, d_time, m.qdd, tmp_qd );
    matrix_add( m.LINKNUM, 1, m.qd, tmp_qd, m.qd );
  }

  matrix_scale
}

// === EOF ===
