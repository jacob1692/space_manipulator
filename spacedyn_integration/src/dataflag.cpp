#include "spacedyn_integration/dataflag.h"

FLAG::FLAG()
{
    //=== initialize flag ====//
    F_SD = false;
    F_T  = false;
    F_SPD = false;
    F_SM = 0; // motor 0=off,1=on,2=stop
    F_THC = false;
    F_CJT = false;

    F_CMPOS = 0;
    F_VWPOS = false;
    F_FCS = true;

    F_CL = false;
    F_FL = false;

    F_GHST = false;
    F_GRAPH = false;
    F_TITLE = 0; // 0 = default; 1 = m.pos0[0]; 2 = m.pos[1]; 3 = m.v0[0]; 4 = m.qd[1];
//    DATA_NUM = 0;
//    F_CLICK = false;
    F_BASEX = false;
    F_TIP = 0;
}
