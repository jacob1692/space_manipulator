#ifndef DATAFLAG_H
#define DATAFLAG_H

class FLAG
{
public:
    FLAG();

    //=== FLAG ===//
    int F_SD;   // Flag_SaveData_ON_OFF
    int F_T;    // Flag_Timer
    int F_SPD;  // Flag_Spd_Running
    int F_SM;    // Flag_Motor_ON_OFF
    int F_THC;  // Flag_BB_Stick
    int F_CJT;  // Flag_Change_Joint_Type

    int F_CMPOS;  // Flag_Camera_Position
    bool F_VWPOS;   // Flag_View_Position(for debug)
    bool F_FCS;   // Flag_focus_Sphere
    int F_GHST;  // SHOW_GHOST

    int F_CL;   //Camera_Lock
    int F_FL;   //focus_Lock
    int F_GRAPH;
    int F_TITLE;
    int DATA_NUM;
    int F_BASEX;
    int F_TIP;
};

#endif // DATAFLAG_H
