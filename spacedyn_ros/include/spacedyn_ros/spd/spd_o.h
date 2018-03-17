#ifndef SPD_O_H
#define SPD_O_H

#include <iostream>
using namespace std;
#include <string>
#include "spacedyn_ros/spd/model.h"
#include "spacedyn_ros/spd/define.h"
#include "spacedyn_ros/spd/common.h"

void model_param( string, MODEL & );
void model_init( dWorldID &, MODEL & );
void model_draw( MODEL & );
void model_ode_result(MODEL &m, Vector3, Matrix3, double*, double* );
void f_kin( Vector3, Matrix3, double *, MODEL & );
void calc_jt( MODEL &, double ** );
void calc_jr( MODEL &, double ** );
void calc_jte( MODEL &, int, int, int[], double* );
void calc_jre( MODEL &, int, int, int[], double* );
void calc_HH_q( MODEL &, double * );

#endif// SPD_O_H
