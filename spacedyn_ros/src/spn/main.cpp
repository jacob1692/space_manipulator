// ---------------------------------------------------
// Main Program to Simulate Multibody Dynamic System 
//
//
// s.abiko [2007.5]
// ---------------------------------------------------
#include <iostream>
using namespace std

#include <gsl/gsl_linalg.h>
#include "spacedyn_ros/spd/common.h"
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/spn.h"

int main(void)
{
  Vector6 v = { 1, 2, 3, 4, 5, 6};
  double *a;

  crossM(*v, *a);

  cout << *a << *(a+1) << *(a+2) << "\n";
  //  printf("%f, %f, %f, %f, %f, %f, \n", *a, *(a+1), *(a+2), *(a+3), *(a+4), *(a+5));

  return 0;
}
// --- EOF ---
