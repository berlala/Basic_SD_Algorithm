//
// File: fcn_3points.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 03-Apr-2019 16:45:30
//

// Include Files
#include "rt_nonfinite.h"
#include "find_P.h"
#include "fcn_3points.h"

// Function Definitions

//
// point2 and point3 are the point on the line
// cross is the coordinate of the corss point
// Arguments    : const double point1[2]
//                const double point2[2]
//                const double point3[2]
//                double cross[2]
// Return Type  : void
//
void fcn_3points(const double point1[2], const double point2[2], const double
                 point3[2], double cross[2])
{
  double KK;
  double b;
  double x_cross;
  if (point3[0] == point2[0]) {
    //  vertical case
    cross[0] = point2[0];
    cross[1] = point1[1];
  } else if (point3[1] == point2[1]) {
    cross[0] = point1[0];
    cross[1] = point2[1];

    //  level case
  } else {
    //  the slope of the line
    KK = -1.0 / ((point3[1] - point2[1]) / (point3[0] - point2[0]));

    // the slope of the porpose line
    b = point1[1] - KK * point1[0];

    //     syms x
    //      eqn1 = KK*x+ b ;
    //      eqn2 = ((x - point3(1))/(point2(1) - point3(1)))*(point2(2) - point3(2))+point3(2); 
    //      eqns = [eqn1-eqn2 == 0];
    //      %eqns = [KK*x+ b,  ((x - point3(1))/(point2(1) - point3(1)))*(point2(2) - point3(2))+point3(2)]; 
    //      x_cross = eval(solve(eqns,x));
    x_cross = point3[0] * point3[1];
    x_cross = (((((x_cross - point3[0] * point2[1]) + point2[0] * point3[1]) -
                 x_cross) + point3[0] * b) - b * point2[0]) / (((KK * point2[0]
      - KK * point3[0]) - point2[1]) + point3[1]);
    cross[0] = x_cross;
    cross[1] = x_cross * KK + b;
  }
}

//
// File trailer for fcn_3points.cpp
//
// [EOF]
//
