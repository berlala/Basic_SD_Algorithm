//
// File: fcn_4points.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 03-Apr-2019 16:57:22
//

// Include Files
#include "rt_nonfinite.h"
#include "find_P.h"
#include "fcn_4points.h"

// Function Definitions

//
// in this function, the output is the responding point for the Req point;
// [point1] and [point2] are the two line to consist of <line1>
// [point3] and [point4] are the two line to consist of <line2>
//  [req] is a point on <Line1>
//  this function give back a point that a <line3> which vertical to <Line1>
//  by point [req]
// Arguments    : const double point_r_data[]
//                const double point1[2]
//                const double point2[2]
//                const double point3[2]
//                const double point4[2]
//                double cross_data[]
//                int cross_size[2]
// Return Type  : void
//
void fcn_4points(const double point_r_data[], const double point1[2], const
                 double point2[2], const double point3[2], const double point4[2],
                 double cross_data[], int cross_size[2])
{
  double K2;
  double b;
  double x_cross;
  cross_size[0] = 0;
  cross_size[1] = 0;
  if (!(point1[0] == point2[0])) {
    if (point1[1] == point2[1]) {
      // parallel to X
      cross_size[0] = 1;
      cross_size[1] = 2;
      cross_data[0] = point_r_data[0];
      cross_data[1] = (point_r_data[0] - point3[0]) / (point4[0] - point3[0]) *
        (point4[1] - point3[1]) + point3[1];
    } else {
      //  the slope of the line1
      K2 = -1.0 / ((point2[1] - point1[1]) / (point2[0] - point1[0]));

      // the slope of the porpose line
      b = point_r_data[1] - K2 * point_r_data[0];

      // syms x
      // eqn1 = K2*x+ b ;
      // eqn2 = ((x - points(3,1))/(points(4,1) - points(3,1)))*(points(4,2) -
      // points(3,2))+points(3,2);
      // eqns = [eqn1-eqn2 == 0];
      // x_cross = eval(solve(eqns,x));
      x_cross = point3[0] * point3[1];
      x_cross = (((((x_cross - point3[0] * point4[1]) + point4[0] * point3[1]) -
                   x_cross) + point3[0] * b) - b * point4[0]) / (((K2 * point4[0]
        - K2 * point3[0]) - point4[1]) + point3[1]);
      cross_size[0] = 1;
      cross_size[1] = 2;
      cross_data[0] = x_cross;
      cross_data[1] = x_cross * K2 + b;
    }
  } else {
    //  parallel to Y
  }
}

//
// File trailer for fcn_4points.cpp
//
// [EOF]
//
