//
// File: fcn_inshape.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 03-Apr-2019 16:45:30
//

// Include Files
#include "rt_nonfinite.h"
#include "find_P.h"
#include "fcn_inshape.h"

// Function Definitions

//
// judge whether Point_r is in the close shape consist of point1~4
// Arguments    : const double point_r[2]
//                const double points[8]
// Return Type  : bool
//
bool fcn_inshape(const double point_r[2], const double points[8])
{
  bool in_shape;
  double x_tmp;
  double b_x_tmp;
  double c_x_tmp;
  double d_x_tmp;
  double x;
  double sign_v_idx_0;
  bool b_x[4];
  double e_x_tmp;
  double f_x_tmp;

  //  points = [point1(1), point1(2);
  //                 point2(1), point2(2);
  //                 point3(1), point3(2);
  //                 point4(1), point4(2);];
  x_tmp = points[1] - point_r[0];
  b_x_tmp = points[5] - point_r[1];
  c_x_tmp = points[4] - point_r[1];
  d_x_tmp = points[0] - point_r[0];
  x = d_x_tmp * b_x_tmp - c_x_tmp * x_tmp;

  // vector cross
  if (x < 0.0) {
    x = -1.0;
  } else if (x > 0.0) {
    x = 1.0;
  } else {
    if (x == 0.0) {
      x = 0.0;
    }
  }

  sign_v_idx_0 = x;
  b_x[0] = (x == x);
  e_x_tmp = points[2] - point_r[0];
  f_x_tmp = points[6] - point_r[1];
  x = x_tmp * f_x_tmp - b_x_tmp * e_x_tmp;

  // vector cross
  if (x < 0.0) {
    x = -1.0;
  } else if (x > 0.0) {
    x = 1.0;
  } else {
    if (x == 0.0) {
      x = 0.0;
    }
  }

  b_x[1] = (x == sign_v_idx_0);
  x_tmp = points[3] - point_r[0];
  b_x_tmp = points[7] - point_r[1];
  x = e_x_tmp * b_x_tmp - f_x_tmp * x_tmp;

  // vector cross
  if (x < 0.0) {
    x = -1.0;
  } else if (x > 0.0) {
    x = 1.0;
  } else {
    if (x == 0.0) {
      x = 0.0;
    }
  }

  b_x[2] = (x == sign_v_idx_0);
  x = x_tmp * c_x_tmp - b_x_tmp * d_x_tmp;

  // vector cross
  if (x < 0.0) {
    x = -1.0;
  } else if (x > 0.0) {
    x = 1.0;
  } else {
    if (x == 0.0) {
      x = 0.0;
    }
  }

  in_shape = (((b_x[0] + b_x[1]) + b_x[2]) + (x == sign_v_idx_0) == 4);

  //  the four signs are same
  return in_shape;
}

//
// File trailer for fcn_inshape.cpp
//
// [EOF]
//
