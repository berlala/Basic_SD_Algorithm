//
// File: fcn_dist.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 03-Apr-2019 16:57:22
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "find_P.h"
#include "fcn_dist.h"

// Function Definitions

//
// Arguments    : const double point1[2]
//                const double point2[2]
// Return Type  : double
//
double fcn_dist(const double point1[2], const double point2[2])
{
  double a;
  double b_a;
  a = point1[0] - point2[0];
  b_a = point1[1] - point2[1];
  return std::sqrt(a * a + b_a * b_a);
}

//
// File trailer for fcn_dist.cpp
//
// [EOF]
//
