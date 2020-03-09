//
// File: fcn_rectangle.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 03-Apr-2019 16:45:30
//

// Include Files
#include "rt_nonfinite.h"
#include "find_P.h"
#include "fcn_rectangle.h"

// Function Definitions

//
// give back a point to build a rectangle by the 4 points.
// point2 is the right-angle point
// Arguments    : const double point1_data[]
//                const double point2_data[]
//                const double point3_data[]
//                double point[2]
// Return Type  : void
//
void fcn_rectangle(const double point1_data[], const double point2_data[], const
                   double point3_data[], double point[2])
{
  point[0] = (point1_data[0] + point3_data[0]) / 2.0;
  point[1] = (point1_data[1] + point3_data[1]) / 2.0;
  point[0] += point[0] - point2_data[0];
  point[1] += point[1] - point2_data[1];
}

//
// File trailer for fcn_rectangle.cpp
//
// [EOF]
//
