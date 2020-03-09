//
// File: sign.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 03-Apr-2019 16:45:30
//

// Include Files
#include "rt_nonfinite.h"
#include "find_P.h"
#include "sign.h"

// Function Definitions

//
// Arguments    : double *x
// Return Type  : void
//
void b_sign(double *x)
{
  if (*x < 0.0) {
    *x = -1.0;
  } else if (*x > 0.0) {
    *x = 1.0;
  } else {
    if (*x == 0.0) {
      *x = 0.0;
    }
  }
}

//
// File trailer for sign.cpp
//
// [EOF]
//
