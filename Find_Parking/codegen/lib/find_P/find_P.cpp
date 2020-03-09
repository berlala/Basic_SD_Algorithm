//
// File: find_P.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 03-Apr-2019 16:45:30
//

// Include Files
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <string.h>
#include "find_P.h"
#include "sign.h"
#include "sort1.h"
#include "fcn_dist.h"
#include "fcn_inshape.h"
#include "fcn_rectangle.h"
#include "fcn_4points.h"
#include "sum.h"
#include "fcn_3points.h"
#include "sqrt.h"

// Type Definitions
struct emxArray_real_T_2x2
{
  double data[4];
  int size[2];
};

struct scFNy68armYpSMF2B9nOvZH_tag
{
  emxArray_real_T_2x2 f1;
};

typedef scFNy68armYpSMF2B9nOvZH_tag cell_wrap_1;

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// FF China, IAI, ir.Bolin ZHAO (bolin.zhao@ff.com)
//  2019/3/28
// Intro: this function is for finding the Parking space and location from a
// defined space.
//
// input:
//  Four point location: [a,b;c,d;e,f;g,h];
//  the sequence should be:
//
// 4 3  | 4 3
// 1 2  | 1 2
// -------------
// 1 2  | 1 2
// 4 3  | 4 3
// Arguments    : const double block[8]
//                double Point_Target[3]
//                double *Type
//                double size[2]
// Return Type  : void
//
void find_P(const double block[8], double Point_Target[3], double *Type, double
            size[2])
{
  int situation;
  double dist[8];
  double dis_all;
  double cum;
  double x[4];
  int iidx[4];
  int Trapezoid_size_idx_0;
  double Trapezoid_data[8];
  double dist_left[2];
  double b_block[2];
  double c_block[2];
  double point_set_1[8];
  double point_set_2[8];
  double dv0[2];
  int index_next;
  int ii;
  int Trag_2_1_size_idx_0;
  int Trag_2_1_size_idx_1;
  double b_point_set_1[2];
  double cross_1_data[2];
  double dist_right_idx_0;
  double dist_right_idx_1;
  bool guard1 = false;
  bool guard2 = false;
  bool b_dist_left[2];
  double Trag_1_1_data[6];
  double Trag_2_1_data[6];
  int i0;
  double b_Trag_1_1_data[2];
  int cross_1_size[2];
  cell_wrap_1 reshapes[2];
  int loop_ub;
  cell_wrap_1 b_reshapes[2];
  double dist_points[16];
  signed char sizes_idx_0;
  double dist_final[4];
  int i1;
  double Yaw;
  double dist_tmp;
  bool varargin_1_idx_0;

  // output:
  //  Point_Target = [x;y;yaw]
  //  Type : 1 for Vertical, 3 for parallel
  //  size: [short edge, long edge] for final cube space.
  //
  situation = 0;

  //  1 for Trapezoid
  memset(&dist[0], 0, sizeof(double) << 3);
  dis_all = block[0] - block[1];
  cum = block[4] - block[5];
  dist[0] = dis_all * dis_all + cum * cum;
  b_sqrt(&dist[0]);
  dist[4] = 1.0;
  dis_all = block[1] - block[2];
  cum = block[5] - block[6];
  dist[1] = dis_all * dis_all + cum * cum;
  b_sqrt(&dist[1]);
  dist[5] = 2.0;
  dis_all = block[2] - block[3];
  cum = block[6] - block[7];
  dist[2] = dis_all * dis_all + cum * cum;
  b_sqrt(&dist[2]);
  dist[6] = 3.0;
  dis_all = block[3] - block[0];
  cum = block[7] - block[4];
  dist[3] = dis_all * dis_all + cum * cum;
  b_sqrt(&dist[3]);
  x[0] = dist[0];
  x[1] = dist[1];
  x[2] = dist[2];
  x[3] = dist[3];
  sort(x, iidx);

  //  Init  for Code Gen
  Trapezoid_size_idx_0 = 4;
  memset(&Trapezoid_data[0], 0, sizeof(double) << 3);

  //   Judge the Type and Direction
  //  when the first edge is 4 or 3, means the first edge is long edge.
  if (((iidx[0] == 2.0) || (iidx[0] == 4.0)) && ((iidx[2] == 1.0) || (iidx[2] ==
        3.0))) {
    //  do not take xiefang into account
    *Type = 3.0;

    // parallel
  } else {
    *Type = 1.0;

    // vertical
  }

  //   Special Case
  //  if Type == 3  % parallel, two long
  //      L11 = a;
  //      L12 = b;
  //      L21 = d;
  //      L22 = c;
  //  elseif Type == 1 %vertical, two long
  //      L11 = a;
  //      L12 =d;
  //      L21 = b;
  //      L22 = c;
  //  end
  //
  //  if Type ==3  % parallel case
  //      if L11(2) ==L12(2)
  //          Line1 = L11(2); % parallel to x-axis
  //      end
  //      if L21(2) == L22(2)
  //          Line2  = L21(2);
  //      end
  //  end
  //
  //  if Type ==1 % vertical case
  //      if L11(1) ==L12(1)
  //          Line1 = L11(1); % parallel to y-axis
  //      end
  //      if L21(1) == L22(1)
  //          Line2  = L21(1);
  //      end
  //  end
  //  General Case
  memset(&dist[0], 0, sizeof(double) << 3);
  if (*Type == 3.0) {
    // affect the block(1,:) and block(2,:)
    dist_left[0] = block[2];
    b_block[0] = block[0];
    c_block[0] = block[1];
    dist_left[1] = block[6];
    b_block[1] = block[4];
    c_block[1] = block[5];
    fcn_3points(dist_left, b_block, c_block, dv0);
    dist[2] = dv0[0];
    dist[6] = dv0[1];
    dist_left[0] = block[3];
    b_block[0] = block[0];
    c_block[0] = block[1];
    dist_left[1] = block[7];
    b_block[1] = block[4];
    c_block[1] = block[5];
    fcn_3points(dist_left, b_block, c_block, dv0);
    dist[3] = dv0[0];
    dist[7] = dv0[1];
    dist_left[0] = block[0];
    b_block[0] = block[2];
    c_block[0] = block[3];
    dist_left[1] = block[4];
    b_block[1] = block[6];
    c_block[1] = block[7];
    fcn_3points(dist_left, b_block, c_block, dv0);
    dist[0] = dv0[0];
    dist[4] = dv0[1];
    dist_left[0] = block[1];
    b_block[0] = block[2];
    c_block[0] = block[3];
    dist_left[1] = block[5];
    b_block[1] = block[6];
    c_block[1] = block[7];
    fcn_3points(dist_left, b_block, c_block, dv0);
    dist[1] = dv0[0];
    dist[5] = dv0[1];
  }

  //  [block_v] give the vertical point from each point under the sequence 1234  
  if (*Type == 1.0) {
    // Vertical Case
    dist_left[0] = block[1];
    b_block[0] = block[0];
    c_block[0] = block[3];
    dist_left[1] = block[5];
    b_block[1] = block[4];
    c_block[1] = block[7];
    fcn_3points(dist_left, b_block, c_block, dv0);
    dist[1] = dv0[0];
    dist[5] = dv0[1];
    dist_left[0] = block[2];
    b_block[0] = block[0];
    c_block[0] = block[3];
    dist_left[1] = block[6];
    b_block[1] = block[4];
    c_block[1] = block[7];
    fcn_3points(dist_left, b_block, c_block, dv0);
    dist[2] = dv0[0];
    dist[6] = dv0[1];
    dist_left[0] = block[0];
    b_block[0] = block[1];
    c_block[0] = block[2];
    dist_left[1] = block[4];
    b_block[1] = block[5];
    c_block[1] = block[6];
    fcn_3points(dist_left, b_block, c_block, dv0);
    dist[0] = dv0[0];
    dist[4] = dv0[1];
    dist_left[0] = block[3];
    b_block[0] = block[1];
    c_block[0] = block[2];
    dist_left[1] = block[7];
    b_block[1] = block[5];
    c_block[1] = block[6];
    fcn_3points(dist_left, b_block, c_block, dv0);
    dist[3] = dv0[0];
    dist[7] = dv0[1];
  }

  //   Situation A: Trapezoid
  memset(&point_set_1[0], 0, sizeof(double) << 3);
  memset(&point_set_2[0], 0, sizeof(double) << 3);
  if (*Type == 3.0) {
    point_set_1[0] = block[0];
    point_set_1[1] = dist[2];
    point_set_1[2] = dist[3];
    point_set_1[3] = block[1];
    point_set_2[0] = block[2];
    point_set_2[1] = dist[0];
    point_set_2[2] = dist[1];
    point_set_2[3] = block[3];
    point_set_1[4] = block[4];
    point_set_1[5] = dist[6];
    point_set_1[6] = dist[7];
    point_set_1[7] = block[5];
    point_set_2[4] = block[6];
    point_set_2[5] = dist[4];
    point_set_2[6] = dist[5];
    point_set_2[7] = block[7];
  } else {
    if (*Type == 1.0) {
      point_set_1[0] = block[1];
      point_set_1[1] = dist[0];
      point_set_1[2] = dist[3];
      point_set_1[3] = block[2];
      point_set_2[0] = block[0];
      point_set_2[1] = dist[1];
      point_set_2[2] = dist[2];
      point_set_2[3] = block[3];
      point_set_1[4] = block[5];
      point_set_1[5] = dist[4];
      point_set_1[6] = dist[7];
      point_set_1[7] = block[6];
      point_set_2[4] = block[4];
      point_set_2[5] = dist[5];
      point_set_2[6] = dist[6];
      point_set_2[7] = block[7];
    }
  }

  //  Judge the Shape/Situation
  //  judge whether the two points in the middle, for Line1
  index_next = 0;
  ii = 0;
  Trag_2_1_size_idx_0 = 0;
  Trag_2_1_size_idx_1 = 0;
  b_point_set_1[0] = point_set_1[0];
  cross_1_data[0] = point_set_1[3];
  b_point_set_1[1] = point_set_1[4];
  cross_1_data[1] = point_set_1[7];
  dis_all = fcn_dist(b_point_set_1, cross_1_data);

  // left point to two middle point
  b_point_set_1[0] = point_set_1[1];
  cross_1_data[0] = point_set_1[0];
  b_point_set_1[1] = point_set_1[5];
  cross_1_data[1] = point_set_1[4];
  dist_left[0] = fcn_dist(b_point_set_1, cross_1_data);

  // right point to two middle point
  b_point_set_1[0] = point_set_1[2];
  cross_1_data[0] = point_set_1[0];
  b_point_set_1[1] = point_set_1[6];
  cross_1_data[1] = point_set_1[4];
  dist_right_idx_0 = fcn_dist(b_point_set_1, cross_1_data);

  // left point to two middle point
  b_point_set_1[0] = point_set_1[1];
  cross_1_data[0] = point_set_1[3];
  b_point_set_1[1] = point_set_1[5];
  cross_1_data[1] = point_set_1[7];
  dist_left[1] = fcn_dist(b_point_set_1, cross_1_data);

  // right point to two middle point
  b_point_set_1[0] = point_set_1[2];
  cross_1_data[0] = point_set_1[3];
  b_point_set_1[1] = point_set_1[6];
  cross_1_data[1] = point_set_1[7];
  dist_right_idx_1 = fcn_dist(b_point_set_1, cross_1_data);

  //
  if ((dist_left[0] < dist_left[1]) || (rtIsNaN(dist_left[0]) && (!rtIsNaN
        (dist_left[1])))) {
    cum = dist_left[1];
  } else {
    cum = dist_left[0];
  }

  guard1 = false;
  guard2 = false;
  if (cum < dis_all) {
    if ((dist_right_idx_0 < dist_right_idx_1) || (rtIsNaN(dist_right_idx_0) && (
          !rtIsNaN(dist_right_idx_1)))) {
      cum = dist_right_idx_1;
    } else {
      cum = dist_right_idx_0;
    }

    if (cum < dis_all) {
      //  the two point in the middle of point1 and 2
      dist[0] = point_set_1[1];
      dist[1] = point_set_1[2];
      dist[2] = point_set_2[3];
      dist[3] = point_set_2[0];
      dist[4] = point_set_1[5];
      dist[5] = point_set_1[6];
      dist[6] = point_set_2[7];
      dist[7] = point_set_2[4];
      memcpy(&Trapezoid_data[0], &dist[0], sizeof(double) << 3);

      //  the four point for the Trapezoid shape
      situation = 1;
    } else {
      guard2 = true;
    }
  } else {
    guard2 = true;
  }

  if (guard2) {
    b_dist_left[0] = (dist_left[0] < dis_all);
    b_dist_left[1] = (dist_left[1] < dis_all);
    if (sum(b_dist_left) == 2.0) {
      b_dist_left[0] = (dist_right_idx_0 < dis_all);
      b_dist_left[1] = (dist_right_idx_1 < dis_all);
      if (sum(b_dist_left) == 1.0) {
        // judge which point on the line
        if (dist_left[0] > dist_left[1]) {
          b_point_set_1[0] = point_set_1[1];
          b_point_set_1[1] = point_set_1[5];
          index_next = 2;
          ii = 2;
          memcpy(&Trag_1_1_data[0], &b_point_set_1[0], sizeof(double));
          memcpy(&Trag_1_1_data[2], &b_point_set_1[1], sizeof(double));
          Trag_1_1_data[1] = point_set_1[0];
          Trag_1_1_data[3] = point_set_1[4];
          situation = 2;
        } else {
          b_point_set_1[0] = point_set_1[1];
          b_point_set_1[1] = point_set_1[5];
          index_next = 2;
          ii = 2;
          memcpy(&Trag_1_1_data[0], &b_point_set_1[0], sizeof(double));
          memcpy(&Trag_1_1_data[2], &b_point_set_1[1], sizeof(double));
          Trag_1_1_data[1] = point_set_1[3];
          Trag_1_1_data[3] = point_set_1[7];
          situation = 2;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    b_dist_left[0] = (dist_left[0] < dis_all);
    b_dist_left[1] = (dist_left[1] < dis_all);
    if (sum(b_dist_left) == 1.0) {
      b_dist_left[0] = (dist_right_idx_0 < dis_all);
      b_dist_left[1] = (dist_right_idx_1 < dis_all);
      if (sum(b_dist_left) == 2.0) {
        if (dist_right_idx_0 > dist_right_idx_1) {
          b_point_set_1[0] = point_set_1[2];
          b_point_set_1[1] = point_set_1[6];
          index_next = 2;
          ii = 2;
          memcpy(&Trag_1_1_data[0], &b_point_set_1[0], sizeof(double));
          memcpy(&Trag_1_1_data[2], &b_point_set_1[1], sizeof(double));
          Trag_1_1_data[1] = point_set_1[0];
          Trag_1_1_data[3] = point_set_1[4];
          situation = 2;
        } else {
          b_point_set_1[0] = point_set_1[2];
          b_point_set_1[1] = point_set_1[6];
          index_next = 2;
          ii = 2;
          memcpy(&Trag_1_1_data[0], &b_point_set_1[0], sizeof(double));
          memcpy(&Trag_1_1_data[2], &b_point_set_1[1], sizeof(double));
          Trag_1_1_data[1] = point_set_1[3];
          Trag_1_1_data[3] = point_set_1[7];
          situation = 2;
        }
      }
    }
  }

  if ((dist_left[0] > dist_left[1]) || (rtIsNaN(dist_left[0]) && (!rtIsNaN
        (dist_left[1])))) {
    cum = dist_left[1];
  } else {
    cum = dist_left[0];
  }

  if ((dist_right_idx_0 > dist_right_idx_1) || (rtIsNaN(dist_right_idx_0) &&
       (!rtIsNaN(dist_right_idx_1)))) {
    dis_all = dist_right_idx_1;
  } else {
    dis_all = dist_right_idx_0;
  }

  if (cum == dis_all) {
    if ((dist_left[0] < dist_left[1]) || (rtIsNaN(dist_left[0]) && (!rtIsNaN
          (dist_left[1])))) {
      cum = dist_left[1];
    } else {
      cum = dist_left[0];
    }

    if ((dist_right_idx_0 < dist_right_idx_1) || (rtIsNaN(dist_right_idx_0) && (
          !rtIsNaN(dist_right_idx_1)))) {
      dist_right_idx_0 = dist_right_idx_1;
    }

    if (cum == dist_right_idx_0) {
      // for line1 is [x,0]or[0,x]
      memcpy(&Trapezoid_data[0], &block[0], sizeof(double) << 3);
    }
  }

  //  for Line2
  b_point_set_1[0] = point_set_2[0];
  cross_1_data[0] = point_set_2[3];
  b_point_set_1[1] = point_set_2[4];
  cross_1_data[1] = point_set_2[7];
  dis_all = fcn_dist(b_point_set_1, cross_1_data);

  // left point to two middle point
  b_point_set_1[0] = point_set_2[1];
  cross_1_data[0] = point_set_2[0];
  b_point_set_1[1] = point_set_2[5];
  cross_1_data[1] = point_set_2[4];
  dist_left[0] = fcn_dist(b_point_set_1, cross_1_data);

  // right point to two middle point
  b_point_set_1[0] = point_set_2[2];
  cross_1_data[0] = point_set_2[0];
  b_point_set_1[1] = point_set_2[6];
  cross_1_data[1] = point_set_2[4];
  dist_right_idx_0 = fcn_dist(b_point_set_1, cross_1_data);

  // left point to two middle point
  b_point_set_1[0] = point_set_2[1];
  cross_1_data[0] = point_set_2[3];
  b_point_set_1[1] = point_set_2[5];
  cross_1_data[1] = point_set_2[7];
  dist_left[1] = fcn_dist(b_point_set_1, cross_1_data);

  // right point to two middle point
  b_point_set_1[0] = point_set_2[2];
  cross_1_data[0] = point_set_2[3];
  b_point_set_1[1] = point_set_2[6];
  cross_1_data[1] = point_set_2[7];
  dist_right_idx_1 = fcn_dist(b_point_set_1, cross_1_data);
  if ((dist_left[0] < dist_left[1]) || (rtIsNaN(dist_left[0]) && (!rtIsNaN
        (dist_left[1])))) {
    cum = dist_left[1];
  } else {
    cum = dist_left[0];
  }

  guard1 = false;
  guard2 = false;
  if (cum < dis_all) {
    if ((dist_right_idx_0 < dist_right_idx_1) || (rtIsNaN(dist_right_idx_0) && (
          !rtIsNaN(dist_right_idx_1)))) {
      cum = dist_right_idx_1;
    } else {
      cum = dist_right_idx_0;
    }

    if (cum < dis_all) {
      //  the two point in the middle of point1 and 2
      dist[0] = point_set_2[1];
      dist[1] = point_set_2[2];
      dist[2] = point_set_1[3];
      dist[3] = point_set_1[0];
      dist[4] = point_set_2[5];
      dist[5] = point_set_2[6];
      dist[6] = point_set_1[7];
      dist[7] = point_set_1[4];
      memcpy(&Trapezoid_data[0], &dist[0], sizeof(double) << 3);

      //  the four point for the Trapezoid shape
      situation = 1;
    } else {
      guard2 = true;
    }
  } else {
    guard2 = true;
  }

  if (guard2) {
    b_dist_left[0] = (dist_left[0] < dis_all);
    b_dist_left[1] = (dist_left[1] < dis_all);
    if (sum(b_dist_left) == 2.0) {
      b_dist_left[0] = (dist_right_idx_0 < dis_all);
      b_dist_left[1] = (dist_right_idx_1 < dis_all);
      if (sum(b_dist_left) == 1.0) {
        // judge which point on the line
        if (dist_left[0] > dist_left[1]) {
          b_point_set_1[0] = point_set_2[1];
          b_point_set_1[1] = point_set_2[5];
          Trag_2_1_size_idx_0 = 2;
          Trag_2_1_size_idx_1 = 2;
          memcpy(&Trag_2_1_data[0], &b_point_set_1[0], sizeof(double));
          memcpy(&Trag_2_1_data[2], &b_point_set_1[1], sizeof(double));
          Trag_2_1_data[1] = point_set_2[0];
          Trag_2_1_data[3] = point_set_2[4];
        } else {
          b_point_set_1[0] = point_set_2[1];
          b_point_set_1[1] = point_set_2[5];
          Trag_2_1_size_idx_0 = 2;
          Trag_2_1_size_idx_1 = 2;
          memcpy(&Trag_2_1_data[0], &b_point_set_1[0], sizeof(double));
          memcpy(&Trag_2_1_data[2], &b_point_set_1[1], sizeof(double));
          Trag_2_1_data[1] = point_set_2[3];
          Trag_2_1_data[3] = point_set_2[7];
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    b_dist_left[0] = (dist_left[0] < dis_all);
    b_dist_left[1] = (dist_left[1] < dis_all);
    if (sum(b_dist_left) == 1.0) {
      b_dist_left[0] = (dist_right_idx_0 < dis_all);
      b_dist_left[1] = (dist_right_idx_1 < dis_all);
      if (sum(b_dist_left) == 2.0) {
        if (dist_right_idx_0 > dist_right_idx_1) {
          b_point_set_1[0] = point_set_2[2];
          b_point_set_1[1] = point_set_2[6];
          Trag_2_1_size_idx_0 = 2;
          Trag_2_1_size_idx_1 = 2;
          memcpy(&Trag_2_1_data[0], &b_point_set_1[0], sizeof(double));
          memcpy(&Trag_2_1_data[2], &b_point_set_1[1], sizeof(double));
          Trag_2_1_data[1] = point_set_2[0];
          Trag_2_1_data[3] = point_set_2[4];
          situation = 2;
        } else {
          b_point_set_1[0] = point_set_2[2];
          b_point_set_1[1] = point_set_2[6];
          Trag_2_1_size_idx_0 = 2;
          Trag_2_1_size_idx_1 = 2;
          memcpy(&Trag_2_1_data[0], &b_point_set_1[0], sizeof(double));
          memcpy(&Trag_2_1_data[2], &b_point_set_1[1], sizeof(double));
          Trag_2_1_data[1] = point_set_2[3];
          Trag_2_1_data[3] = point_set_2[7];
          situation = 2;
        }
      }
    }
  }

  if ((dist_left[0] > dist_left[1]) || (rtIsNaN(dist_left[0]) && (!rtIsNaN
        (dist_left[1])))) {
    cum = dist_left[1];
  } else {
    cum = dist_left[0];
  }

  if ((dist_right_idx_0 > dist_right_idx_1) || (rtIsNaN(dist_right_idx_0) &&
       (!rtIsNaN(dist_right_idx_1)))) {
    dis_all = dist_right_idx_1;
  } else {
    dis_all = dist_right_idx_0;
  }

  if (cum == dis_all) {
    if ((dist_left[0] < dist_left[1]) || (rtIsNaN(dist_left[0]) && (!rtIsNaN
          (dist_left[1])))) {
      cum = dist_left[1];
    } else {
      cum = dist_left[0];
    }

    if ((dist_right_idx_0 < dist_right_idx_1) || (rtIsNaN(dist_right_idx_0) && (
          !rtIsNaN(dist_right_idx_1)))) {
      dist_right_idx_0 = dist_right_idx_1;
    }

    if (cum == dist_right_idx_0) {
      // for line2 is [x,0]or[0,x]
      memcpy(&Trapezoid_data[0], &block[0], sizeof(double) << 3);
    }
  }

  //  Build two Triangles
  if (situation == 2) {
    if ((index_next != 0) && (ii != 0)) {
      // target_line  is Line2
      for (i0 = 0; i0 < ii; i0++) {
        b_Trag_1_1_data[i0] = Trag_1_1_data[1 + index_next * i0];
      }

      dist_left[0] = block[0];
      b_block[0] = block[1];
      c_block[0] = block[2];
      b_point_set_1[0] = block[3];
      dist_left[1] = block[4];
      b_block[1] = block[5];
      c_block[1] = block[6];
      b_point_set_1[1] = block[7];
      fcn_4points(b_Trag_1_1_data, dist_left, b_block, c_block, b_point_set_1,
                  cross_1_data, cross_1_size);
      reshapes[0].f1.size[0] = index_next;
      loop_ub = index_next * ii;
      for (i0 = 0; i0 < loop_ub; i0++) {
        reshapes[0].f1.data[i0] = Trag_1_1_data[i0];
      }

      if ((cross_1_size[0] != 0) && (cross_1_size[1] != 0)) {
        sizes_idx_0 = (signed char)cross_1_size[0];
      } else {
        sizes_idx_0 = 0;
      }

      loop_ub = sizes_idx_0 * ii;
      for (i0 = 0; i0 < loop_ub; i0++) {
        reshapes[1].f1.data[i0] = cross_1_data[i0];
      }

      index_next += sizes_idx_0;
      for (i0 = 0; i0 < ii; i0++) {
        loop_ub = reshapes[0].f1.size[0];
        for (i1 = 0; i1 < loop_ub; i1++) {
          Trag_1_1_data[i1 + index_next * i0] = reshapes[0].f1.data[i1 +
            reshapes[0].f1.size[0] * i0];
        }
      }

      for (i0 = 0; i0 < ii; i0++) {
        loop_ub = sizes_idx_0;
        for (i1 = 0; i1 < loop_ub; i1++) {
          Trag_1_1_data[(i1 + reshapes[0].f1.size[0]) + index_next * i0] =
            reshapes[1].f1.data[i1 + sizes_idx_0 * i0];
        }
      }
    }

    if ((Trag_2_1_size_idx_0 != 0) && (Trag_2_1_size_idx_1 != 0)) {
      // target_line  is Line2
      for (i0 = 0; i0 < Trag_2_1_size_idx_1; i0++) {
        b_Trag_1_1_data[i0] = Trag_2_1_data[1 + Trag_2_1_size_idx_0 * i0];
      }

      dist_left[0] = block[2];
      b_block[0] = block[3];
      c_block[0] = block[0];
      b_point_set_1[0] = block[1];
      dist_left[1] = block[6];
      b_block[1] = block[7];
      c_block[1] = block[4];
      b_point_set_1[1] = block[5];
      fcn_4points(b_Trag_1_1_data, dist_left, b_block, c_block, b_point_set_1,
                  cross_1_data, cross_1_size);
      b_reshapes[0].f1.size[0] = Trag_2_1_size_idx_0;
      loop_ub = Trag_2_1_size_idx_0 * Trag_2_1_size_idx_1;
      for (i0 = 0; i0 < loop_ub; i0++) {
        b_reshapes[0].f1.data[i0] = Trag_2_1_data[i0];
      }

      if ((cross_1_size[0] != 0) && (cross_1_size[1] != 0)) {
        sizes_idx_0 = (signed char)cross_1_size[0];
      } else {
        sizes_idx_0 = 0;
      }

      loop_ub = sizes_idx_0 * Trag_2_1_size_idx_1;
      for (i0 = 0; i0 < loop_ub; i0++) {
        b_reshapes[1].f1.data[i0] = cross_1_data[i0];
      }

      Trag_2_1_size_idx_0 += sizes_idx_0;
      for (i0 = 0; i0 < Trag_2_1_size_idx_1; i0++) {
        loop_ub = b_reshapes[0].f1.size[0];
        for (i1 = 0; i1 < loop_ub; i1++) {
          Trag_2_1_data[i1 + Trag_2_1_size_idx_0 * i0] = b_reshapes[0]
            .f1.data[i1 + b_reshapes[0].f1.size[0] * i0];
        }
      }

      for (i0 = 0; i0 < Trag_2_1_size_idx_1; i0++) {
        loop_ub = sizes_idx_0;
        for (i1 = 0; i1 < loop_ub; i1++) {
          Trag_2_1_data[(i1 + b_reshapes[0].f1.size[0]) + Trag_2_1_size_idx_0 *
            i0] = b_reshapes[1].f1.data[i1 + sizes_idx_0 * i0];
        }
      }
    }
  }

  //  Base on the Trags to build the Trapezoids
  // 1) based on the Triangle to find the final point to consist rectangle;
  // 2) judge whether the final point is in the AREA
  // 3) if it is in, build the Trapezoids;
  // 4) if both exist (two trapezoids), judge the size
  if (situation == 2) {
    //  for first
    if ((index_next != 0) && (ii != 0)) {
      for (i0 = 0; i0 < ii; i0++) {
        b_Trag_1_1_data[i0] = Trag_1_1_data[index_next * i0];
      }

      for (i0 = 0; i0 < ii; i0++) {
        b_point_set_1[i0] = Trag_1_1_data[1 + index_next * i0];
      }

      for (i0 = 0; i0 < ii; i0++) {
        cross_1_data[i0] = Trag_1_1_data[2 + index_next * i0];
      }

      fcn_rectangle(b_Trag_1_1_data, b_point_set_1, cross_1_data, dv0);
      dist_left[0] = dv0[0];
      dist_left[1] = dv0[1];

      //  step1,  get the last point
      //  step2 , check whether the  last point in SHAPE
      if (fcn_inshape(dist_left, block)) {
        Trapezoid_size_idx_0 = (signed char)index_next + 1;
        loop_ub = (signed char)index_next;
        for (i0 = 0; i0 < loop_ub; i0++) {
          Trapezoid_data[i0] = Trag_1_1_data[i0];
        }

        for (i0 = 0; i0 < loop_ub; i0++) {
          Trapezoid_data[i0 + Trapezoid_size_idx_0] = Trag_1_1_data[i0 +
            index_next];
        }

        Trapezoid_data[(signed char)index_next] = dv0[0];
        Trapezoid_data[(signed char)index_next + Trapezoid_size_idx_0] = dv0[1];
      }
    }

    //  for second
    if ((Trag_2_1_size_idx_0 != 0) && (Trag_2_1_size_idx_1 != 0)) {
      for (i0 = 0; i0 < Trag_2_1_size_idx_1; i0++) {
        b_Trag_1_1_data[i0] = Trag_2_1_data[Trag_2_1_size_idx_0 * i0];
      }

      for (i0 = 0; i0 < Trag_2_1_size_idx_1; i0++) {
        b_point_set_1[i0] = Trag_2_1_data[1 + Trag_2_1_size_idx_0 * i0];
      }

      for (i0 = 0; i0 < Trag_2_1_size_idx_1; i0++) {
        cross_1_data[i0] = Trag_2_1_data[2 + Trag_2_1_size_idx_0 * i0];
      }

      fcn_rectangle(b_Trag_1_1_data, b_point_set_1, cross_1_data, dv0);
      dist_left[0] = dv0[0];
      dist_left[1] = dv0[1];

      //  step1,  get the last point
      //  step2 , check whether the  last point in SHAPE
      if (fcn_inshape(dist_left, block)) {
        Trapezoid_size_idx_0 = (signed char)Trag_2_1_size_idx_0 + 1;
        loop_ub = (signed char)Trag_2_1_size_idx_0;
        for (i0 = 0; i0 < loop_ub; i0++) {
          Trapezoid_data[i0] = Trag_2_1_data[i0];
        }

        for (i0 = 0; i0 < loop_ub; i0++) {
          Trapezoid_data[i0 + Trapezoid_size_idx_0] = Trag_2_1_data[i0 +
            Trag_2_1_size_idx_0];
        }

        Trapezoid_data[(signed char)Trag_2_1_size_idx_0] = dv0[0];
        Trapezoid_data[(signed char)Trag_2_1_size_idx_0 + Trapezoid_size_idx_0] =
          dv0[1];
      }
    }
  }

  //  Process the Trapezoid
  for (index_next = 0; index_next < 4; index_next++) {
    if (1 + index_next == 4) {
      ii = 0;
    } else {
      ii = index_next + 1;
    }

    b_point_set_1[0] = Trapezoid_data[index_next];
    cross_1_data[0] = Trapezoid_data[ii];
    dist_points[index_next] = Trapezoid_data[index_next];
    dist_points[index_next + 8] = Trapezoid_data[ii];
    dis_all = Trapezoid_data[index_next + Trapezoid_size_idx_0];
    b_point_set_1[1] = dis_all;
    cross_1_data[1] = Trapezoid_data[ii + Trapezoid_size_idx_0];
    dist_points[index_next + 4] = dis_all;
    dist_points[index_next + 12] = Trapezoid_data[ii + Trapezoid_size_idx_0];
    cum = fcn_dist(b_point_set_1, cross_1_data);
    dist_final[index_next] = cum;
    x[index_next] = cum;
  }

  sort(x, iidx);

  //   size  = [dist_final(index_d(1)), dist_final(index_d(2))];
  //   Point_P = [((dist_points(index_d(1),1))+dist_points(index_d(2),3))/2,...
  //                    ((dist_points(index_d(1),2))+dist_points(index_d(2),4))/2] 
  situation = 0;
  index_next = 0;

  //  init
  if ((iidx[1] == (double)iidx[0] + 1.0) || (iidx[1] == (double)iidx[0] - 1.0))
  {
    index_next = iidx[1] - 1;
    situation = 1;
  }

  if (((iidx[2] == (double)iidx[0] + 1.0) || (iidx[2] == (double)iidx[0] - 1.0))
      && (situation == 0)) {
    index_next = iidx[2] - 1;
    situation = 1;
  }

  if (((iidx[3] == (double)iidx[0] + 1.0) || (iidx[3] == (double)iidx[0] - 1.0))
      && (situation == 0)) {
    index_next = iidx[3] - 1;
  }

  // [index_next]
  situation = iidx[0] - 1;
  size[0] = dist_final[situation];
  size[1] = dist_final[index_next];

  //
  dist[0] = dist_points[situation];
  dis_all = dist_points[iidx[0] + 3];
  dist[4] = dis_all;
  Yaw = dist_points[iidx[0] + 7];
  dist[1] = Yaw;
  dist_tmp = dist_points[iidx[0] + 11];
  dist[5] = dist_tmp;
  dist[2] = dist_points[index_next];
  dist[6] = dist_points[4 + index_next];
  dist[3] = dist_points[8 + index_next];
  dist[7] = dist_points[12 + index_next];
  dist_left[0] = 0.0;
  dist_left[1] = 0.0;
  situation = 0;
  for (index_next = 0; index_next < 4; index_next++) {
    cum = 0.0;
    for (ii = 0; ii < 4; ii++) {
      varargin_1_idx_0 = (dist[index_next] == dist[ii]);
      if ((int)varargin_1_idx_0 > (int)(dist[index_next + 4] == dist[ii + 4])) {
        varargin_1_idx_0 = false;
      }

      cum += (double)varargin_1_idx_0;
    }

    if (cum == 1.0) {
      dist_left[situation] = 1.0 + (double)index_next;

      //  triangle long edge
      situation++;
    }
  }

  situation = (int)dist_left[0];
  index_next = (int)dist_left[1];
  dist_right_idx_0 = (dist[situation - 1] + dist[index_next - 1]) / 2.0;
  dist_right_idx_1 = (dist[situation + 3] + dist[index_next + 3]) / 2.0;

  //  Car final location
  //  Final Yaw Angle
  dist_left[0] = (dist_points[iidx[0] - 1] + Yaw) / 2.0;
  dist_left[1] = (dis_all + dist_tmp) / 2.0;

  //  heading target point
  if (dist_left[0] == dist_right_idx_0) {
    cum = dist_right_idx_1;
    b_sign(&cum);
    Yaw = -1.5707963267948966 * cum;
  } else if (dist_left[1] == dist_right_idx_1) {
    Yaw = 0.0;
  } else {
    Yaw = rt_atan2d_snf(dist_left[1] - dist_right_idx_1, dist_left[0] -
                        dist_right_idx_0);
    if ((Yaw < -1.5707963267948966) && (*Type == 3.0)) {
      Yaw += 3.1415926535897931;
    } else if ((Yaw > 1.5707963267948966) && (*Type == 3.0)) {
      Yaw -= 3.1415926535897931;
    } else {
      guard1 = false;
      if ((Yaw > 0.0) && (*Type == 1.0)) {
        cum = dist_right_idx_1;
        b_sign(&cum);
        if (cum == 1.0) {
          Yaw -= 3.1415926535897931;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1 && ((Yaw < 0.0) && (*Type == 1.0))) {
        dis_all = dist_right_idx_1;
        if (dist_right_idx_1 < 0.0) {
          dis_all = -1.0;
        } else if (dist_right_idx_1 > 0.0) {
          dis_all = 1.0;
        } else {
          if (dist_right_idx_1 == 0.0) {
            dis_all = 0.0;
          }
        }

        if (dis_all == -1.0) {
          Yaw += 3.1415926535897931;
        }
      }
    }
  }

  Point_Target[0] = dist_right_idx_0;
  Point_Target[1] = dist_right_idx_1;
  Point_Target[2] = Yaw;
}

//
// File trailer for find_P.cpp
//
// [EOF]
//
