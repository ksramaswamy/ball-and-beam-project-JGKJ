/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeFval_ReuseHx.c
 *
 * Code generation for function 'computeFval_ReuseHx'
 *
 */

/* Include files */
#include "computeFval_ReuseHx.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include <string.h>

/* Function Definitions */
real_T computeFval_ReuseHx(const b_struct_T *obj, real_T workspace[7191],
                           const real_T f[50], const real_T x[51])
{
  real_T val;
  int32_T ixlast;
  int32_T k;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;
  case 3:
    if (obj->hasLinear) {
      ixlast = obj->nvar;
      for (k = 0; k < ixlast; k++) {
        workspace[k] = 0.5 * obj->Hx[k] + f[k];
      }
      val = 0.0;
      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        for (k = 0; k < ixlast; k++) {
          val += x[k] * workspace[k];
        }
      }
    } else {
      val = 0.0;
      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        for (k = 0; k < ixlast; k++) {
          val += x[k] * obj->Hx[k];
        }
      }
      val *= 0.5;
    }
    break;
  default:
    if (obj->hasLinear) {
      ixlast = obj->nvar;
      if (0 <= ixlast - 1) {
        memcpy(&workspace[0], &f[0], ixlast * sizeof(real_T));
      }
      ixlast = 49 - obj->nvar;
      for (k = 0; k <= ixlast; k++) {
        workspace[obj->nvar + k] = 0.0;
      }
      val = 0.0;
      for (k = 0; k < 50; k++) {
        workspace[k] += 0.5 * obj->Hx[k];
        val += x[k] * workspace[k];
      }
    } else {
      val = 0.0;
      for (k = 0; k < 50; k++) {
        val += x[k] * obj->Hx[k];
      }
      val *= 0.5;
      ixlast = obj->nvar + 1;
      for (k = ixlast; k < 51; k++) {
        val += x[k - 1] * 0.0;
      }
    }
    break;
  }
  return val;
}

/* End of code generation (computeFval_ReuseHx.c) */
