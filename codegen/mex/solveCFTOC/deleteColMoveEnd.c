/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * deleteColMoveEnd.c
 *
 * Code generation for function 'deleteColMoveEnd'
 *
 */

/* Include files */
#include "deleteColMoveEnd.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_types.h"
#include "blas.h"
#include "mwmathutil.h"

/* Function Definitions */
void deleteColMoveEnd(f_struct_T *obj, int32_T idx)
{
  real_T b_temp_tmp;
  real_T c;
  real_T s;
  real_T temp_tmp;
  int32_T QRk0;
  int32_T b_i;
  int32_T b_k;
  int32_T c_temp_tmp;
  int32_T endIdx;
  int32_T i;
  int32_T idxRotGCol_tmp;
  int32_T k;
  int32_T n;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
      i++;
    }
    idx = i;
  }
  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    obj->jpvt[idx - 1] = obj->jpvt[obj->ncols - 1];
    b_i = obj->minRowCol;
    for (k = 0; k < b_i; k++) {
      obj->QR[k + 90 * (idx - 1)] = obj->QR[k + 90 * (obj->ncols - 1)];
    }
    obj->ncols--;
    obj->minRowCol = muIntScalarMin_sint32(obj->mrows, obj->ncols);
    if (idx < obj->mrows) {
      i = obj->mrows - 1;
      endIdx = muIntScalarMin_sint32(i, obj->ncols);
      k = endIdx;
      i = 90 * (idx - 1);
      while (k >= idx) {
        b_i = k + i;
        temp_tmp = obj->QR[b_i - 1];
        b_temp_tmp = obj->QR[b_i];
        c = 0.0;
        s = 0.0;
        drotg(&temp_tmp, &b_temp_tmp, &c, &s);
        obj->QR[b_i - 1] = temp_tmp;
        obj->QR[b_i] = b_temp_tmp;
        b_i = 90 * (k - 1);
        obj->QR[k + b_i] = 0.0;
        QRk0 = k + 90 * idx;
        n = obj->ncols - idx;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            c_temp_tmp = QRk0 + b_k * 90;
            temp_tmp = obj->QR[c_temp_tmp];
            b_temp_tmp = obj->QR[c_temp_tmp - 1];
            obj->QR[c_temp_tmp] = c * temp_tmp - s * b_temp_tmp;
            obj->QR[c_temp_tmp - 1] = c * b_temp_tmp + s * temp_tmp;
          }
        }
        n = obj->mrows;
        if (obj->mrows >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            c_temp_tmp = b_i + b_k;
            temp_tmp = obj->Q[c_temp_tmp + 90];
            b_temp_tmp = obj->Q[c_temp_tmp];
            obj->Q[c_temp_tmp + 90] = c * temp_tmp - s * b_temp_tmp;
            obj->Q[c_temp_tmp] = c * b_temp_tmp + s * temp_tmp;
          }
        }
        k--;
      }
      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        idxRotGCol_tmp = 90 * (k - 1);
        i = k + idxRotGCol_tmp;
        temp_tmp = obj->QR[i - 1];
        b_temp_tmp = obj->QR[i];
        c = 0.0;
        s = 0.0;
        drotg(&temp_tmp, &b_temp_tmp, &c, &s);
        obj->QR[i - 1] = temp_tmp;
        obj->QR[i] = b_temp_tmp;
        QRk0 = k * 91;
        n = obj->ncols - k;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            c_temp_tmp = QRk0 + b_k * 90;
            temp_tmp = obj->QR[c_temp_tmp];
            b_temp_tmp = obj->QR[c_temp_tmp - 1];
            obj->QR[c_temp_tmp] = c * temp_tmp - s * b_temp_tmp;
            obj->QR[c_temp_tmp - 1] = c * b_temp_tmp + s * temp_tmp;
          }
        }
        n = obj->mrows;
        if (obj->mrows >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            c_temp_tmp = idxRotGCol_tmp + b_k;
            temp_tmp = obj->Q[c_temp_tmp + 90];
            b_temp_tmp = obj->Q[c_temp_tmp];
            obj->Q[c_temp_tmp + 90] = c * temp_tmp - s * b_temp_tmp;
            obj->Q[c_temp_tmp] = c * b_temp_tmp + s * temp_tmp;
          }
        }
      }
    }
  }
}

/* End of code generation (deleteColMoveEnd.c) */
