/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_lambda.c
 *
 * Code generation for function 'compute_lambda'
 *
 */

/* Include files */
#include "compute_lambda.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Function Definitions */
void compute_lambda(real_T workspace[7191], struct_T *solution,
                    const b_struct_T *objective, const f_struct_T *qrmanager)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T beta1;
  real_T tol;
  int32_T idx;
  int32_T idxQR;
  int32_T nActiveConstr_tmp;
  char_T TRANSA;
  char_T TRANSA1;
  char_T UPLO1;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T nonDegenerate;
  nActiveConstr_tmp = qrmanager->ncols;
  if (qrmanager->ncols > 0) {
    guard1 = false;
    if (objective->objtype != 4) {
      tol = 100.0 * (real_T)qrmanager->mrows * 2.2204460492503131E-16;
      if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
        nonDegenerate = true;
      } else {
        nonDegenerate = false;
      }
      if (nonDegenerate) {
        idx = nActiveConstr_tmp;
        guard2 = false;
        if (qrmanager->mrows < qrmanager->ncols) {
          idxQR = qrmanager->mrows + 90 * (qrmanager->ncols - 1);
          while ((idx > qrmanager->mrows) &&
                 (muDoubleScalarAbs(qrmanager->QR[idxQR - 1]) >= tol)) {
            idx--;
            idxQR -= 90;
          }
          nonDegenerate = (idx == qrmanager->mrows);
          if (nonDegenerate) {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          idxQR = idx + 90 * (idx - 1);
          while ((idx >= 1) &&
                 (muDoubleScalarAbs(qrmanager->QR[idxQR - 1]) >= tol)) {
            idx--;
            idxQR -= 91;
          }
          nonDegenerate = (idx == 0);
        }
      }
      if (!nonDegenerate) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      if (qrmanager->mrows >= 1) {
        tol = 1.0;
        beta1 = 0.0;
        TRANSA = 'T';
        m_t = (ptrdiff_t)qrmanager->mrows;
        n_t = (ptrdiff_t)qrmanager->ncols;
        lda_t = (ptrdiff_t)90;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dgemv(&TRANSA, &m_t, &n_t, &tol, &qrmanager->Q[0], &lda_t,
              &objective->grad[0], &incx_t, &beta1, &workspace[0], &incy_t);
      }
      TRANSA = 'N';
      TRANSA1 = 'N';
      UPLO1 = 'U';
      n_t = (ptrdiff_t)qrmanager->ncols;
      lda_t = (ptrdiff_t)90;
      incx_t = (ptrdiff_t)1;
      dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t, &qrmanager->QR[0], &lda_t,
            &workspace[0], &incx_t);
      for (idx = 0; idx < nActiveConstr_tmp; idx++) {
        solution->lambda[idx] = -workspace[idx];
      }
    }
  }
}

/* End of code generation (compute_lambda.c) */
