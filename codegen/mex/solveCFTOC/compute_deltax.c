/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_deltax.c
 *
 * Code generation for function 'compute_deltax'
 *
 */

/* Include files */
#include "compute_deltax.h"
#include "fullColLDL2_.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void compute_deltax(const real_T H[2500], struct_T *solution,
                    e_struct_T *memspace, const f_struct_T *qrmanager,
                    c_struct_T *cholmanager, const b_struct_T *objective)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T SCALED_REG_PRIMAL;
  real_T beta1;
  int32_T A_maxDiag_idx;
  int32_T exitg1;
  int32_T idx;
  int32_T mNull_tmp;
  int32_T nVar_tmp;
  int32_T nullStartIdx_tmp;
  char_T TRANSA1;
  char_T TRANSB1;
  char_T UPLO1;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (0 <= nVar_tmp) {
      memset(&solution->searchDir[0], 0, (nVar_tmp + 1) * sizeof(real_T));
    }
  } else {
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution->searchDir[idx] = -objective->grad[idx];
    }
    if (qrmanager->ncols <= 0) {
      if (objective->objtype == 3) {
        SCALED_REG_PRIMAL = 1.4901161193847656E-6 * (real_T)qrmanager->mrows;
        cholmanager->ndims = qrmanager->mrows;
        for (idx = 0; idx <= nVar_tmp; idx++) {
          A_maxDiag_idx = qrmanager->mrows * idx;
          mNull_tmp = 90 * idx;
          for (nullStartIdx_tmp = 0; nullStartIdx_tmp <= nVar_tmp;
               nullStartIdx_tmp++) {
            cholmanager->FMat[mNull_tmp + nullStartIdx_tmp] =
                H[A_maxDiag_idx + nullStartIdx_tmp];
          }
        }
        if (qrmanager->mrows < 1) {
          A_maxDiag_idx = -1;
        } else {
          n_t = (ptrdiff_t)qrmanager->mrows;
          k_t = (ptrdiff_t)91;
          k_t = idamax(&n_t, &cholmanager->FMat[0], &k_t);
          A_maxDiag_idx = (int32_T)k_t - 1;
        }
        cholmanager->regTol_ = muDoubleScalarMax(
            muDoubleScalarAbs(
                cholmanager->FMat[A_maxDiag_idx + 90 * A_maxDiag_idx]) *
                2.2204460492503131E-16,
            muDoubleScalarAbs(SCALED_REG_PRIMAL));
        fullColLDL2_(cholmanager, qrmanager->mrows, SCALED_REG_PRIMAL);
        if (cholmanager->ConvexCheck) {
          idx = 0;
          do {
            exitg1 = 0;
            if (idx <= nVar_tmp) {
              if (cholmanager->FMat[idx + 90 * idx] <= 0.0) {
                cholmanager->info = -idx - 1;
                exitg1 = 1;
              } else {
                idx++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (cholmanager->ndims >= 1) {
            TRANSB1 = 'U';
            TRANSA1 = 'N';
            UPLO1 = 'L';
            n_t = (ptrdiff_t)cholmanager->ndims;
            lda_t = (ptrdiff_t)90;
            k_t = (ptrdiff_t)1;
            dtrsv(&UPLO1, &TRANSA1, &TRANSB1, &n_t, &cholmanager->FMat[0],
                  &lda_t, &solution->searchDir[0], &k_t);
          }
          A_maxDiag_idx = cholmanager->ndims;
          for (idx = 0; idx < A_maxDiag_idx; idx++) {
            solution->searchDir[idx] /= cholmanager->FMat[idx + 90 * idx];
          }
          if (cholmanager->ndims >= 1) {
            TRANSB1 = 'U';
            TRANSA1 = 'T';
            UPLO1 = 'L';
            n_t = (ptrdiff_t)cholmanager->ndims;
            lda_t = (ptrdiff_t)90;
            k_t = (ptrdiff_t)1;
            dtrsv(&UPLO1, &TRANSA1, &TRANSB1, &n_t, &cholmanager->FMat[0],
                  &lda_t, &solution->searchDir[0], &k_t);
          }
        }
      }
    } else {
      nullStartIdx_tmp = 90 * qrmanager->ncols;
      if (objective->objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double[idx] =
              -qrmanager->Q[nVar_tmp + 90 * (qrmanager->ncols + idx)];
        }
        if (qrmanager->mrows >= 1) {
          SCALED_REG_PRIMAL = 1.0;
          beta1 = 0.0;
          TRANSB1 = 'N';
          m_t = (ptrdiff_t)qrmanager->mrows;
          n_t = (ptrdiff_t)mNull_tmp;
          lda_t = (ptrdiff_t)90;
          k_t = (ptrdiff_t)1;
          ldb_t = (ptrdiff_t)1;
          dgemv(&TRANSB1, &m_t, &n_t, &SCALED_REG_PRIMAL,
                &qrmanager->Q[nullStartIdx_tmp], &lda_t,
                &memspace->workspace_double[0], &k_t, &beta1,
                &solution->searchDir[0], &ldb_t);
        }
      } else {
        if (objective->objtype == 3) {
          if (qrmanager->mrows >= 1) {
            SCALED_REG_PRIMAL = 1.0;
            beta1 = 0.0;
            TRANSB1 = 'N';
            TRANSA1 = 'N';
            m_t = (ptrdiff_t)qrmanager->mrows;
            n_t = (ptrdiff_t)mNull_tmp;
            k_t = (ptrdiff_t)qrmanager->mrows;
            lda_t = (ptrdiff_t)qrmanager->mrows;
            ldb_t = (ptrdiff_t)90;
            ldc_t = (ptrdiff_t)141;
            dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &SCALED_REG_PRIMAL,
                  &H[0], &lda_t, &qrmanager->Q[nullStartIdx_tmp], &ldb_t,
                  &beta1, &memspace->workspace_double[0], &ldc_t);
          }
          if (qrmanager->mrows >= 1) {
            SCALED_REG_PRIMAL = 1.0;
            beta1 = 0.0;
            TRANSB1 = 'N';
            TRANSA1 = 'T';
            m_t = (ptrdiff_t)mNull_tmp;
            n_t = (ptrdiff_t)mNull_tmp;
            k_t = (ptrdiff_t)qrmanager->mrows;
            lda_t = (ptrdiff_t)90;
            ldb_t = (ptrdiff_t)141;
            ldc_t = (ptrdiff_t)90;
            dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &SCALED_REG_PRIMAL,
                  &qrmanager->Q[nullStartIdx_tmp], &lda_t,
                  &memspace->workspace_double[0], &ldb_t, &beta1,
                  &cholmanager->FMat[0], &ldc_t);
          }
        }
        SCALED_REG_PRIMAL = 1.4901161193847656E-6 * (real_T)mNull_tmp;
        cholmanager->ndims = mNull_tmp;
        n_t = (ptrdiff_t)mNull_tmp;
        k_t = (ptrdiff_t)91;
        k_t = idamax(&n_t, &cholmanager->FMat[0], &k_t);
        cholmanager->regTol_ = muDoubleScalarMax(
            muDoubleScalarAbs(
                cholmanager
                    ->FMat[((int32_T)k_t + 90 * ((int32_T)k_t - 1)) - 1]) *
                2.2204460492503131E-16,
            SCALED_REG_PRIMAL);
        fullColLDL2_(cholmanager, mNull_tmp, SCALED_REG_PRIMAL);
        if (cholmanager->ConvexCheck) {
          idx = 0;
          do {
            exitg1 = 0;
            if (idx <= mNull_tmp - 1) {
              if (cholmanager->FMat[idx + 90 * idx] <= 0.0) {
                cholmanager->info = -idx - 1;
                exitg1 = 1;
              } else {
                idx++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows >= 1) {
            SCALED_REG_PRIMAL = -1.0;
            beta1 = 0.0;
            TRANSB1 = 'T';
            m_t = (ptrdiff_t)qrmanager->mrows;
            n_t = (ptrdiff_t)mNull_tmp;
            lda_t = (ptrdiff_t)90;
            k_t = (ptrdiff_t)1;
            ldb_t = (ptrdiff_t)1;
            dgemv(&TRANSB1, &m_t, &n_t, &SCALED_REG_PRIMAL,
                  &qrmanager->Q[nullStartIdx_tmp], &lda_t, &objective->grad[0],
                  &k_t, &beta1, &memspace->workspace_double[0], &ldb_t);
          }
          if (cholmanager->ndims >= 1) {
            TRANSB1 = 'U';
            TRANSA1 = 'N';
            UPLO1 = 'L';
            n_t = (ptrdiff_t)cholmanager->ndims;
            lda_t = (ptrdiff_t)90;
            k_t = (ptrdiff_t)1;
            dtrsv(&UPLO1, &TRANSA1, &TRANSB1, &n_t, &cholmanager->FMat[0],
                  &lda_t, &memspace->workspace_double[0], &k_t);
          }
          A_maxDiag_idx = cholmanager->ndims;
          for (idx = 0; idx < A_maxDiag_idx; idx++) {
            memspace->workspace_double[idx] /=
                cholmanager->FMat[idx + 90 * idx];
          }
          if (cholmanager->ndims >= 1) {
            TRANSB1 = 'U';
            TRANSA1 = 'T';
            UPLO1 = 'L';
            n_t = (ptrdiff_t)cholmanager->ndims;
            lda_t = (ptrdiff_t)90;
            k_t = (ptrdiff_t)1;
            dtrsv(&UPLO1, &TRANSA1, &TRANSB1, &n_t, &cholmanager->FMat[0],
                  &lda_t, &memspace->workspace_double[0], &k_t);
          }
          if (qrmanager->mrows >= 1) {
            SCALED_REG_PRIMAL = 1.0;
            beta1 = 0.0;
            TRANSB1 = 'N';
            m_t = (ptrdiff_t)qrmanager->mrows;
            n_t = (ptrdiff_t)mNull_tmp;
            lda_t = (ptrdiff_t)90;
            k_t = (ptrdiff_t)1;
            ldb_t = (ptrdiff_t)1;
            dgemv(&TRANSB1, &m_t, &n_t, &SCALED_REG_PRIMAL,
                  &qrmanager->Q[nullStartIdx_tmp], &lda_t,
                  &memspace->workspace_double[0], &k_t, &beta1,
                  &solution->searchDir[0], &ldb_t);
          }
        }
      }
    }
  }
}

/* End of code generation (compute_deltax.c) */
