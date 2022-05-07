/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveCFTOC.h
 *
 * Code generation for function 'solveCFTOC'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "solveCFTOC_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void solveCFTOC(solveCFTOCStackData *SD, const real_T H[2500],
                const real_T f[50], const real_T A[2000], const real_T b[40],
                const real_T lb[50], const real_T ub[50], const real_T x0[50],
                real_T x[50]);

/* End of code generation (solveCFTOC.h) */
