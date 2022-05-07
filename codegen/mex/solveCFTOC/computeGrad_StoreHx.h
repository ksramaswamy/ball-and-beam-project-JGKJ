/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeGrad_StoreHx.h
 *
 * Code generation for function 'computeGrad_StoreHx'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "solveCFTOC_internal_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void computeGrad_StoreHx(b_struct_T *obj, const real_T H[2500],
                         const real_T f[50], const real_T x[51]);

/* End of code generation (computeGrad_StoreHx.h) */
