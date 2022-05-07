/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeFval.h
 *
 * Code generation for function 'computeFval'
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
real_T computeFval(const b_struct_T *obj, real_T workspace[7191],
                   const real_T H[2500], const real_T f[50],
                   const real_T x[51]);

/* End of code generation (computeFval.h) */
