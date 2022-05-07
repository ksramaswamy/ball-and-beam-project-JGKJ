/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_lambda.h
 *
 * Code generation for function 'compute_lambda'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void compute_lambda(real_T workspace[7191], struct_T *solution,
                    const b_struct_T *objective, const f_struct_T *qrmanager);

/* End of code generation (compute_lambda.h) */
