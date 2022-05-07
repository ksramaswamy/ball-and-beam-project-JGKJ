/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeFirstOrderOpt.h
 *
 * Code generation for function 'computeFirstOrderOpt'
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
void computeFirstOrderOpt(struct_T *solution, const b_struct_T *objective,
                          int32_T workingset_nVar,
                          const real_T workingset_ATwset[7191],
                          int32_T workingset_nActiveConstr,
                          real_T workspace[7191]);

/* End of code generation (computeFirstOrderOpt.h) */
