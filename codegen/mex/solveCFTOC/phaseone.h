/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * phaseone.h
 *
 * Code generation for function 'phaseone'
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
void b_phaseone(const real_T H[2500], const real_T f[50], struct_T *solution,
                e_struct_T *memspace, g_struct_T *workingset,
                f_struct_T *qrmanager, c_struct_T *cholmanager,
                b_struct_T *objective, h_struct_T *options,
                const d_struct_T *runTimeOptions);

void phaseone(const real_T H[2500], const real_T f[50], struct_T *solution,
              e_struct_T *memspace, g_struct_T *workingset,
              f_struct_T *qrmanager, c_struct_T *cholmanager,
              const d_struct_T *runTimeOptions, b_struct_T *objective,
              h_struct_T *options);

/* End of code generation (phaseone.h) */
