/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * driver.h
 *
 * Code generation for function 'driver'
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
void driver(const real_T H[2500], const real_T f[50], struct_T *solution,
            e_struct_T *memspace, g_struct_T *workingset,
            c_struct_T *cholmanager, d_struct_T runTimeOptions,
            f_struct_T *qrmanager, b_struct_T *objective);

/* End of code generation (driver.h) */
