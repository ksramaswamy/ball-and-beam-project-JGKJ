/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * PresolveWorkingSet.h
 *
 * Code generation for function 'PresolveWorkingSet'
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
void PresolveWorkingSet(struct_T *solution, e_struct_T *memspace,
                        g_struct_T *workingset, f_struct_T *qrmanager);

void b_PresolveWorkingSet(struct_T *solution, e_struct_T *memspace,
                          g_struct_T *workingset, f_struct_T *qrmanager);

/* End of code generation (PresolveWorkingSet.h) */
