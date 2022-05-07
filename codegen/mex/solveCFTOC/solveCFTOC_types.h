/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveCFTOC_types.h
 *
 * Code generation for function 'solveCFTOC'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  real_T FMat[8100];
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  real_T workspace_[4320];
  real_T workspace2_[4320];
} c_struct_T;
#endif /* typedef_c_struct_T */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T
typedef struct {
  int32_T ldq;
  real_T QR[12690];
  real_T Q[8100];
  int32_T jpvt[141];
  int32_T mrows;
  int32_T ncols;
  real_T tau[90];
  int32_T minRowCol;
  boolean_T usedPivoting;
} f_struct_T;
#endif /* typedef_f_struct_T */

#ifndef typedef_b_solveCFTOC
#define typedef_b_solveCFTOC
typedef struct {
  f_struct_T QRManager;
  c_struct_T CholRegManager;
} b_solveCFTOC;
#endif /* typedef_b_solveCFTOC */

#ifndef typedef_solveCFTOCStackData
#define typedef_solveCFTOCStackData
typedef struct {
  b_solveCFTOC f0;
} solveCFTOCStackData;
#endif /* typedef_solveCFTOCStackData */

/* End of code generation (solveCFTOC_types.h) */
