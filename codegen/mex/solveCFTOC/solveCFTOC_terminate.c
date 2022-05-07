/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveCFTOC_terminate.c
 *
 * Code generation for function 'solveCFTOC_terminate'
 *
 */

/* Include files */
#include "solveCFTOC_terminate.h"
#include "_coder_solveCFTOC_mex.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_data.h"

/* Function Definitions */
void solveCFTOC_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void solveCFTOC_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (solveCFTOC_terminate.c) */
