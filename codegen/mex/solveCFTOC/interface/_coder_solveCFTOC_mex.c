/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_solveCFTOC_mex.c
 *
 * Code generation for function '_coder_solveCFTOC_mex'
 *
 */

/* Include files */
#include "_coder_solveCFTOC_mex.h"
#include "_coder_solveCFTOC_api.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_data.h"
#include "solveCFTOC_initialize.h"
#include "solveCFTOC_terminate.h"
#include "solveCFTOC_types.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  solveCFTOCStackData *solveCFTOCStackDataGlobal = NULL;
  solveCFTOCStackDataGlobal = (solveCFTOCStackData *)emlrtMxCalloc(
      (size_t)1, (size_t)1U * sizeof(solveCFTOCStackData));
  mexAtExit(&solveCFTOC_atexit);
  /* Module initialization. */
  solveCFTOC_initialize();
  /* Dispatch the entry-point. */
  unsafe_solveCFTOC_mexFunction(solveCFTOCStackDataGlobal, nlhs, plhs, nrhs,
                                prhs);
  /* Module termination. */
  solveCFTOC_terminate();
  emlrtMxFree(solveCFTOCStackDataGlobal);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

void unsafe_solveCFTOC_mexFunction(solveCFTOCStackData *SD, int32_T nlhs,
                                   mxArray *plhs[1], int32_T nrhs,
                                   const mxArray *prhs[7])
{
  const mxArray *outputs;
  /* Check for proper number of arguments. */
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 7, 4, 10, "solveCFTOC");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 10,
                        "solveCFTOC");
  }
  /* Call the function. */
  solveCFTOC_api(SD, prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/* End of code generation (_coder_solveCFTOC_mex.c) */
