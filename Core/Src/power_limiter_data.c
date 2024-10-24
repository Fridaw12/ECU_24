/*
 * power_limiter_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "power_limiter".
 *
 * Model version              : 1.20
 * Simulink Coder version : 9.9 (R2023a) 19-Nov-2022
 * C source code generated on : Thu Jul  6 21:06:00 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "power_limiter.h"

/* Block parameters (default storage) */
P_power_limiter_T power_limiter_P = {
  /* Variable: D_p
   * Referenced by: '<S45>/Derivative Gain'
   */
  0.1,

  /* Variable: I_p
   * Referenced by: '<S50>/Integral Gain'
   */
  100.0,

  /* Variable: N_p
   * Referenced by:
   *   '<S55>/N Copy'
   *   '<S56>/Filter Coefficient'
   */
  10.0,

  /* Variable: P_p
   * Referenced by:
   *   '<S51>/Proportional Gain'
   *   '<S52>/Proportional Gain'
   *   '<S57>/P Copy'
   */
  50.0,

  /* Variable: Ts_p
   * Referenced by: '<S1>/Gain'
   */
  0.01,

  /* Mask Parameter: FilteredDerivativeDiscreteorCon
   * Referenced by: '<S1>/[A,B]'
   */
  0.0,

  /* Mask Parameter: FilteredDerivativeDiscreteorC_p
   * Referenced by: '<S1>/[A,B]'
   */
  0.0,

  /* Mask Parameter: PIDController4_InitialCondition
   * Referenced by: '<S46>/Filter Differentiator TF'
   */
  0.0,

  /* Mask Parameter: PIDController4_InitialConditi_l
   * Referenced by: '<S53>/Integrator'
   */
  0.0,

  /* Mask Parameter: LowPassFilterDiscreteorContinuo
   * Referenced by: '<S2>/K'
   */
  1.0,

  /* Mask Parameter: LowPassFilterDiscreteorContin_k
   * Referenced by: '<S10>/Time constant'
   */
  0.3,

  /* Mask Parameter: FilteredDerivativeDiscreteorC_c
   * Referenced by: '<S5>/Time constant'
   */
  0.05,

  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S13>/Constant'
   */
  2.0,

  /* Mask Parameter: LowPassFilterDiscreteorContin_b
   * Referenced by: '<S10>/Constant'
   */
  1.0,

  /* Mask Parameter: FilteredDerivativeDiscreteorC_i
   * Referenced by: '<S5>/Minimum sampling to time constant ratio'
   */
  5.0,

  /* Expression: -25
   * Referenced by: '<Root>/Gain2'
   */
  -25.0,

  /* Expression: 0
   * Referenced by: '<S14>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S41>/Constant1'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S46>/Filter Den Constant'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant1'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Switch1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Switch'
   */
  0.0,

  /* Expression: 1e3
   * Referenced by: '<Root>/Saturation1'
   */
  1000.0,

  /* Expression: 10
   * Referenced by: '<Root>/Saturation1'
   */
  10.0,

  /* Expression: 2/3
   * Referenced by: '<Root>/Gain12'
   */
  0.66666666666666663,

  /* Expression: 1
   * Referenced by: '<Root>/Saturation3'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Saturation3'
   */
  0.0,

  /* Expression: 0.25
   * Referenced by: '<Root>/Gain1'
   */
  0.25,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S16>/Integrator'
   */
  0.01,

  /* Expression: antiwindupUpperLimit
   * Referenced by: '<S16>/Integrator'
   */
  1.0E+6,

  /* Expression: antiwindupLowerLimit
   * Referenced by: '<S16>/Integrator'
   */
  -1.0E+6,

  /* Expression: windupUpperLimit
   * Referenced by: '<S16>/Saturation'
   */
  0.0,

  /* Expression: windupLowerLimit
   * Referenced by: '<S16>/Saturation'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S1>/Constant'
   */
  0.0,

  /* Computed Parameter: Integrator_gainval_a
   * Referenced by: '<S9>/Integrator'
   */
  0.01,

  /* Expression: antiwindupUpperLimit
   * Referenced by: '<S9>/Integrator'
   */
  0.0,

  /* Expression: antiwindupLowerLimit
   * Referenced by: '<S9>/Integrator'
   */
  0.0,

  /* Expression: windupUpperLimit
   * Referenced by: '<S9>/Saturation'
   */
  0.0,

  /* Expression: windupLowerLimit
   * Referenced by: '<S9>/Saturation'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S41>/Clamping_zero'
   */
  0.0,

  /* Computed Parameter: Tsamp_WtEt
   * Referenced by: '<S48>/Tsamp'
   */
  0.005,

  /* Expression: [1 -1]
   * Referenced by: '<S46>/Filter Differentiator TF'
   */
  { 1.0, -1.0 },

  /* Expression: -1
   * Referenced by: '<Root>/Constant'
   */
  -1.0,

  /* Computed Parameter: Integrator_gainval_b
   * Referenced by: '<S53>/Integrator'
   */
  0.005,

  /* Expression: 25
   * Referenced by: '<Root>/Gain'
   */
  25.0,

  /* Computed Parameter: sampletime_WtEt
   * Referenced by: '<S4>/sample time'
   */
  0.01,

  /* Expression: 1
   * Referenced by: '<Root>/Saturation2'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Saturation2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S4>/Delay Input2'
   */
  0.0,

  /* Computed Parameter: Constant4_Value
   * Referenced by: '<S41>/Constant4'
   */
  1,

  /* Computed Parameter: Constant5_Value
   * Referenced by: '<S41>/Constant5'
   */
  -1,

  /* Computed Parameter: Constant6_Value
   * Referenced by: '<S41>/Constant6'
   */
  1,

  /* Computed Parameter: Constant7_Value
   * Referenced by: '<S41>/Constant7'
   */
  -1
};
