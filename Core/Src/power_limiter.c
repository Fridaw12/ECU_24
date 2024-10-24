/*
 * power_limiter.c
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
#include "rt_nonfinite.h"
#include <math.h>
#include "rtwtypes.h"
#include <string.h>

/* Block signals (default storage) */
B_power_limiter_T power_limiter_B;

/* Block states (default storage) */
DW_power_limiter_T power_limiter_DW;

/* External inputs (root inport signals with default storage) */
ExtU_power_limiter_T power_limiter_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_power_limiter_T power_limiter_Y;

/* Real-time model */
static RT_MODEL_power_limiter_T power_limiter_M_;
RT_MODEL_power_limiter_T *const power_limiter_M = &power_limiter_M_;

/* Model step function */
void power_limiter_step(void)
{
  real_T rtb_Abs1;
  real_T rtb_Abs2;
  real_T rtb_DerivativeGain_tmp;
  real_T rtb_IntegralGain;
  real_T rtb_Integrator_m;
  real_T rtb_K;
  real_T rtb_Max;
  real_T rtb_Saturation;
  real_T rtb_Saturation2;
  real_T rtb_Saturation3;
  real_T rtb_Sign;
  real_T u0;
  int32_T i;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T rtb_LogicalOperator;
  boolean_T rtb_RelationalOperator;
  boolean_T rtb_RelationalOperator1;
  boolean_T rtb_fixforDTpropagationissue;

  /* Signum: '<Root>/Sign' incorporates:
   *  Inport: '<Root>/m_ref'
   */
//  if (rtIsNaN(power_limiter_U.m_ref)) {
//    rtb_Sign = (rtNaN);
//  } else
  if (power_limiter_U.m_ref < 0.0) {
    rtb_Sign = -1.0;
  } else {
    rtb_Sign = (power_limiter_U.m_ref > 0.0);
  }

  /* End of Signum: '<Root>/Sign' */

  /* Switch: '<Root>/Switch1' incorporates:
   *  Inport: '<Root>/p_limn'
   *  Inport: '<Root>/p_limp'
   */
  if (rtb_Sign > power_limiter_P.Switch1_Threshold) {
    u0 = power_limiter_U.p_limp;
  } else {
    u0 = power_limiter_U.p_limn;
  }

  /* Switch: '<Root>/Switch' incorporates:
   *  Inport: '<Root>/i_limn_ams'
   *  Inport: '<Root>/i_limp_ams'
   */
  if (rtb_Sign > power_limiter_P.Switch_Threshold) {
    rtb_Abs2 = power_limiter_U.i_limp_ams;
  } else {
    rtb_Abs2 = power_limiter_U.i_limn_ams;
  }

  /* MinMax: '<Root>/Min2' incorporates:
   *  Inport: '<Root>/i_lim_fuse'
   *  Inport: '<Root>/u_dc_mcu'
   *  Product: '<Root>/Product3'
   *  Product: '<Root>/Product4'
   *  Switch: '<Root>/Switch'
   *  Switch: '<Root>/Switch1'
   */
  rtb_IntegralGain = fmin(fmin(u0, rtb_Abs2 * power_limiter_U.u_dc_mcu),
    power_limiter_U.i_lim_fuse * power_limiter_U.u_dc_mcu);

  /* Abs: '<Root>/Abs' incorporates:
   *  Inport: '<Root>/u_q_mcu'
   */
  rtb_Abs2 = fabs(power_limiter_U.u_q_mcu);

  /* Saturate: '<Root>/Saturation1' */
  if (rtb_Abs2 > power_limiter_P.Saturation1_UpperSat) {
    rtb_Abs2 = power_limiter_P.Saturation1_UpperSat;
  } else if (rtb_Abs2 < power_limiter_P.Saturation1_LowerSat) {
    rtb_Abs2 = power_limiter_P.Saturation1_LowerSat;
  }

  /* End of Saturate: '<Root>/Saturation1' */

  /* Gain: '<Root>/Gain12' incorporates:
   *  Inport: '<Root>/i_q_max'
   *  Product: '<Root>/Product7'
   */
  rtb_Saturation3 = rtb_IntegralGain / rtb_Abs2 / power_limiter_U.i_q_max *
    power_limiter_P.Gain12_Gain;

  /* Saturate: '<Root>/Saturation3' */
  if (rtb_Saturation3 > power_limiter_P.Saturation3_UpperSat) {
    rtb_Saturation3 = power_limiter_P.Saturation3_UpperSat;
  } else if (rtb_Saturation3 < power_limiter_P.Saturation3_LowerSat) {
    rtb_Saturation3 = power_limiter_P.Saturation3_LowerSat;
  }

  /* End of Saturate: '<Root>/Saturation3' */

  /* MinMax: '<Root>/Min' incorporates:
   *  Constant: '<Root>/Constant1'
   *  Gain: '<Root>/Gain1'
   *  Sum: '<Root>/Add1'
   */
  rtb_Saturation2 = fmin(power_limiter_P.Constant1_Value_l - rtb_Saturation3,
    power_limiter_P.Gain1_Gain * rtb_Saturation3);

  /* Logic: '<S10>/Logical Operator' incorporates:
   *  Constant: '<S10>/Constant'
   *  Constant: '<S10>/Time constant'
   *  Constant: '<S13>/Constant'
   *  Constant: '<S14>/Constant'
   *  RelationalOperator: '<S13>/Compare'
   *  RelationalOperator: '<S14>/Compare'
   *  Sum: '<S10>/Sum1'
   */
  rtb_LogicalOperator = ((power_limiter_P.LowPassFilterDiscreteorContin_k -
    power_limiter_B.Probe[0] <= power_limiter_P.Constant_Value) &&
    (power_limiter_P.LowPassFilterDiscreteorContin_b <
     power_limiter_P.CompareToConstant_const));

  /* Delay: '<Root>/Delay' incorporates:
   *  Inport: '<Root>/p_mcu'
   */
  if (power_limiter_DW.icLoad) {
    for (i = 0; i < 10; i++) {
      power_limiter_DW.Delay_DSTATE[i] = power_limiter_U.p_mcu;
    }
  }

  /* Gain: '<S2>/K' incorporates:
   *  Delay: '<Root>/Delay'
   *  Inport: '<Root>/i_dc_ams'
   *  Inport: '<Root>/u_dc_ams'
   *  Product: '<Root>/Product2'
   *  Sum: '<Root>/Add4'
   */
  rtb_K = (power_limiter_U.u_dc_ams * power_limiter_U.i_dc_ams -
           power_limiter_DW.Delay_DSTATE[0]) *
    power_limiter_P.LowPassFilterDiscreteorContinuo;

  /* DiscreteIntegrator: '<S16>/Integrator' */
  if (power_limiter_DW.Integrator_IC_LOADING != 0) {
    power_limiter_DW.Integrator_DSTATE = rtb_K;
    if (power_limiter_DW.Integrator_DSTATE >=
        power_limiter_P.Integrator_UpperSat) {
      power_limiter_DW.Integrator_DSTATE = power_limiter_P.Integrator_UpperSat;
    } else if (power_limiter_DW.Integrator_DSTATE <=
               power_limiter_P.Integrator_LowerSat) {
      power_limiter_DW.Integrator_DSTATE = power_limiter_P.Integrator_LowerSat;
    }
  }

  if (rtb_LogicalOperator || (power_limiter_DW.Integrator_PrevResetState != 0))
  {
    power_limiter_DW.Integrator_DSTATE = rtb_K;
    if (power_limiter_DW.Integrator_DSTATE >=
        power_limiter_P.Integrator_UpperSat) {
      power_limiter_DW.Integrator_DSTATE = power_limiter_P.Integrator_UpperSat;
    } else if (power_limiter_DW.Integrator_DSTATE <=
               power_limiter_P.Integrator_LowerSat) {
      power_limiter_DW.Integrator_DSTATE = power_limiter_P.Integrator_LowerSat;
    }
  }

  if (power_limiter_DW.Integrator_DSTATE >= power_limiter_P.Integrator_UpperSat)
  {
    power_limiter_DW.Integrator_DSTATE = power_limiter_P.Integrator_UpperSat;
  } else if (power_limiter_DW.Integrator_DSTATE <=
             power_limiter_P.Integrator_LowerSat) {
    power_limiter_DW.Integrator_DSTATE = power_limiter_P.Integrator_LowerSat;
  }

  /* Saturate: '<S16>/Saturation' incorporates:
   *  DiscreteIntegrator: '<S16>/Integrator'
   */
  if (power_limiter_DW.Integrator_DSTATE > power_limiter_P.Saturation_UpperSat)
  {
    rtb_Saturation = power_limiter_P.Saturation_UpperSat;
  } else if (power_limiter_DW.Integrator_DSTATE <
             power_limiter_P.Saturation_LowerSat) {
    rtb_Saturation = power_limiter_P.Saturation_LowerSat;
  } else {
    rtb_Saturation = power_limiter_DW.Integrator_DSTATE;
  }

  /* End of Saturate: '<S16>/Saturation' */

  /* Abs: '<Root>/Abs1' incorporates:
   *  Inport: '<Root>/p_mcu'
   *  Sum: '<Root>/Add2'
   */
  rtb_Abs1 = fabs(power_limiter_U.p_mcu + rtb_Saturation);

  /* MinMax: '<S5>/MinMax' incorporates:
   *  Constant: '<S5>/Time constant'
   *  Gain: '<S5>/Minimum sampling to time constant ratio'
   */
  rtb_Integrator_m = fmax(power_limiter_P.FilteredDerivativeDiscreteorC_i *
    power_limiter_B.Probe_e[0], power_limiter_P.FilteredDerivativeDiscreteorC_c);

  /* DiscreteIntegrator: '<S9>/Integrator' incorporates:
   *  Constant: '<S1>/Constant'
   */
  if (power_limiter_DW.Integrator_IC_LOADING_g != 0) {
    power_limiter_DW.Integrator_DSTATE_m = rtb_Abs1;
    if (power_limiter_DW.Integrator_DSTATE_m >=
        power_limiter_P.Integrator_UpperSat_f) {
      power_limiter_DW.Integrator_DSTATE_m =
        power_limiter_P.Integrator_UpperSat_f;
    } else if (power_limiter_DW.Integrator_DSTATE_m <=
               power_limiter_P.Integrator_LowerSat_a) {
      power_limiter_DW.Integrator_DSTATE_m =
        power_limiter_P.Integrator_LowerSat_a;
    }
  }

  if ((power_limiter_P.Constant_Value_h != 0.0) ||
      (power_limiter_DW.Integrator_PrevResetState_d != 0)) {
    power_limiter_DW.Integrator_DSTATE_m = rtb_Abs1;
    if (power_limiter_DW.Integrator_DSTATE_m >=
        power_limiter_P.Integrator_UpperSat_f) {
      power_limiter_DW.Integrator_DSTATE_m =
        power_limiter_P.Integrator_UpperSat_f;
    } else if (power_limiter_DW.Integrator_DSTATE_m <=
               power_limiter_P.Integrator_LowerSat_a) {
      power_limiter_DW.Integrator_DSTATE_m =
        power_limiter_P.Integrator_LowerSat_a;
    }
  }

  if (power_limiter_DW.Integrator_DSTATE_m >=
      power_limiter_P.Integrator_UpperSat_f) {
    power_limiter_DW.Integrator_DSTATE_m = power_limiter_P.Integrator_UpperSat_f;
  } else if (power_limiter_DW.Integrator_DSTATE_m <=
             power_limiter_P.Integrator_LowerSat_a) {
    power_limiter_DW.Integrator_DSTATE_m = power_limiter_P.Integrator_LowerSat_a;
  }

  /* Saturate: '<S9>/Saturation' incorporates:
   *  DiscreteIntegrator: '<S9>/Integrator'
   */
  if (power_limiter_DW.Integrator_DSTATE_m >
      power_limiter_P.Saturation_UpperSat_n) {
    u0 = power_limiter_P.Saturation_UpperSat_n;
  } else if (power_limiter_DW.Integrator_DSTATE_m <
             power_limiter_P.Saturation_LowerSat_j) {
    u0 = power_limiter_P.Saturation_LowerSat_j;
  } else {
    u0 = power_limiter_DW.Integrator_DSTATE_m;
  }

  /* Product: '<S1>/1//T' incorporates:
   *  Fcn: '<S5>/Avoid Divide by Zero'
   *  Saturate: '<S9>/Saturation'
   *  Sum: '<S1>/Sum1'
   */
  rtb_Integrator_m = 1.0 / ((real_T)(rtb_Integrator_m == 0.0) *
    2.2204460492503131e-16 + rtb_Integrator_m) * (rtb_Abs1 - u0);

  /* Gain: '<S1>/Gain' */
  u0 = power_limiter_P.Ts_p * rtb_Integrator_m;

  /* Saturate: '<S1>/[A,B]' */
  if (u0 > power_limiter_P.FilteredDerivativeDiscreteorC_p) {
    u0 = power_limiter_P.FilteredDerivativeDiscreteorC_p;
  } else if (u0 < power_limiter_P.FilteredDerivativeDiscreteorCon) {
    u0 = power_limiter_P.FilteredDerivativeDiscreteorCon;
  }

  /* Product: '<Root>/Product5' incorporates:
   *  Inport: '<Root>/p_max'
   *  Inport: '<Root>/u_dc_mcu'
   *  Inport: '<Root>/u_dc_nom'
   *  Product: '<Root>/Product1'
   *  Saturate: '<S1>/[A,B]'
   *  Sum: '<Root>/Add'
   *  Sum: '<Root>/Add3'
   */
  rtb_Abs2 = (rtb_IntegralGain - (rtb_Abs1 + u0)) * (1.0 / power_limiter_U.p_max)
    / rtb_Abs2 * (power_limiter_U.u_dc_mcu / power_limiter_U.u_dc_nom);

  /* SampleTimeMath: '<S48>/Tsamp' incorporates:
   *  Constant: '<S55>/N Copy'
   *
   * About '<S48>/Tsamp':
   *  y = u * K where K = ( w * Ts )
   */
  rtb_Max = power_limiter_P.N_p * power_limiter_P.Tsamp_WtEt;

  /* Math: '<S46>/Reciprocal' incorporates:
   *  Constant: '<S46>/Filter Den Constant'
   *  Sum: '<S46>/SumDen'
   *
   * About '<S46>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_IntegralGain = 1.0 / (power_limiter_P.FilterDenConstant_Value + rtb_Max);

  /* SignalConversion generated from: '<S46>/Filter Differentiator TF' incorporates:
   *  Constant: '<S46>/Filter Den Constant'
   *  Product: '<S46>/Divide'
   *  Sum: '<S46>/SumNum'
   */
  power_limiter_B.TmpSignalConversionAtFilterDiff[0] =
    power_limiter_P.FilterDenConstant_Value;
  power_limiter_B.TmpSignalConversionAtFilterDiff[1] = (rtb_Max -
    power_limiter_P.FilterDenConstant_Value) * rtb_IntegralGain;

  /* DiscreteTransferFcn: '<S46>/Filter Differentiator TF' incorporates:
   *  Gain: '<S45>/Derivative Gain'
   */
  rtb_Abs1 = power_limiter_P.D_p * rtb_Abs2 -
    power_limiter_B.TmpSignalConversionAtFilterDiff[1] * power_limiter_DW.p_filt;

  /* Gain: '<S56>/Filter Coefficient' incorporates:
   *  DiscreteTransferFcn: '<S46>/Filter Differentiator TF'
   *  Product: '<S46>/DenCoefOut'
   */
  rtb_Max = (power_limiter_P.FilterDifferentiatorTF_NumCoef[0] * rtb_Abs1 +
             power_limiter_P.FilterDifferentiatorTF_NumCoef[1] *
             power_limiter_DW.p_filt) * rtb_IntegralGain * power_limiter_P.N_p;

  /* Gain: '<S52>/Proportional Gain' incorporates:
   *  Sum: '<S64>/Sum Fdbk'
   */
  rtb_IntegralGain = ((rtb_Abs2 + power_limiter_DW.p_int) + rtb_Max) *
    power_limiter_P.P_p;

  /* Switch: '<S44>/Switch' incorporates:
   *  Constant: '<Root>/Constant'
   *  RelationalOperator: '<S44>/u_GTE_up'
   *  RelationalOperator: '<S44>/u_GT_lo'
   *  Switch: '<S44>/Switch1'
   */
  if (rtb_IntegralGain >= rtb_Saturation2) {
    u0 = rtb_Saturation2;
  } else if (rtb_IntegralGain > power_limiter_P.Constant_Value_d) {
    /* Switch: '<S44>/Switch1' */
    u0 = rtb_IntegralGain;
  } else {
    u0 = power_limiter_P.Constant_Value_d;
  }

  /* Sum: '<S44>/Diff' incorporates:
   *  Switch: '<S44>/Switch'
   */
  rtb_IntegralGain -= u0;

  /* RelationalOperator: '<S41>/Relational Operator' incorporates:
   *  Constant: '<S41>/Clamping_zero'
   */
  rtb_RelationalOperator = (power_limiter_P.Clamping_zero_Value !=
    rtb_IntegralGain);

  /* RelationalOperator: '<S41>/fix for DT propagation issue' incorporates:
   *  Constant: '<S41>/Clamping_zero'
   */
  rtb_fixforDTpropagationissue = (rtb_IntegralGain >
    power_limiter_P.Clamping_zero_Value);

  /* Gain: '<S50>/Integral Gain' */
  rtb_IntegralGain = power_limiter_P.I_p * rtb_Abs2;

  /* Switch: '<S41>/Switch3' incorporates:
   *  Constant: '<S41>/Constant6'
   *  Constant: '<S41>/Constant7'
   */
  if (rtb_fixforDTpropagationissue) {
    tmp = power_limiter_P.Constant6_Value;
  } else {
    tmp = power_limiter_P.Constant7_Value;
  }

  /* Switch: '<S41>/Switch2' incorporates:
   *  Constant: '<S41>/Clamping_zero'
   *  Constant: '<S41>/Constant4'
   *  Constant: '<S41>/Constant5'
   *  RelationalOperator: '<S41>/fix for DT propagation issue1'
   */
  if (rtb_IntegralGain > power_limiter_P.Clamping_zero_Value) {
    tmp_0 = power_limiter_P.Constant4_Value;
  } else {
    tmp_0 = power_limiter_P.Constant5_Value;
  }

  /* RelationalOperator: '<S41>/Equal1' incorporates:
   *  Switch: '<S41>/Switch2'
   *  Switch: '<S41>/Switch3'
   */
  rtb_fixforDTpropagationissue = (tmp == tmp_0);

  /* RelationalOperator: '<S41>/Relational Operator1' incorporates:
   *  Constant: '<S41>/Clamping_zero'
   *  Constant: '<S57>/P Copy'
   */
  rtb_RelationalOperator1 = (power_limiter_P.P_p >
    power_limiter_P.Clamping_zero_Value);

  /* Switch: '<S41>/Switch' incorporates:
   *  Constant: '<S41>/Constant1'
   *  Logic: '<S41>/AND1'
   *  Logic: '<S41>/AND2'
   *  Logic: '<S41>/AND3'
   *  Logic: '<S41>/NOT1'
   *  Logic: '<S41>/NOT2'
   *  Logic: '<S41>/OR1'
   */
  if (rtb_RelationalOperator && ((rtb_fixforDTpropagationissue &&
        rtb_RelationalOperator1) || ((!rtb_fixforDTpropagationissue) &&
        (!rtb_RelationalOperator1)))) {
    rtb_IntegralGain = power_limiter_P.Constant1_Value;
  }

  /* End of Switch: '<S41>/Switch' */

  /* DiscreteIntegrator: '<S53>/Integrator' */
  rtb_DerivativeGain_tmp = power_limiter_P.Integrator_gainval_b *
    rtb_IntegralGain;

  /* DiscreteIntegrator: '<S53>/Integrator' */
  rtb_IntegralGain = rtb_DerivativeGain_tmp + power_limiter_DW.p_int;

  /* Gain: '<S51>/Proportional Gain' incorporates:
   *  Sum: '<S63>/Sum'
   */
  rtb_Max = ((rtb_Abs2 + rtb_IntegralGain) + rtb_Max) * power_limiter_P.P_p;

  /* Switch: '<S61>/Switch2' incorporates:
   *  RelationalOperator: '<S61>/LowerRelop1'
   */
  if (!(rtb_Max > rtb_Saturation2)) {
    /* Switch: '<S61>/Switch' incorporates:
     *  Constant: '<Root>/Constant'
     *  RelationalOperator: '<S61>/UpperRelop'
     */
    if (rtb_Max < power_limiter_P.Constant_Value_d) {
      rtb_Saturation2 = power_limiter_P.Constant_Value_d;
    } else {
      rtb_Saturation2 = rtb_Max;
    }

    /* End of Switch: '<S61>/Switch' */
  }

  /* End of Switch: '<S61>/Switch2' */

  /* Outport: '<Root>/dbg_pid' */
  power_limiter_Y.dbg_pid = rtb_Saturation2;

  /* Outport: '<Root>/dbg_ff' */
  power_limiter_Y.dbg_ff = rtb_Saturation3;

  /* Outport: '<Root>/dbg_err' */
  power_limiter_Y.dbg_err = rtb_Abs2;

  /* Product: '<S4>/delta rise limit' incorporates:
   *  Gain: '<Root>/Gain'
   *  SampleTimeMath: '<S4>/sample time'
   *
   * About '<S4>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Max = power_limiter_P.Gain_Gain * rtb_Saturation3 *
    power_limiter_P.sampletime_WtEt;

  /* UnitDelay: '<S4>/Delay Input2' */
  rtb_Abs2 = power_limiter_DW.DelayInput2_DSTATE;

  /* Sum: '<Root>/Add5' */
  u0 = rtb_Saturation2 + rtb_Saturation3;

  /* Saturate: '<Root>/Saturation2' */
  if (u0 > power_limiter_P.Saturation2_UpperSat) {
    u0 = power_limiter_P.Saturation2_UpperSat;
  } else if (u0 < power_limiter_P.Saturation2_LowerSat) {
    u0 = power_limiter_P.Saturation2_LowerSat;
  }

  /* Sum: '<S4>/Difference Inputs1' incorporates:
   *  Abs: '<Root>/Abs2'
   *  Inport: '<Root>/m_ref'
   *  MinMax: '<Root>/Min1'
   *  Product: '<Root>/Product6'
   *  Saturate: '<Root>/Saturation2'
   */
  rtb_Sign = rtb_Sign * fmin(fabs(power_limiter_U.m_ref), u0) - rtb_Abs2;

  /* Switch: '<S71>/Switch2' incorporates:
   *  RelationalOperator: '<S71>/LowerRelop1'
   */
  if (!(rtb_Sign > rtb_Max)) {
    /* Product: '<S4>/delta fall limit' incorporates:
     *  Gain: '<Root>/Gain2'
     *  SampleTimeMath: '<S4>/sample time'
     *
     * About '<S4>/sample time':
     *  y = K where K = ( w * Ts )
     */
    rtb_Saturation3 = power_limiter_P.Gain2_Gain * rtb_Saturation3 *
      power_limiter_P.sampletime_WtEt;

    /* Switch: '<S71>/Switch' incorporates:
     *  RelationalOperator: '<S71>/UpperRelop'
     */
    if (rtb_Sign < rtb_Saturation3) {
      rtb_Max = rtb_Saturation3;
    } else {
      rtb_Max = rtb_Sign;
    }

    /* End of Switch: '<S71>/Switch' */
  }

  /* End of Switch: '<S71>/Switch2' */

  /* Sum: '<S4>/Difference Inputs2' */
  rtb_Saturation3 = rtb_Max + rtb_Abs2;

  /* Outport: '<Root>/m_set' */
  power_limiter_Y.m_set = rtb_Saturation3;

  /* MinMax: '<S10>/Max' incorporates:
   *  Constant: '<S10>/Time constant'
   */
  rtb_Max = fmax(power_limiter_B.Probe[0],
                 power_limiter_P.LowPassFilterDiscreteorContin_k);

  /* Update for Delay: '<Root>/Delay' incorporates:
   *  Inport: '<Root>/p_mcu'
   */
  power_limiter_DW.icLoad = false;
  for (i = 0; i < 9; i++) {
    power_limiter_DW.Delay_DSTATE[i] = power_limiter_DW.Delay_DSTATE[i + 1];
  }

  power_limiter_DW.Delay_DSTATE[9] = power_limiter_U.p_mcu;

  /* End of Update for Delay: '<Root>/Delay' */

  /* Update for DiscreteIntegrator: '<S16>/Integrator' incorporates:
   *  Fcn: '<S10>/Avoid Divide by Zero'
   *  Product: '<S2>/1//T'
   *  Sum: '<S2>/Sum1'
   */
  power_limiter_DW.Integrator_IC_LOADING = 0U;
  power_limiter_DW.Integrator_DSTATE += 1.0 / ((real_T)(rtb_Max == 0.0) *
    2.2204460492503131e-16 + rtb_Max) * (rtb_K - rtb_Saturation) *
    power_limiter_P.Integrator_gainval;
  if (power_limiter_DW.Integrator_DSTATE >= power_limiter_P.Integrator_UpperSat)
  {
    power_limiter_DW.Integrator_DSTATE = power_limiter_P.Integrator_UpperSat;
  } else if (power_limiter_DW.Integrator_DSTATE <=
             power_limiter_P.Integrator_LowerSat) {
    power_limiter_DW.Integrator_DSTATE = power_limiter_P.Integrator_LowerSat;
  }

  power_limiter_DW.Integrator_PrevResetState = (int8_T)rtb_LogicalOperator;

  /* End of Update for DiscreteIntegrator: '<S16>/Integrator' */

  /* Update for DiscreteIntegrator: '<S9>/Integrator' incorporates:
   *  Constant: '<S1>/Constant'
   */
  power_limiter_DW.Integrator_IC_LOADING_g = 0U;
  power_limiter_DW.Integrator_DSTATE_m += power_limiter_P.Integrator_gainval_a *
    rtb_Integrator_m;
  if (power_limiter_DW.Integrator_DSTATE_m >=
      power_limiter_P.Integrator_UpperSat_f) {
    power_limiter_DW.Integrator_DSTATE_m = power_limiter_P.Integrator_UpperSat_f;
  } else if (power_limiter_DW.Integrator_DSTATE_m <=
             power_limiter_P.Integrator_LowerSat_a) {
    power_limiter_DW.Integrator_DSTATE_m = power_limiter_P.Integrator_LowerSat_a;
  }

  if (power_limiter_P.Constant_Value_h > 0.0) {
    power_limiter_DW.Integrator_PrevResetState_d = 1;
  } else if (power_limiter_P.Constant_Value_h < 0.0) {
    power_limiter_DW.Integrator_PrevResetState_d = -1;
  } else if (power_limiter_P.Constant_Value_h == 0.0) {
    power_limiter_DW.Integrator_PrevResetState_d = 0;
  } else {
    power_limiter_DW.Integrator_PrevResetState_d = 2;
  }

  /* End of Update for DiscreteIntegrator: '<S9>/Integrator' */

  /* Update for DiscreteTransferFcn: '<S46>/Filter Differentiator TF' */
  power_limiter_DW.p_filt = rtb_Abs1;

  /* Update for DiscreteIntegrator: '<S53>/Integrator' */
  power_limiter_DW.p_int = rtb_DerivativeGain_tmp + rtb_IntegralGain;

  /* Update for UnitDelay: '<S4>/Delay Input2' */
  power_limiter_DW.DelayInput2_DSTATE = rtb_Saturation3;
}

/* Model initialize function */
void power_limiter_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  power_limiter_P.FilteredDerivativeDiscreteorCon = rtMinusInf;
  power_limiter_P.FilteredDerivativeDiscreteorC_p = rtInf;
  power_limiter_P.Saturation_UpperSat = rtInf;
  power_limiter_P.Saturation_LowerSat = rtMinusInf;
  power_limiter_P.Integrator_UpperSat_f = rtInf;
  power_limiter_P.Integrator_LowerSat_a = rtMinusInf;
  power_limiter_P.Saturation_UpperSat_n = rtInf;
  power_limiter_P.Saturation_LowerSat_j = rtMinusInf;

  /* initialize error status */
  rtmSetErrorStatus(power_limiter_M, (NULL));

  /* block I/O */
  (void) memset(((void *) &power_limiter_B), 0,
                sizeof(B_power_limiter_T));

  /* states (dwork) */
  (void) memset((void *)&power_limiter_DW, 0,
                sizeof(DW_power_limiter_T));

  /* external inputs */
  (void)memset(&power_limiter_U, 0, sizeof(ExtU_power_limiter_T));

  /* external outputs */
  (void)memset(&power_limiter_Y, 0, sizeof(ExtY_power_limiter_T));

  /* Start for Probe: '<S10>/Probe' */
  power_limiter_B.Probe[0] = 0.01;
  power_limiter_B.Probe[1] = 0.0;

  /* Start for Probe: '<S5>/Probe' */
  power_limiter_B.Probe_e[0] = 0.01;
  power_limiter_B.Probe_e[1] = 0.0;

  /* InitializeConditions for Delay: '<Root>/Delay' */
  power_limiter_DW.icLoad = true;

  /* InitializeConditions for DiscreteIntegrator: '<S16>/Integrator' */
  power_limiter_DW.Integrator_PrevResetState = 0;
  power_limiter_DW.Integrator_IC_LOADING = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S9>/Integrator' */
  power_limiter_DW.Integrator_PrevResetState_d = 0;
  power_limiter_DW.Integrator_IC_LOADING_g = 1U;

  /* InitializeConditions for DiscreteTransferFcn: '<S46>/Filter Differentiator TF' */
  power_limiter_DW.p_filt = power_limiter_P.PIDController4_InitialCondition;

  /* InitializeConditions for DiscreteIntegrator: '<S53>/Integrator' */
  power_limiter_DW.p_int = power_limiter_P.PIDController4_InitialConditi_l;

  /* InitializeConditions for UnitDelay: '<S4>/Delay Input2' */
  power_limiter_DW.DelayInput2_DSTATE =
    power_limiter_P.DelayInput2_InitialCondition;
}

/* Model terminate function */
void power_limiter_terminate(void)
{
  /* (no terminate code required) */
}
