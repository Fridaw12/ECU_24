/*
 * power_limiter.h
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

#ifndef RTW_HEADER_power_limiter_h_
#define RTW_HEADER_power_limiter_h_
#ifndef power_limiter_COMMON_INCLUDES_
#define power_limiter_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* power_limiter_COMMON_INCLUDES_ */

#include "power_limiter_types.h"
#include "rt_nonfinite.h"
//#include "rtGetNaN.h"
#include <stddef.h>
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Probe[2];                     /* '<S10>/Probe' */
  real_T Probe_e[2];                   /* '<S5>/Probe' */
  real_T TmpSignalConversionAtFilterDiff[2];
} B_power_limiter_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE[10];             /* '<Root>/Delay' */
  real_T Integrator_DSTATE;            /* '<S16>/Integrator' */
  real_T Integrator_DSTATE_m;          /* '<S9>/Integrator' */
  real_T p_filt;                       /* '<S46>/Filter Differentiator TF' */
  real_T p_int;                        /* '<S53>/Integrator' */
  real_T DelayInput2_DSTATE;           /* '<S4>/Delay Input2' */
  int8_T Integrator_PrevResetState;    /* '<S16>/Integrator' */
  int8_T Integrator_PrevResetState_d;  /* '<S9>/Integrator' */
  uint8_T Integrator_IC_LOADING;       /* '<S16>/Integrator' */
  uint8_T Integrator_IC_LOADING_g;     /* '<S9>/Integrator' */
  boolean_T icLoad;                    /* '<Root>/Delay' */
} DW_power_limiter_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T m_ref;                        /* '<Root>/m_ref' */
  real_T p_max;                        /* '<Root>/p_max' */
  real_T u_dc_nom;                     /* '<Root>/u_dc_nom' */
  real_T p_limp;                       /* '<Root>/p_limp' */
  real_T u_dc_ams;                     /* '<Root>/u_dc_ams' */
  real_T u_q_mcu;                      /* '<Root>/u_q_mcu' */
  real_T i_q_max;                      /* '<Root>/i_q_max' */
  real_T i_dc_ams;                     /* '<Root>/i_dc_ams' */
  real_T i_lim_fuse;                   /* '<Root>/i_lim_fuse' */
  real_T i_limp_ams;                   /* '<Root>/i_limp_ams' */
  real_T u_dc_mcu;                     /* '<Root>/u_dc_mcu' */
  real_T p_mcu;                        /* '<Root>/p_mcu' */
  real_T p_limn;                       /* '<Root>/p_limn' */
  real_T i_limn_ams;                   /* '<Root>/i_limn_ams' */
} ExtU_power_limiter_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T dbg_pid;                      /* '<Root>/dbg_pid' */
  real_T dbg_ff;                       /* '<Root>/dbg_ff' */
  real_T dbg_err;                      /* '<Root>/dbg_err' */
  real_T m_set;                        /* '<Root>/m_set' */
} ExtY_power_limiter_T;

/* Parameters (default storage) */
struct P_power_limiter_T_ {
  real_T D_p;                          /* Variable: D_p
                                        * Referenced by: '<S45>/Derivative Gain'
                                        */
  real_T I_p;                          /* Variable: I_p
                                        * Referenced by: '<S50>/Integral Gain'
                                        */
  real_T N_p;                          /* Variable: N_p
                                        * Referenced by:
                                        *   '<S55>/N Copy'
                                        *   '<S56>/Filter Coefficient'
                                        */
  real_T P_p;                          /* Variable: P_p
                                        * Referenced by:
                                        *   '<S51>/Proportional Gain'
                                        *   '<S52>/Proportional Gain'
                                        *   '<S57>/P Copy'
                                        */
  real_T Ts_p;                         /* Variable: Ts_p
                                        * Referenced by: '<S1>/Gain'
                                        */
  real_T FilteredDerivativeDiscreteorCon;
                              /* Mask Parameter: FilteredDerivativeDiscreteorCon
                               * Referenced by: '<S1>/[A,B]'
                               */
  real_T FilteredDerivativeDiscreteorC_p;
                              /* Mask Parameter: FilteredDerivativeDiscreteorC_p
                               * Referenced by: '<S1>/[A,B]'
                               */
  real_T PIDController4_InitialCondition;
                              /* Mask Parameter: PIDController4_InitialCondition
                               * Referenced by: '<S46>/Filter Differentiator TF'
                               */
  real_T PIDController4_InitialConditi_l;
                              /* Mask Parameter: PIDController4_InitialConditi_l
                               * Referenced by: '<S53>/Integrator'
                               */
  real_T LowPassFilterDiscreteorContinuo;
                              /* Mask Parameter: LowPassFilterDiscreteorContinuo
                               * Referenced by: '<S2>/K'
                               */
  real_T LowPassFilterDiscreteorContin_k;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_k
                               * Referenced by: '<S10>/Time constant'
                               */
  real_T FilteredDerivativeDiscreteorC_c;
                              /* Mask Parameter: FilteredDerivativeDiscreteorC_c
                               * Referenced by: '<S5>/Time constant'
                               */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S13>/Constant'
                                       */
  real_T LowPassFilterDiscreteorContin_b;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_b
                               * Referenced by: '<S10>/Constant'
                               */
  real_T FilteredDerivativeDiscreteorC_i;
                              /* Mask Parameter: FilteredDerivativeDiscreteorC_i
                               * Referenced by: '<S5>/Minimum sampling to time constant ratio'
                               */
  real_T Gain2_Gain;                   /* Expression: -25
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S14>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S41>/Constant1'
                                        */
  real_T FilterDenConstant_Value;      /* Expression: 1
                                        * Referenced by: '<S46>/Filter Den Constant'
                                        */
  real_T Constant1_Value_l;            /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0
                                        * Referenced by: '<Root>/Switch1'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 1e3
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: 10
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Gain12_Gain;                  /* Expression: 2/3
                                        * Referenced by: '<Root>/Gain12'
                                        */
  real_T Saturation3_UpperSat;         /* Expression: 1
                                        * Referenced by: '<Root>/Saturation3'
                                        */
  real_T Saturation3_LowerSat;         /* Expression: 0
                                        * Referenced by: '<Root>/Saturation3'
                                        */
  real_T Gain1_Gain;                   /* Expression: 0.25
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S16>/Integrator'
                                        */
  real_T Integrator_UpperSat;          /* Expression: antiwindupUpperLimit
                                        * Referenced by: '<S16>/Integrator'
                                        */
  real_T Integrator_LowerSat;          /* Expression: antiwindupLowerLimit
                                        * Referenced by: '<S16>/Integrator'
                                        */
  real_T Saturation_UpperSat;          /* Expression: windupUpperLimit
                                        * Referenced by: '<S16>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: windupLowerLimit
                                        * Referenced by: '<S16>/Saturation'
                                        */
  real_T Constant_Value_h;             /* Expression: 0
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Integrator_gainval_a;       /* Computed Parameter: Integrator_gainval_a
                                      * Referenced by: '<S9>/Integrator'
                                      */
  real_T Integrator_UpperSat_f;        /* Expression: antiwindupUpperLimit
                                        * Referenced by: '<S9>/Integrator'
                                        */
  real_T Integrator_LowerSat_a;        /* Expression: antiwindupLowerLimit
                                        * Referenced by: '<S9>/Integrator'
                                        */
  real_T Saturation_UpperSat_n;        /* Expression: windupUpperLimit
                                        * Referenced by: '<S9>/Saturation'
                                        */
  real_T Saturation_LowerSat_j;        /* Expression: windupLowerLimit
                                        * Referenced by: '<S9>/Saturation'
                                        */
  real_T Clamping_zero_Value;          /* Expression: 0
                                        * Referenced by: '<S41>/Clamping_zero'
                                        */
  real_T Tsamp_WtEt;                   /* Computed Parameter: Tsamp_WtEt
                                        * Referenced by: '<S48>/Tsamp'
                                        */
  real_T FilterDifferentiatorTF_NumCoef[2];/* Expression: [1 -1]
                                            * Referenced by: '<S46>/Filter Differentiator TF'
                                            */
  real_T Constant_Value_d;             /* Expression: -1
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Integrator_gainval_b;       /* Computed Parameter: Integrator_gainval_b
                                      * Referenced by: '<S53>/Integrator'
                                      */
  real_T Gain_Gain;                    /* Expression: 25
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T sampletime_WtEt;              /* Computed Parameter: sampletime_WtEt
                                        * Referenced by: '<S4>/sample time'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: 1
                                        * Referenced by: '<Root>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: 0
                                        * Referenced by: '<Root>/Saturation2'
                                        */
  real_T DelayInput2_InitialCondition; /* Expression: 0
                                        * Referenced by: '<S4>/Delay Input2'
                                        */
  int8_T Constant4_Value;              /* Computed Parameter: Constant4_Value
                                        * Referenced by: '<S41>/Constant4'
                                        */
  int8_T Constant5_Value;              /* Computed Parameter: Constant5_Value
                                        * Referenced by: '<S41>/Constant5'
                                        */
  int8_T Constant6_Value;              /* Computed Parameter: Constant6_Value
                                        * Referenced by: '<S41>/Constant6'
                                        */
  int8_T Constant7_Value;              /* Computed Parameter: Constant7_Value
                                        * Referenced by: '<S41>/Constant7'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_power_limiter_T {
  const char_T *errorStatus;
};

/* Block parameters (default storage) */
extern P_power_limiter_T power_limiter_P;

/* Block signals (default storage) */
extern B_power_limiter_T power_limiter_B;

/* Block states (default storage) */
extern DW_power_limiter_T power_limiter_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_power_limiter_T power_limiter_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_power_limiter_T power_limiter_Y;

/* Model entry point functions */
extern void power_limiter_initialize(void);
extern void power_limiter_step(void);
extern void power_limiter_terminate(void);

/* Real-time Model object */
extern RT_MODEL_power_limiter_T *const power_limiter_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'power_limiter'
 * '<S1>'   : 'power_limiter/Filtered Derivative (Discrete or Continuous)'
 * '<S2>'   : 'power_limiter/Low-Pass Filter (Discrete or Continuous)1'
 * '<S3>'   : 'power_limiter/PID Controller4'
 * '<S4>'   : 'power_limiter/Rate Limiter Dynamic'
 * '<S5>'   : 'power_limiter/Filtered Derivative (Discrete or Continuous)/Enable//disable time constant'
 * '<S6>'   : 'power_limiter/Filtered Derivative (Discrete or Continuous)/Initialization'
 * '<S7>'   : 'power_limiter/Filtered Derivative (Discrete or Continuous)/Integrator (Discrete or Continuous)'
 * '<S8>'   : 'power_limiter/Filtered Derivative (Discrete or Continuous)/Initialization/Init_u'
 * '<S9>'   : 'power_limiter/Filtered Derivative (Discrete or Continuous)/Integrator (Discrete or Continuous)/Discrete'
 * '<S10>'  : 'power_limiter/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant'
 * '<S11>'  : 'power_limiter/Low-Pass Filter (Discrete or Continuous)1/Initialization'
 * '<S12>'  : 'power_limiter/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)'
 * '<S13>'  : 'power_limiter/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Constant'
 * '<S14>'  : 'power_limiter/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Zero'
 * '<S15>'  : 'power_limiter/Low-Pass Filter (Discrete or Continuous)1/Initialization/Init_u'
 * '<S16>'  : 'power_limiter/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)/Discrete'
 * '<S17>'  : 'power_limiter/PID Controller4/Anti-windup'
 * '<S18>'  : 'power_limiter/PID Controller4/D Gain'
 * '<S19>'  : 'power_limiter/PID Controller4/Filter'
 * '<S20>'  : 'power_limiter/PID Controller4/Filter ICs'
 * '<S21>'  : 'power_limiter/PID Controller4/I Gain'
 * '<S22>'  : 'power_limiter/PID Controller4/Ideal P Gain'
 * '<S23>'  : 'power_limiter/PID Controller4/Ideal P Gain Fdbk'
 * '<S24>'  : 'power_limiter/PID Controller4/Integrator'
 * '<S25>'  : 'power_limiter/PID Controller4/Integrator ICs'
 * '<S26>'  : 'power_limiter/PID Controller4/N Copy'
 * '<S27>'  : 'power_limiter/PID Controller4/N Gain'
 * '<S28>'  : 'power_limiter/PID Controller4/P Copy'
 * '<S29>'  : 'power_limiter/PID Controller4/Parallel P Gain'
 * '<S30>'  : 'power_limiter/PID Controller4/Reset Signal'
 * '<S31>'  : 'power_limiter/PID Controller4/Saturation'
 * '<S32>'  : 'power_limiter/PID Controller4/Saturation Fdbk'
 * '<S33>'  : 'power_limiter/PID Controller4/Sum'
 * '<S34>'  : 'power_limiter/PID Controller4/Sum Fdbk'
 * '<S35>'  : 'power_limiter/PID Controller4/Tracking Mode'
 * '<S36>'  : 'power_limiter/PID Controller4/Tracking Mode Sum'
 * '<S37>'  : 'power_limiter/PID Controller4/Tsamp - Integral'
 * '<S38>'  : 'power_limiter/PID Controller4/Tsamp - Ngain'
 * '<S39>'  : 'power_limiter/PID Controller4/postSat Signal'
 * '<S40>'  : 'power_limiter/PID Controller4/preSat Signal'
 * '<S41>'  : 'power_limiter/PID Controller4/Anti-windup/Disc. Clamping Ideal'
 * '<S42>'  : 'power_limiter/PID Controller4/Anti-windup/Disc. Clamping Ideal/Dead Zone'
 * '<S43>'  : 'power_limiter/PID Controller4/Anti-windup/Disc. Clamping Ideal/Dead Zone/External'
 * '<S44>'  : 'power_limiter/PID Controller4/Anti-windup/Disc. Clamping Ideal/Dead Zone/External/Dead Zone Dynamic'
 * '<S45>'  : 'power_limiter/PID Controller4/D Gain/Internal Parameters'
 * '<S46>'  : 'power_limiter/PID Controller4/Filter/Disc. Trapezoidal Filter'
 * '<S47>'  : 'power_limiter/PID Controller4/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S48>'  : 'power_limiter/PID Controller4/Filter/Disc. Trapezoidal Filter/Tsamp/Internal Ts'
 * '<S49>'  : 'power_limiter/PID Controller4/Filter ICs/Internal IC - Filter'
 * '<S50>'  : 'power_limiter/PID Controller4/I Gain/Internal Parameters'
 * '<S51>'  : 'power_limiter/PID Controller4/Ideal P Gain/Internal Parameters'
 * '<S52>'  : 'power_limiter/PID Controller4/Ideal P Gain Fdbk/Internal Parameters'
 * '<S53>'  : 'power_limiter/PID Controller4/Integrator/Discrete'
 * '<S54>'  : 'power_limiter/PID Controller4/Integrator ICs/Internal IC'
 * '<S55>'  : 'power_limiter/PID Controller4/N Copy/Internal Parameters'
 * '<S56>'  : 'power_limiter/PID Controller4/N Gain/Internal Parameters'
 * '<S57>'  : 'power_limiter/PID Controller4/P Copy/Internal Parameters Ideal'
 * '<S58>'  : 'power_limiter/PID Controller4/Parallel P Gain/Passthrough'
 * '<S59>'  : 'power_limiter/PID Controller4/Reset Signal/Disabled'
 * '<S60>'  : 'power_limiter/PID Controller4/Saturation/External'
 * '<S61>'  : 'power_limiter/PID Controller4/Saturation/External/Saturation Dynamic'
 * '<S62>'  : 'power_limiter/PID Controller4/Saturation Fdbk/Passthrough'
 * '<S63>'  : 'power_limiter/PID Controller4/Sum/Sum_PID'
 * '<S64>'  : 'power_limiter/PID Controller4/Sum Fdbk/Enabled'
 * '<S65>'  : 'power_limiter/PID Controller4/Tracking Mode/Disabled'
 * '<S66>'  : 'power_limiter/PID Controller4/Tracking Mode Sum/Passthrough'
 * '<S67>'  : 'power_limiter/PID Controller4/Tsamp - Integral/TsSignalSpecification'
 * '<S68>'  : 'power_limiter/PID Controller4/Tsamp - Ngain/Passthrough'
 * '<S69>'  : 'power_limiter/PID Controller4/postSat Signal/Feedback_Path'
 * '<S70>'  : 'power_limiter/PID Controller4/preSat Signal/Feedback_Path'
 * '<S71>'  : 'power_limiter/Rate Limiter Dynamic/Saturation Dynamic'
 */
#endif                                 /* RTW_HEADER_power_limiter_h_ */
