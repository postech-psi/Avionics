//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ukf.h
//
// Code generated for Simulink model 'ukf'.
//
// Model version                  : 1.32
// Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
// C/C++ source code generated on : Tue Dec  2 15:14:53 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef ukf_h_
#define ukf_h_
#include <cmath>
#include "rtwtypes.h"
#include "coder_bounded_array.h"

extern "C"
{
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"

extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
}                                      // extern "C"

// Class declaration for model ukf
class kf final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    coder::bounded_array<real_T,100,2> history_buffer;// '<Root>/Apogee_predictor' 
    real_T x_hat[2];                   // '<Root>/Kalman_Filter'
    real_T P_a[4];                     // '<Root>/Kalman_Filter'
    real_T theta[3];                   // '<Root>/Apogee_predictor'
    real_T P_g[9];                     // '<Root>/Apogee_predictor'
    real_T c_x_data[50];
    real_T sigma_points[10];
    real_T propagated_points[10];
    real_T K[9];
    real_T dv[9];
    boolean_T b_x_data[50];
    boolean_T is_initialized_not_empty;// '<Root>/Kalman_Filter'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    real_T accel_z_in;                 // '<Root>/accel_z_in'
    real_T baro_altitude_in;           // '<Root>/baro_altitude_in'
    real_T current_time_in;            // '<Root>/current_time_in'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real_T kf_altitude_out;            // '<Root>/kf_altitude_out'
    real_T kf_velocity_out;            // '<Root>/kf_velocity_out'
    real_T apogee_out;                 // '<Root>/apogee_out'
    real_T r2_out;                     // '<Root>/r2_out'
  };

  // Real-time Model Data Structure
  struct RT_MODEL {
    const char_T * volatile errorStatus;
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const volatile aErrorStatus);
  };

  // Copy Constructor
  kf(kf const&) = delete;

  // Assignment Operator
  kf& operator= (kf const&) & = delete;

  // Move Constructor
  kf(kf &&) = delete;

  // Move Assignment Operator
  kf& operator= (kf &&) = delete;

  // Real-Time Model get method
  kf::RT_MODEL * getRTM();

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  kf();

  // Destructor
  ~kf();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // private member function(s) for subsystem '<Root>'
  void binary_expand_op_1(real_T in1[10], int32_T in2, const real_T in3[2],
    const real_T in4[4], int32_T in5);
  void binary_expand_op(real_T in1[10], int32_T in2, const real_T in3[2], const
                        real_T in4[4], int32_T in5);

  // Real-Time Model
  RT_MODEL rtM;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ukf'
//  '<S1>'   : 'ukf/Apogee_predictor'
//  '<S2>'   : 'ukf/Kalman_Filter'

#endif                                 // ukf_h_

//
// File trailer for generated code.
//
// [EOF]
//
