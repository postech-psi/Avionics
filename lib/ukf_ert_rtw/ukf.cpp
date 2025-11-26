//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ukf.cpp
//
// Code generated for Simulink model 'ukf'.
//
// Model version                  : 1.31
// Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
// C/C++ source code generated on : Sun Nov  9 20:56:38 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "ukf.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include "cmath"
#include "limits"

extern "C"
{
  real_T rtNaN { -std::numeric_limits<real_T>::quiet_NaN() };

  real_T rtInf { std::numeric_limits<real_T>::infinity() };

  real_T rtMinusInf { -std::numeric_limits<real_T>::infinity() };

  real32_T rtNaNF { -std::numeric_limits<real32_T>::quiet_NaN() };

  real32_T rtInfF { std::numeric_limits<real32_T>::infinity() };

  real32_T rtMinusInfF { -std::numeric_limits<real32_T>::infinity() };
}

extern "C"
{
  // Return rtNaN needed by the generated code.
  static real_T rtGetNaN(void)
  {
    return rtNaN;
  }

  // Return rtNaNF needed by the generated code.
  static real32_T rtGetNaNF(void)
  {
    return rtNaNF;
  }
}

extern "C"
{
  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return std::isinf(value);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return std::isinf(value);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    return std::isnan(value);
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    return std::isnan(value);
  }
}

void kf::binary_expand_op_1(real_T in1[10], int32_T in2, const real_T in3[2],
  const real_T in4[4], int32_T in5)
{
  int32_T tmp;
  int32_T tmp_0;

  // MATLAB Function: '<Root>/Kalman_Filter'
  tmp = in2 << 1;
  tmp_0 = (in2 + 1) << 1;
  in1[tmp_0] = in4[tmp] + in3[0];
  in1[tmp_0 + 1] = in4[(in5 + 1 != 1) + tmp] + in3[1];
}

void kf::binary_expand_op(real_T in1[10], int32_T in2, const real_T in3[2],
  const real_T in4[4], int32_T in5)
{
  int32_T tmp;
  int32_T tmp_0;

  // MATLAB Function: '<Root>/Kalman_Filter'
  tmp = in2 << 1;
  tmp_0 = (in2 + 3) << 1;
  in1[tmp_0] = in3[0] - in4[tmp];
  in1[tmp_0 + 1] = in3[1] - in4[(in5 + 1 != 1) + tmp];
}

// Model step function
void kf::step()
{
  real_T Wc[5];
  real_T Wm[5];
  real_T P_minus[4];
  real_T K[3];
  real_T x[3];
  real_T P_minus_0;
  real_T P_minus_1;
  real_T Wc_0;
  real_T accumulatedData;
  real_T diff_idx_0;
  real_T diff_idx_1;
  real_T propagated_points;
  real_T ssq;
  real_T x_hat;
  int32_T b_j;
  int32_T idxAjj;
  int32_T ix;
  int32_T nz;
  boolean_T exitg1;

  // MATLAB Function: '<Root>/Kalman_Filter' incorporates:
  //   Inport: '<Root>/accel_z_in'
  //   Inport: '<Root>/baro_altitude_in'

  if (!rtDW.is_initialized_not_empty) {
    rtDW.x_hat[0] = rtU.baro_altitude_in;
    rtDW.x_hat[1] = 0.0;
    rtDW.P_a[0] = 100.0;
    rtDW.P_a[1] = 0.0;
    rtDW.P_a[2] = 0.0;
    rtDW.P_a[3] = 100.0;
    rtDW.is_initialized_not_empty = true;
  }

  Wm[0] = -999998.99997124437;
  Wc[0] = -999995.99997224438;
  Wm[1] = 249999.99999281109;
  Wc[1] = 249999.99999281109;
  Wm[2] = 249999.99999281109;
  Wc[2] = 249999.99999281109;
  Wm[3] = 249999.99999281109;
  Wc[3] = 249999.99999281109;
  Wm[4] = 249999.99999281109;
  Wc[4] = 249999.99999281109;
  P_minus[0] = ((rtDW.P_a[0] + rtDW.P_a[0]) * 0.5 + 1.0E-9) *
    2.0000000000575113E-6;
  diff_idx_0 = (rtDW.P_a[1] + rtDW.P_a[2]) * 0.5 * 2.0000000000575113E-6;
  P_minus[1] = diff_idx_0;
  P_minus[2] = diff_idx_0;
  P_minus[3] = ((rtDW.P_a[3] + rtDW.P_a[3]) * 0.5 + 1.0E-9) *
    2.0000000000575113E-6;
  nz = 0;
  b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (b_j < 2)) {
    idxAjj = (b_j << 1) + b_j;
    ssq = 0.0;
    if (b_j >= 1) {
      ssq = P_minus[1] * P_minus[1];
    }

    ssq = P_minus[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      P_minus[idxAjj] = ssq;
      if (b_j + 1 < 2) {
        ssq = 1.0 / ssq;
        for (ix = idxAjj + 2; ix <= idxAjj + 2; ix++) {
          P_minus[ix - 1] *= ssq;
        }
      }

      b_j++;
    } else {
      P_minus[idxAjj] = ssq;
      nz = b_j + 1;
      exitg1 = true;
    }
  }

  if (nz == 0) {
    b_j = 2;
    P_minus[2] = 0.0;
  } else {
    b_j = nz - 1;
  }

  if (b_j < 1) {
    idxAjj = -1;
  } else {
    idxAjj = b_j - 1;
  }

  if (nz > 0) {
    diff_idx_0 = rtDW.x_hat[0];

    // Outport: '<Root>/kf_velocity_out'
    rtY.kf_velocity_out = rtDW.x_hat[1];
  } else {
    std::memset(&rtDW.sigma_points[0], 0, 10U * sizeof(real_T));
    rtDW.sigma_points[0] = rtDW.x_hat[0];
    rtDW.sigma_points[1] = rtDW.x_hat[1];
    if (idxAjj + 1 == 2) {
      rtDW.sigma_points[2] = rtDW.x_hat[0] + P_minus[0];
      rtDW.sigma_points[3] = rtDW.x_hat[1] + P_minus[1];
      rtDW.sigma_points[6] = rtDW.x_hat[0] - P_minus[0];
      rtDW.sigma_points[7] = rtDW.x_hat[1] - P_minus[1];
      rtDW.sigma_points[4] = rtDW.x_hat[0] + P_minus[2];
      rtDW.sigma_points[5] = rtDW.x_hat[1] + P_minus[3];
      rtDW.sigma_points[8] = rtDW.x_hat[0] - P_minus[2];
      rtDW.sigma_points[9] = rtDW.x_hat[1] - P_minus[3];
    } else {
      binary_expand_op_1(rtDW.sigma_points, 0, rtDW.x_hat, P_minus, idxAjj);
      binary_expand_op(rtDW.sigma_points, 0, rtDW.x_hat, P_minus, idxAjj);
      binary_expand_op_1(rtDW.sigma_points, 1, rtDW.x_hat, P_minus, idxAjj);
      binary_expand_op(rtDW.sigma_points, 1, rtDW.x_hat, P_minus, idxAjj);
    }

    x_hat = 0.0;
    accumulatedData = 0.0;
    for (nz = 0; nz < 5; nz++) {
      idxAjj = nz << 1;
      diff_idx_0 = rtDW.sigma_points[idxAjj + 1];
      ssq = (rtU.accel_z_in - 9.81) - 0.6125 * diff_idx_0 * std::abs(diff_idx_0)
        * 0.62 * 0.00725 / 4.25;
      propagated_points = (diff_idx_0 * 0.02 + rtDW.sigma_points[idxAjj]) + 0.5 *
        ssq * 0.0004;
      rtDW.propagated_points[idxAjj] = propagated_points;
      ssq = ssq * 0.02 + diff_idx_0;
      rtDW.propagated_points[idxAjj + 1] = ssq;
      diff_idx_0 = Wm[nz];
      x_hat += propagated_points * diff_idx_0;
      accumulatedData += ssq * diff_idx_0;
    }

    rtDW.x_hat[1] = accumulatedData;
    rtDW.x_hat[0] = x_hat;
    x_hat = rtDW.x_hat[0];
    accumulatedData = rtDW.x_hat[1];
    P_minus_0 = 2.9881050105278437E-5;
    ssq = 0.0029881050105278442;
    propagated_points = 0.0029881050105278442;
    P_minus_1 = 0.29881050105278439;
    for (nz = 0; nz < 5; nz++) {
      idxAjj = nz << 1;
      diff_idx_0 = rtDW.propagated_points[idxAjj] - x_hat;
      diff_idx_1 = rtDW.propagated_points[idxAjj + 1] - accumulatedData;
      Wc_0 = Wc[nz];
      P_minus_0 += diff_idx_0 * diff_idx_0 * Wc_0;
      ssq += diff_idx_1 * diff_idx_0 * Wc_0;
      propagated_points = ssq;
      P_minus_1 += diff_idx_1 * diff_idx_1 * Wc_0;
    }

    diff_idx_0 = 0.0 * ssq + P_minus_0;
    accumulatedData = 0.0 * P_minus_1 + propagated_points;
    x_hat = accumulatedData * 0.0 + diff_idx_0;
    diff_idx_0 /= x_hat + 0.0217839917878801;
    diff_idx_1 = accumulatedData / (x_hat + 0.0217839917878801);
    accumulatedData = rtU.baro_altitude_in - (0.0 * rtDW.x_hat[1] + rtDW.x_hat[0]);
    rtDW.x_hat[0] += diff_idx_0 * accumulatedData;
    Wc_0 = (x_hat + 0.0217839917878801) * diff_idx_0;
    rtDW.P_a[0] = P_minus_0 - Wc_0 * diff_idx_0;
    x_hat = (x_hat + 0.0217839917878801) * diff_idx_1;
    rtDW.P_a[1] = ssq - x_hat * diff_idx_0;
    rtDW.x_hat[1] += diff_idx_1 * accumulatedData;
    rtDW.P_a[2] = propagated_points - Wc_0 * diff_idx_1;
    rtDW.P_a[3] = P_minus_1 - x_hat * diff_idx_1;
    diff_idx_0 = rtDW.x_hat[0];

    // Outport: '<Root>/kf_velocity_out' incorporates:
    //   Inport: '<Root>/accel_z_in'
    //   Inport: '<Root>/baro_altitude_in'

    rtY.kf_velocity_out = rtDW.x_hat[1];
  }

  // End of MATLAB Function: '<Root>/Kalman_Filter'

  // Outport: '<Root>/kf_altitude_out'
  rtY.kf_altitude_out = diff_idx_0;

  // MATLAB Function: '<Root>/Apogee_predictor' incorporates:
  //   Inport: '<Root>/current_time_in'

  x[0] = rtU.current_time_in * rtU.current_time_in;
  x[1] = rtU.current_time_in;
  x[2] = 1.0;
  propagated_points = 0.0;
  for (nz = 0; nz < 3; nz++) {
    propagated_points += ((rtDW.P_g[3 * nz + 1] * rtU.current_time_in +
      rtDW.P_g[3 * nz] * x[0]) + rtDW.P_g[3 * nz + 2]) * x[nz];
  }

  ssq = 0.0;
  for (nz = 0; nz < 3; nz++) {
    K[nz] = ((rtDW.P_g[nz + 3] * rtU.current_time_in + rtDW.P_g[nz] * x[0]) +
             rtDW.P_g[nz + 6]) / (propagated_points + 0.965);
    ssq += x[nz] * rtDW.theta[nz];
  }

  ssq = diff_idx_0 - ssq;
  for (nz = 0; nz < 3; nz++) {
    rtDW.theta[nz] += K[nz] * ssq;
    propagated_points = x[nz];
    rtDW.K[3 * nz] = K[0] * propagated_points;
    rtDW.K[3 * nz + 1] = K[1] * propagated_points;
    rtDW.K[3 * nz + 2] = K[2] * propagated_points;
  }

  for (nz = 0; nz < 3; nz++) {
    ssq = rtDW.K[nz + 3];
    propagated_points = rtDW.K[nz];
    P_minus_1 = rtDW.K[nz + 6];
    for (idxAjj = 0; idxAjj < 3; idxAjj++) {
      b_j = 3 * idxAjj + nz;
      rtDW.dv[b_j] = (rtDW.P_g[b_j] - ((rtDW.P_g[3 * idxAjj + 1] * ssq +
        rtDW.P_g[3 * idxAjj] * propagated_points) + rtDW.P_g[3 * idxAjj + 2] *
        P_minus_1)) / 0.965;
    }
  }

  std::memcpy(&rtDW.P_g[0], &rtDW.dv[0], 9U * sizeof(real_T));
  if (rtDW.theta[0] < -1.0E-6) {
    // Outport: '<Root>/apogee_out'
    rtY.apogee_out = rtDW.theta[2] - rtDW.theta[1] * rtDW.theta[1] / (4.0 *
      rtDW.theta[0]);
  } else {
    // Outport: '<Root>/apogee_out'
    rtY.apogee_out = (rtNaN);
  }

  for (nz = 0; nz < 2; nz++) {
    for (idxAjj = 0; idxAjj < 49; idxAjj++) {
      rtDW.history_buffer.data[idxAjj + 50 * nz] = rtDW.history_buffer.data[(50 *
        nz + idxAjj) + 1];
    }
  }

  rtDW.history_buffer.data[49] = rtU.current_time_in;
  rtDW.history_buffer.data[99] = diff_idx_0;
  for (nz = 0; nz < 50; nz++) {
    rtDW.b_x_data[nz] = std::isnan(rtDW.history_buffer.data[nz]);
  }

  nz = rtDW.b_x_data[0];
  for (ix = 0; ix < 49; ix++) {
    nz += rtDW.b_x_data[ix + 1];
  }

  if (nz == 0) {
    diff_idx_0 = rtDW.theta[0];
    accumulatedData = rtDW.theta[1];
    x_hat = rtDW.theta[2];
    for (nz = 0; nz < 50; nz++) {
      ssq = rtDW.history_buffer.data[nz];
      propagated_points = rtDW.history_buffer.data[nz + 50] - ((ssq * ssq *
        diff_idx_0 + ssq * accumulatedData) + x_hat);
      rtDW.c_x_data[nz] = propagated_points * propagated_points;
    }

    ssq = rtDW.c_x_data[0];
    accumulatedData = rtDW.history_buffer.data[50];
    for (nz = 0; nz < 49; nz++) {
      ssq += rtDW.c_x_data[nz + 1];
      accumulatedData += rtDW.history_buffer.data[nz + 51];
    }

    accumulatedData /= 50.0;
    for (nz = 0; nz < 50; nz++) {
      propagated_points = rtDW.history_buffer.data[nz + 50] - accumulatedData;
      rtDW.c_x_data[nz] = propagated_points * propagated_points;
    }

    accumulatedData = rtDW.c_x_data[0];
    for (nz = 0; nz < 49; nz++) {
      accumulatedData += rtDW.c_x_data[nz + 1];
    }

    if (accumulatedData > 0.0) {
      // Outport: '<Root>/r2_out'
      rtY.r2_out = 1.0 - ssq / accumulatedData;
    } else {
      // Outport: '<Root>/r2_out'
      rtY.r2_out = 1.0;
    }
  } else {
    // Outport: '<Root>/r2_out'
    rtY.r2_out = (rtNaN);
  }

  // End of MATLAB Function: '<Root>/Apogee_predictor'
}

// Model initialize function
void kf::initialize()
{
  {
    int32_T i;
    static const int16_T tmp[9]{ 1000, 0, 0, 0, 1000, 0, 0, 0, 1000 };

    // SystemInitialize for MATLAB Function: '<Root>/Apogee_predictor'
    for (i = 0; i < 9; i++) {
      rtDW.P_g[i] = tmp[i];
    }

    rtDW.history_buffer.size[0] = 50;
    rtDW.history_buffer.size[1] = 2;
    for (i = 0; i < 100; i++) {
      rtDW.history_buffer.data[i] = (rtNaN);
    }

    // End of SystemInitialize for MATLAB Function: '<Root>/Apogee_predictor'
  }
}

const char_T* kf::RT_MODEL::getErrorStatus() const
{
  return (errorStatus);
}

void kf::RT_MODEL::setErrorStatus(const char_T* const volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

// Constructor
kf::kf() :
  rtU(),
  rtY(),
  rtDW(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
kf::~kf() = default;

// Real-Time Model get method
kf::RT_MODEL * kf::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
