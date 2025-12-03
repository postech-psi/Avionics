// ============================================================================
// Academic License
// ============================================================================

#include "ukf.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include "limits"

int BUFFER_SIZE = 50;
// ----------------------------------------------------------------------------
// RT NaN/Inf
// ----------------------------------------------------------------------------
extern "C"
{
    real_T   rtNaN      { -std::numeric_limits<real_T>::quiet_NaN() };
    real_T   rtInf      {  std::numeric_limits<real_T>::infinity() };
    real_T   rtMinusInf { -std::numeric_limits<real_T>::infinity() };

    real32_T rtNaNF     { -std::numeric_limits<real32_T>::quiet_NaN() };
    real32_T rtInfF     {  std::numeric_limits<real32_T>::infinity() };
    real32_T rtMinusInfF{ -std::numeric_limits<real32_T>::infinity() };
}

// ============================================================================
// Helper: Cholesky decomposition of 3x3 SPD matrix
// A = Rᵀ R.  R = upper-triangular.
// Return true on success.
// ============================================================================
static bool chol3x3(const double A[9], double R[9])
{
    // Zero R
    for (int i=0;i<9;i++) R[i] = 0.0;

    // We assume A is symmetric: A[1]=A[3], A[2]=A[6], A[5]=A[7]

    // R[0,0]
    double v = A[0];
    if (v <= 0.0) return false;
    R[0] = std::sqrt(v);

    // R[0,1], R[0,2]
    R[1] = A[1] / R[0];
    R[2] = A[2] / R[0];

    // R[1,1]
    v = A[4] - R[1]*R[1];
    if (v <= 0.0) return false;
    R[4] = std::sqrt(v);

    // R[1,2]
    R[5] = (A[5] - R[1]*R[2]) / R[4];

    // R[2,2]
    v = A[8] - (R[2]*R[2] + R[5]*R[5]);
    if (v <= 0.0) return false;
    R[8] = std::sqrt(v);

    return true;
}

// ============================================================================
// Solve normal equation using Cholesky:
// (XᵀX) θ = Xᵀy
// A = XᵀX, b = Xᵀy
// Solve A θ = b
// ============================================================================
static bool solveNormalEqCholesky(const double A[9], const double b[3], double x[3])
{
    double R[9];
    if (!chol3x3(A, R)) {
        x[0]=x[1]=x[2]=rtNaN;
        return false;
    }

    // Solve Rᵀ ỹ = b  → ỹ
    double ytilde[3];
    // Rᵀ is lower-triangular:
    // [R00  0     0   ]
    // [R01  R11   0   ]
    // [R02  R12  R22  ]

    // ỹ0
    ytilde[0] = b[0] / R[0];

    // ỹ1
    ytilde[1] = (b[1] - R[1]*ytilde[0]) / R[4];

    // ỹ2
    ytilde[2] = (b[2] - R[2]*ytilde[0] - R[5]*ytilde[1]) / R[8];

    // Now solve R x = ỹ  (R is upper triangular)
    // R:
    // [R00 R01 R02]
    // [0   R11 R12]
    // [0   0   R22]

    x[2] = ytilde[2] / R[8];
    x[1] = (ytilde[1] - R[5]*x[2]) / R[4];
    x[0] = (ytilde[0] - R[1]*x[1] - R[2]*x[2]) / R[0];

    return true;
}

// ============================================================================
// Kalman binary ops (unchanged)
// ============================================================================
void kf::binary_expand_op_1(real_T in1[10], int32_T in2, const real_T in3[2],
  const real_T in4[4], int32_T in5)
{
    int32_T tmp = in2 << 1;
    int32_T tmp0 = (in2 + 1) << 1;
    in1[tmp0]     = in4[tmp] + in3[0];
    in1[tmp0 + 1] = in4[(in5 + 1 != 1) + tmp] + in3[1];
}

void kf::binary_expand_op(real_T in1[10], int32_T in2, const real_T in3[2],
  const real_T in4[4], int32_T in5)
{
    int32_T tmp = in2 << 1;
    int32_T tmp0 = (in2 + 3) << 1;
    in1[tmp0]     = in3[0] - in4[tmp];
    in1[tmp0 + 1] = in3[1] - in4[(in5 + 1 != 1) + tmp];
}

// ============================================================================
// MODEL STEP FUNCTION
// ============================================================================
void kf::step()
{
    // -------------------------------
    // Kalman Filter (unchanged)
    // -------------------------------
    real_T Wc[5];
    real_T Wm[5];
    real_T P_minus[4];
    real_T diff_idx_0;
    real_T diff_idx_1;
    real_T propagated_points;
    real_T accumulatedData;
    real_T ssq;
    real_T x_hat;
    int32_T b_j;
    int32_T idxAjj;
    int32_T ix;
    int32_T nz;
    boolean_T exitg1;

    // ======== KF initialization if first run ========
    if (!rtDW.is_initialized_not_empty) {
        rtDW.x_hat[0] = rtU.baro_altitude_in;
        rtDW.x_hat[1] = 0.0;
        rtDW.P_a[0] = 100.0;
        rtDW.P_a[1] = 0.0;
        rtDW.P_a[2] = 0.0;
        rtDW.P_a[3] = 100.0;
        rtDW.is_initialized_not_empty = true;
    }

    // KF weights
    Wm[0] = -999998.99997124437;
    Wc[0] = -999995.99997224438;
    for (int i=1;i<5;i++){
        Wm[i] = 249999.99999281109;
        Wc[i] = 249999.99999281109;
    }

    // KF prediction
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

    // Cholesky of P_minus (KF part)
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

    idxAjj = (b_j < 1) ? -1 : (b_j - 1);

    if (nz > 0) {
        diff_idx_0 = rtDW.x_hat[0];
        rtY.kf_velocity_out = rtDW.x_hat[1];
    }
    else
    {
        // ----- State propagation -----
        std::memset(&rtDW.sigma_points[0], 0, 10U * sizeof(real_T));

        rtDW.sigma_points[0] = rtDW.x_hat[0];
        rtDW.sigma_points[1] = rtDW.x_hat[1];

        // Sigma points
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
            binary_expand_op  (rtDW.sigma_points, 0, rtDW.x_hat, P_minus, idxAjj);
            binary_expand_op_1(rtDW.sigma_points, 1, rtDW.x_hat, P_minus, idxAjj);
            binary_expand_op  (rtDW.sigma_points, 1, rtDW.x_hat, P_minus, idxAjj);
        }

        // Propagate model
        x_hat = 0.0;
        accumulatedData = 0.0;
        for (nz=0;nz<5;nz++){
            int id = nz<<1;
            double v = rtDW.sigma_points[id+1];

            ssq = (rtU.accel_z_in)
                - 0.6125 * v * std::abs(v) * 0.62 * 0.00725 / 4.25;

            propagated_points =
                (v * 0.02 + rtDW.sigma_points[id]) + 0.5 * ssq * 0.0004;

            rtDW.propagated_points[id] = propagated_points;

            ssq = ssq * 0.02 + v;
            rtDW.propagated_points[id+1] = ssq;

            x_hat          += propagated_points * Wm[nz];
            accumulatedData+= ssq * Wm[nz];
        }

        rtDW.x_hat[0] = x_hat;
        rtDW.x_hat[1] = accumulatedData;

        // Covariance update
        double P00=2.9881050105278437E-5;
        double P01=0.0029881050105278442;
        double P11=0.29881050105278439;
        double tmp01;

        for (nz=0;nz<5;nz++){
            int id = nz<<1;
            double dx0 = rtDW.propagated_points[id]   - x_hat;
            double dx1 = rtDW.propagated_points[id+1] - accumulatedData;

            double W = Wc[nz];

            P00 += dx0*dx0*W;
            P01 += dx1*dx0*W;
            P11 += dx1*dx1*W;
        }

        double S = P00 + 0.0217839917878801;   // R measurement noise
        double K0 = P00 / S;
        double K1 = P01 / S;

        double z = rtU.baro_altitude_in;
        double y = z - (rtDW.x_hat[0]);

        rtDW.x_hat[0] += K0 * y;
        rtDW.x_hat[1] += K1 * y;

        rtDW.P_a[0] = P00 - K0*P00;
        rtDW.P_a[1] = P01 - K1*P00;
        rtDW.P_a[2] = P01 - K0*P01;
        rtDW.P_a[3] = P11 - K1*P01;

        diff_idx_0 = rtDW.x_hat[0];
        rtY.kf_velocity_out = rtDW.x_hat[1];
    }

    // KF altitude out
    rtY.kf_altitude_out = diff_idx_0;



    // ========================================================================
    // APoGEE PREDICTOR — NEW VERSION (25-sample OLS using Cholesky)
    // ========================================================================

    // Shift buffer
    for (int i=0;i<BUFFER_SIZE-1;i++){
        rtDW.time_buffer[i] = rtDW.time_buffer[i+1];
        rtDW.alt_buffer [i] = rtDW.alt_buffer [i+1];
    }
    rtDW.time_buffer[BUFFER_SIZE-1] = rtU.current_time_in;
    rtDW.alt_buffer [BUFFER_SIZE-1] = diff_idx_0;

    if (rtDW.buffer_count < BUFFER_SIZE)
        rtDW.buffer_count++;

    // If not full → no estimate
    if (rtDW.buffer_count < BUFFER_SIZE) {
        rtY.apogee_out = rtNaN;
        rtY.r2_out     = rtNaN;
        return;
    }

    // Build normal equations
    double XtX[9] = {0};
    double Xty[3] = {0};

    for (int i=0;i<BUFFER_SIZE;i++){
        double t = rtDW.time_buffer[i];
        double y = rtDW.alt_buffer [i];

        double x0 = t*t;
        double x1 = t;
        double x2 = 1.0;

        // XᵀX
        XtX[0]+= x0*x0;   XtX[1]+= x0*x1;   XtX[2]+= x0*x2;
        XtX[3]+= x1*x0;   XtX[4]+= x1*x1;   XtX[5]+= x1*x2;
        XtX[6]+= x2*x0;   XtX[7]+= x2*x1;   XtX[8]+= x2*x2;

        // Xᵀy
        Xty[0]+= x0*y;
        Xty[1]+= x1*y;
        Xty[2]+= x2*y;
    }

    double theta[3];
    bool ok = solveNormalEqCholesky(XtX, Xty, theta);
    if (!ok){
        rtY.apogee_out = rtNaN;
        rtY.r2_out     = rtNaN;
        return;
    }

    rtDW.theta[0] = theta[0];
    rtDW.theta[1] = theta[1];
    rtDW.theta[2] = theta[2];

    // Compute apogee from parabola vertex
    if (theta[0] < -1e-9){
        rtY.apogee_out = theta[2] - (theta[1]*theta[1])/(4.0*theta[0]);
    } else {
        rtY.apogee_out = rtNaN;
    }

    // r2 not used
    rtY.r2_out = 0.0;
}

// ============================================================================
// INITIALIZE
// ============================================================================
void kf::initialize()
{
    // Apogee LS buffers initialization
    //25 -> 50
    for (int i=0;i<BUFFER_SIZE;i++){
        rtDW.time_buffer[i] = rtNaN;
        rtDW.alt_buffer [i] = rtNaN;
    }
    rtDW.buffer_count = 0;

    rtDW.theta[0]=rtDW.theta[1]=rtDW.theta[2]=0.0;
}

// ============================================================================
// Model getters
// ============================================================================
const char_T* kf::RT_MODEL::getErrorStatus() const
{
    return (errorStatus);
}

void kf::RT_MODEL::setErrorStatus(const char_T* const volatile aErrorStatus)
{
    (errorStatus = aErrorStatus);
}

kf::kf() :
  rtU(),
  rtY(),
  rtDW(),
  rtM()
{}

kf::~kf() = default;

kf::RT_MODEL * kf::getRTM()
{
  return (&rtM);
}

// ============================================================================
// EOF
// ============================================================================
