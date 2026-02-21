#include "EKF.h"
#include <math.h>
#include "Arduino_BMI270_BMM150.h"

EKF::EKF() {
  // tune these parameters
  float P0 = 0.1f;
  float Q0[2] = {0.001f, 0.001f};
  float R0[3] = {0.011f, 0.011f, 0.011f};

  // non-configurable
  phi_rad = 0.0f;
  theta_rad = 0.0f;

  axPrev = 0.0f;
  ayPrev = 0.0f;
  azPrev = 0.0f;
  gxPrev = 0.0f;
  gyPrev = 0.0f;
  gzPrev = 0.0f;

  P[0] = P0;
  P[1] = 0.0f;
  P[2] = 0.0f;
  P[3] = P0;

  Q[0] = Q0[0];
  Q[1] = Q0[1];

  R[0] = R0[0];
  R[1] = R0[1];
  R[2] = R0[2];
}


void EKF::predict(float gx, float gy, float gz, float T) {
  // TODO: apply low-pass filter
  float p = gx;
  float q = gy;
  float r = gz;

  // Precompute some useful terms
  float sp = sin(phi_rad);
  float cp = cos(phi_rad);
  float tt = tan(theta_rad);

  // x[+] = x[-] + T * f(x,u)
  phi_rad   = phi_rad   + T * (p + tt * (q * sp + r * cp));
  theta_rad = theta_rad + T * (q * cp - r * sp);

  // Recompute the useful terms for new state estimates
  sp = sin(phi_rad);
  cp = cos(phi_rad);
  float st = sin(theta_rad);
  float ct = cos(theta_rad);
  tt = st / ct;

  // Compute the Jacobian of f(x, u)
  float J[4] = {
    tt * (q * cp - r * sp),  (r * cp + q * sp) * (tt * tt + 1.0f),
    -(r * cp + q * sp),      0.0f
  };

  // Update covariance
  float P_temp[4] = {
    T * ( Q[0] + 2.0f*J[0]*P[0] + J[1]*P[1] + J[1]*P[2] ),
    T * ( J[0]*P[1] + J[2]*P[0] + J[1]*P[3] + J[3]*P[1] ),
    T * ( J[0]*P[2] + J[2]*P[0] + J[1]*P[3] + J[3]*P[2] ),
    T * ( Q[1] + J[2]*P[1] + J[2]*P[2] + 2.0f*J[3]*P[3] )
  };

  P[0] = P[0] + P_temp[0];
  P[1] = P[1] + P_temp[1];
  P[2] = P[2] + P_temp[2];
  P[3] = P[3] + P_temp[3];

}

void EKF::update(float ax, float ay, float az) {

  // Precompute some useful terms
  float sp = sin(phi_rad);
  float cp = cos(phi_rad);
  float st = sin(theta_rad);
  float ct = cos(theta_rad);

  // Find output function h(x,u)
  float h[3] = {
    g * st,
    -g * ct * sp,
    -g * ct * cp
  };

  // Compute jacobian of h(x,u)
  float C[6] = {
    0.0f,           g * ct,
    -g * cp * ct,   g * sp * st,
    g * sp * ct,    g * cp * st
  };

  // Compute the Kalman gain K = P * C' / (C * P * C' + R)

  // C * P * C' + R
  float G[9] = {
    P[3]*C[1]*C[1] + R[0],         C[1]*C[2]*P[2] + C[1]*C[3]*P[3],                                     C[1]*C[4]*P[2] + C[1]*C[5]*P[3],
    C[1]*(C[2]*P[1] + C[3]*P[3]),  R[1] + C[2]*(C[2]*P[0] + C[3]*P[2]) + C[3]*(C[2]*P[1] + C[3]*P[3]),  C[4]*(C[2]*P[0] + C[3]*P[2]) + C[5]*(C[2]*P[1] + C[3]*P[3]),
    C[1]*(C[4]*P[1] + C[5]*P[3]),  C[2]*(C[4]*P[0] + C[5]*P[2]) + C[3]*(C[4]*P[1] + C[5]*P[3]),         R[2] + C[4]*(C[4]*P[0] + C[5]*P[2]) + C[5]*(C[4]*P[1] + C[5]*P[3])
  };
  // Inverse of determinant
  float Gdetinv = 1.0f / (G[0]*G[4]*G[8] - G[0]*G[5]*G[7] - G[1]*G[3]*G[8] + G[1]*G[5]*G[6] + G[2]*G[3]*G[7] - G[2]*G[4]*G[6]);
  // Find inverse
  float Ginv[9] = {
     Gdetinv * (G[4]*G[8] - G[5]*G[7]), -Gdetinv * (G[1]*G[8] - G[2]*G[7]),  Gdetinv * (G[1]*G[5] - G[2]*G[4]),
    -Gdetinv * (G[3]*G[8] - G[5]*G[6]),  Gdetinv * (G[0]*G[8] - G[2]*G[6]), -Gdetinv * (G[0]*G[5] - G[2]*G[3]),
     Gdetinv * (G[3]*G[7] - G[4]*G[6]), -Gdetinv * (G[0]*G[7] - G[1]*G[6]),  Gdetinv * (G[0]*G[4] - G[1]*G[3])
  };
  // K = P * C' * Ginv
  float K[6] = {
    Ginv[3]*(C[2]*P[0] + C[3]*P[1]) + Ginv[6]*(C[4]*P[0] + C[5]*P[1]) + C[1]*Ginv[0]*P[1],  Ginv[4]*(C[2]*P[0] + C[3]*P[1]) + Ginv[7]*(C[4]*P[0] + C[5]*P[1]) + C[1]*Ginv[1]*P[1],  Ginv[5]*(C[2]*P[0] + C[3]*P[1]) + Ginv[8]*(C[4]*P[0] + C[5]*P[1]) + C[1]*Ginv[2]*P[1],
    Ginv[3]*(C[2]*P[2] + C[3]*P[3]) + Ginv[6]*(C[4]*P[2] + C[5]*P[3]) + C[1]*Ginv[0]*P[3],  Ginv[4]*(C[2]*P[2] + C[3]*P[3]) + Ginv[7]*(C[4]*P[2] + C[5]*P[3]) + C[1]*Ginv[1]*P[3],  Ginv[5]*(C[2]*P[2] + C[3]*P[3]) + Ginv[8]*(C[4]*P[2] + C[5]*P[3]) + C[1]*Ginv[2]*P[3]
  };

  // Update covariance matrix: P++ = (I - K * C) * P+
  float Ptmp[4] = {
    -P[0]*(C[2]*K[1] + C[4]*K[2] - 1.0f) - P[2]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]), -P[1]*(C[2]*K[1] + C[4]*K[2] - 1.0f) - P[3]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]),
    -P[0]*(C[2]*K[4] + C[4]*K[5]) - P[2]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f), -P[1]*(C[2]*K[4] + C[4]*K[5]) - P[3]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f)
  };

  P[0] = P[0] + Ptmp[0];
  P[1] = P[1] + Ptmp[1];
  P[2] = P[2] + Ptmp[2];
  P[3] = P[3] + Ptmp[3];

  // Update state estimate
  phi_rad   = phi_rad   + K[0] * (ax - h[0]) + K[1] * (ay - h[1]) + K[2] * (az - h[2]);
  theta_rad = theta_rad + K[3] * (ax - h[0]) + K[4] * (ay - h[1]) + K[5] * (az - h[2]);
}
