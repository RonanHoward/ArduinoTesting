#ifndef EKF_RollPitch_h
#define EKF_RollPitch_h

#include <math.h>

#define g ((float) 9.81f)

class EKF_RollPitch {

  public:

    float phi_rad;
    float theta_rad;

    // Diagonal matricies
    float P[4]; // error covariance
    float Q[2]; // process noise matrix
    float R[3]; // measurment noise matrix

    EKF_RollPitch(float P0, float *Q0, float *R0);
    void predict(float gx, float gy, float gz, float T);
    void update(float ax, float ay, float az);

};

#endif