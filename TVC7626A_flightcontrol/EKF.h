/*
  EKF.h - A sensor fusion library for the Arduino nano 33 BLE sense rev2.
  Developed for rocketry.
  Created by Ronan Howard, January 27, 2026.
  Released into the public domain.
*/
#ifndef EKF_h
#define EKF_h

#include <math.h>
#include "Arduino_BMI270_BMM150.h"

#define g ((float) 9.81f)

class EKF {

  public:

    // current state
    float phi_rad;
    float theta_rad;

    // Low Pass Filters
    float a_alpha, g_alpha;
    float axPrev, ayPrev, azPrev;
    float gxPrev, gyPrev, gzPrev;

    // Diagonal matricies
    float P[4]; // error covariance
    float Q[2]; // process noise matrix
    float R[3]; // measurment noise matrix

    float aax, aay, aaz, agx, agy, agz; // alpha values for IIR pre-filter
    float fax, fay, faz, fgx, fgy, fgz; // filtered readings

    EKF();

    void lpf_acc(float ax, float ay, float az);
    void lpf_gyro(float gx, float gy, float gz);

    void predict(float T);
    void update();

};

#endif