/**********************************************************************************************

PseudoCode:

p[n] = Kp * e[n] ;
i[n] = i[n-1] + (Ki*T/2)*(e[n] + e[n-1]) ;
d[n] = d[n-1]*(2t+T)/(2t - T) + 2Kd / (2t + T)*(e[n] - e[n-1]) ;

u[n] = p[n] + i[n] + d[n] ;

**********************************************************************************************/

#ifndef PID_h
#define PID_h

#define PID_FILTER_PERIOD 0.001
#define PID_CONTROL 0

class PID {
  public:
    PID();
    void update(float measurement);
    float get_un();
    float get_frequency();
  private:
    // constants
    const float control = 0.0f;
    const float Kp = 1.20f;
    const float Ki = 0.80f;
    const float Kd = 0.33f;
    const float t = 0.05f;
    const float T = PID_FILTER_PERIOD;
    const float frequency = 1 / PID_FILTER_PERIOD;
    const float int_limit_max = 0.001f;
    const float int_limit_min = -0.001f;
    // mutables
    float e_n = 0.0f;
    float e_n_1 = 0.0f;
    float d_n_1 = 0.0f;
    float u_n = 0.0f;
    float p_n = 0.0f;
    float i_n = 0.0f;
    float d_n = 0.0f;
};

#endif