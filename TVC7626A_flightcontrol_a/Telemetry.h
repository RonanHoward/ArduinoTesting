#ifndef Telemetery_h
#define Telemetery_h

#define TELEMETRY_SAMPLES_PER_SEC 10
#define TELEMETRY_LOG_TIME_SEC 50
#define TELEMETRY_LOG_DELAY_MS 3000

class Telemetry {
  public:
    Telemetry();

    unsigned int current_slot;

    float logs[TELEMETRY_SAMPLES_PER_SEC*TELEMETRY_LOG_TIME_SEC][9];

    void log(float time, float l1, float l2, float l3, float l4, float l5, float l6, float l7, float l8);
    
};

#endif
