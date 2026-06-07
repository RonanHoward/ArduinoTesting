// This program turns the built-in led on when sufficient voltage is detected on the launch event pin


// Ignition
#define IGNITION_PIN A0
#define THRESHOLD_VOLTAGE 2.5f // Threshold for launch (adjust this)
#define ADC_MAX 4095
#define VREF 3.3f
// Rocket State
enum class FlightState : uint8_t {
  PRE_LAUNCH,
  FLIGHT,
  OUTPUT_TELEMETRY
};

FlightState flightState = FlightState::PRE_LAUNCH;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  analogReadResolution(12);

  digitalWrite(LED_BUILTIN, LOW);

}

void loop() {

  if (flightState == FlightState::FLIGHT) {
    controlFlight();
  } else if (flightState == FlightState::PRE_LAUNCH) {
    detectLaunch();
  } else {
    outputTelemetry();
  }

}

void detectLaunch() {
  // Read analog pin
  // NOTE: this is not debounced, consider this in future revisions
  int raw = analogRead(IGNITION_PIN);
  float voltage = raw * (VREF / ADC_MAX);

  if (voltage >= THRESHOLD_VOLTAGE) {
    flightState = FlightState::FLIGHT;
  }
}

void controlFlight() {
  digitalWrite(LED_BUILTIN, HIGH);
}

void outputTelemetry() {
  return;
}
