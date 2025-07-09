/******************************************************************************

Madgwick AHRS Proportional Vectoring V0R0
JULY 2025 (c) Concept Propulsion.  All Rights Reserved.
R. HOWARD

Portions of avr-libc are Copyright (c) 1999-2016  All Rights Reserved

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

       Nano 33 Sense R2                            This Project

        ||||| V |||||                              ||||| V |||||
SCK~D13 |1        30| D12~CIPO                     |1        30| Servo A
   +3V3 |2        29| D11~COPI                +3V3 |2        29| Servo B
   AREF |3        28| D10~                    AREF |3        28|
     A0 |4        27| D9~                          |4        27|
     A1 |5        26| D8~                          |5        26|
     A2 |6        25| D7~                          |6        25|
     A3 |7        24| D6~                          |7        24|
 SDA A4 |8        23| D5~                          |8        23| Q2 Servo
 SCL A5 |9        22| D4~                          |9        22| Q1 Servo
     A6 |10       21| D3~                          |10       21|
     A7 |11       20| D2~                          |11       20|
    +5v |12       19| GND                      +5V |12       19| GND
  RESET |13       18| RESET                  RESET |13       18| RESET
    GND |14       17| D1 RX                    GND |14       17|
    VIN |15       16| D0 TX                    VIN |15       16|
        |||||||||||||                            ||||||||||||| 

********************************* RESOURCES **********************************/

// Include the Nano 33 Sense R2 IMU methods library
// Default values:
// Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
// Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
// Magnetometer range is set at ±1300 uT with a resolution of 0.3 uT.
// Accelerometer and gyrospcope output data rate is fixed at 99.84 Hz.
// Magnetometer output data rate is fixed at 10 Hz.


#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h>
#include <Servo.h>



class Gimbal {
  public:
    Servo Xaxis_servo;
    Servo Yaxis_servo;

    int HARD_SERVO_LIMIT;

    Gimbal() {
      HARD_SERVO_LIMIT = 30;
    }
    
    void attach() {

      Xaxis_servo.attach(D4);
      Yaxis_servo.attach(D5);

    }

    // inputs are identical to two servos as if they were oriented with the arduino
    void write(long pitch_command, long yaw_command) {

      // create temp variables to operate on and center the range (i.e. from [0,180] to [-90,90])
      float temp_pitch = pitch_command - 90;
      float temp_yaw = yaw_command - 90;

      // rotate command point 45 degrees
      long q1_servo_command = round(temp_pitch * 0.707106781187 - temp_yaw * 0.707106781187);
      long q2_servo_command = round(temp_yaw * 0.707106781187 + temp_pitch * 0.707106781187);

      // apply hard servo limit
      if (q1_servo_command > HARD_SERVO_LIMIT) {
        q1_servo_command = HARD_SERVO_LIMIT;
      } else if (q1_servo_command < -HARD_SERVO_LIMIT) {
        q1_servo_command = -HARD_SERVO_LIMIT;
      }
      if (q2_servo_command > HARD_SERVO_LIMIT) {
        q2_servo_command = HARD_SERVO_LIMIT;
      } else if (q2_servo_command < -HARD_SERVO_LIMIT) {
        q2_servo_command = -HARD_SERVO_LIMIT;
      }

      // shift back to range [0,180]
      q1_servo_command = q1_servo_command + 90;
      q2_servo_command = q2_servo_command + 90;

      // write computed commands
      Xaxis_servo.write(q1_servo_command);
      Yaxis_servo.write(q2_servo_command);

    }
  
};





Madgwick filter;
Gimbal myGimbal;
unsigned long microsPerReading, microsPrevious, microsNow;

void setup() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  myGimbal.attach();

  filter.begin(104.00);

  microsPerReading = 9615; // 104Hz sample rate
  microsPrevious = micros();
}


float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, heading;

void loop() {

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // Read sensors and update filter
    // verify that data is ready
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      IMU.readMagneticField(mx, my, mz);

      filter.update(gx, gy, gz, ax, ay, az, mx, -my, -mz);

      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw();
      Serial.print("Orientation: ");
      Serial.print(heading);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.println(roll);
    } else {
      Serial.println("Unable to read IMU value(s)");
    }

    microsPrevious = microsPrevious + microsPerReading;

    
    // command
    long command_x;
    long command_y;
    if (pitch > myGimbal.HARD_SERVO_LIMIT) { command_x = myGimbal.HARD_SERVO_LIMIT; }
    else if (pitch < -myGimbal.HARD_SERVO_LIMIT) { command_x = -myGimbal.HARD_SERVO_LIMIT; }
    else { command_x = round(pitch); }
    if (roll > myGimbal.HARD_SERVO_LIMIT) { command_y = myGimbal.HARD_SERVO_LIMIT; }
    else if (roll < -myGimbal.HARD_SERVO_LIMIT) { command_y = -myGimbal.HARD_SERVO_LIMIT; }
    else { command_y = round(roll); }
    command_x = command_x + 90;
    command_y = command_y + 90;
    myGimbal.write(command_y, command_x);

  }

  
  
}
