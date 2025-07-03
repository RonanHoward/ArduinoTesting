/******************************************************************************

Proportional Correction Test V0R1
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
SCK~D13 |1        30| D12~CIPO                     |1        30|
   +3V3 |2        29| D11~COPI                +3V3 |2        29|
   AREF |3        28| D10~                    AREF |3        28|
     A0 |4        27| D9~                          |4        27|
     A1 |5        26| D8~                          |5        26|
     A2 |6        25| D7~                          |6        25|
     A3 |7        24| D6~                          |7        24|
 SDA A4 |8        23| D5~                          |8        23|
 SCL A5 |9        22| D4~                          |9        22|
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
#include <Servo.h>

Servo Xaxis_servo;
Servo Yaxis_servo;

// EDITORIAL VARIABLES (TUNE THESE)

const float SERVO_ANGLE_LIMIT = 1.0;        // use 1.0 for full range, 0.0 for no range. Affects servo range according to this graph https://www.desmos.com/calculator/pz3dhdu8sx
const float PROPORTIONALITY_CONSTANT = 1.0; // tvc angle multiplied by this float each iteration. range 0.0 to 1.0 recommended

// variable declarations

const long MAP_RADIUS = (long) floor( 90 * SERVO_ANGLE_LIMIT );
const long LOWER_MAP_BOUND = 90 - MAP_RADIUS;
const long UPPER_MAP_BOUND = 90 + MAP_RADIUS;

float x, y, z;
int degreesX = 0;
int degreesY = 0;

long Xaxis_command;
long Yaxis_command;


/********************************** SETUP *************************************/

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  // use D12 and D11 for servo A and B, use D4 and D5 for servo Q1 and Q2
  Xaxis_servo.attach(D4);
  Yaxis_servo.attach(D5);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (SERVO_ANGLE_LIMIT < 0.0 || SERVO_ANGLE_LIMIT > 1.0) {
    Serial.println("SERVO_ANGLE_LIMIT must be between 0.0 and 1");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
}

/********************************** MAIN LOOP *********************************/


void loop() {

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    // xz-plane
    double angleX = atan( x/z ) * PROPORTIONALITY_CONSTANT; // bounded by [-PI/2, PI/2] which is roughly [-1.570796, 1.570796]
    long intAngleX = (long) floor(angleX * 100); // new bounds: [-157, 157]
    // yz-plane
    double angleY = atan( y/z ) * PROPORTIONALITY_CONSTANT;
    long intAngleY = (long) floor(angleY * 100);


    // create servo commands

    Xaxis_command = map (intAngleX, -157, 157, LOWER_MAP_BOUND, UPPER_MAP_BOUND);
    Yaxis_command = map (intAngleY, -157, 157, LOWER_MAP_BOUND, UPPER_MAP_BOUND);

    // write servo commands

    Xaxis_servo.write(Xaxis_command);
    Yaxis_servo.write(Yaxis_command);
    

    // print acceleration vector
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }

  delay(50);

}


  /***************************** INTERRUPT SERVICE ******************************/


  /***************************** CALLED SUBROUTINES ****************************/
  

  /************************************ END **********************************/
