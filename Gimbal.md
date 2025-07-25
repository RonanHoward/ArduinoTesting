# TVC-7626 Gimbal Library

Official library interface for the TVC-7627 by Concept Propulsion™. This library provides abstracted methods to allow for a intuitive programming experience.


## Basic Information

The TVC-7627 uses two SG90 servos to control the angle of the gimbal. Due to the compact gimbal design, there exists no general solution for the relationship between servo position and gimbal position (read more). To solve this issue, this library uses a lookup table that directly relates each possible gimbal position to it's associated servo commands. The lookup table is stored in a 2 dimensional K-D tree in order to optimize runtime efficiency.

## Quickstart

Here is an example of an Arduino sketch which implements the gimbal library. This script simply initializes the gimbal and sets it to the flush position.

```cpp
#include <Gimbal.h>

Gimbal myGimbal;

void setup() {

    myGimbal.attach();

    myGimbal.write(0, 0);

}

void loop() {

    // ...

}
```

## Usage

This library has two main classes, `Gimbal` and `KDTree`. The `Gimbal` class is the main class used to send commands to the TVC-7626; it utilizes the `KDTree` class to efficiently convert gimbal angles to servo commands.

### `Gimbal()`

- `KDTree lookup_tree` - The lookup table in a KD tree data data structure
- `int HARD_SERVO_LIMIT` - Maximum radius (in servo command values) servo can commanded to from flush position
- `int SERVO_A_FLUSH` - Servo (A) command required to set gimbal to flush position.
- `int SERVO_B_FLUSH` - Servo (B) command required to set gimbal to flush position.

- `Gimbal()` *(constructor)*
    - Initializes gimbal and generates lookup table

- `void Gimbal::attach()`
    - Attaches servos from pins D4 and D5

- `void Gimbal::write(float x_angle, float y_angle)`
    - Writes to servos in order to position gimbal at specified angles


### `KDTree()`

- `KDTree()` *(constructor)*
    - Initializes an empty KDTree. (i.e. `global_root = nullptr`)
