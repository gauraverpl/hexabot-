# Hexabot

A six-legged Arduino Mega robot using 24 hobby servos, an HC-SR04 ultrasonic sensor, and a Bluetooth serial module. The code supports manual remote control and basic autonomous obstacle avoidance.

## Hardware

- Arduino Mega 2560
- 24 hobby servos (4 per leg)
- 6-leg hexapod chassis
- HC-SR04 ultrasonic sensor
- Bluetooth UART module (HC-05, HC-06, or compatible)
- Separate regulated power supply for the servos
- Common ground between Arduino, Bluetooth module, sensor, and servo supply
- Fuse and/or suitable protection for the servo power circuit

> **Warning:** Do not power 24 servos from the Arduino 5 V regulator or pins. Use an external supply rated for the servos, add a common ground, and verify polarity and current capacity.

## Wiring

| Component | Arduino Mega pin |
|---|---|
| HC-SR04 TRIG | 46 |
| HC-SR04 ECHO | 47 |
| Bluetooth RX | TX1 pin 18 |
| Bluetooth TX | RX1 pin 19 |
| Servo signals | 22–45 |

Bluetooth module logic levels must be compatible with the Arduino. Many HC-05/HC-06 modules use 3.3 V logic and require a voltage divider or level shifter on Arduino TX → Bluetooth RX.

## Servo assignment

The sketch groups the 24 servos into six legs with four joints per leg:

- Legs: front-left, middle-left, rear-left, front-right, middle-right, rear-right
- Joints per leg: coxa/hip, femur, tibia, foot
- Mega pins: 22–25, 26–29, 30–33, 34–37, 38–41, and 42–45

Check and update these values in `hexabot.ino` before operation:

- `SERVO_PINS`
- `NEUTRAL`
- `SERVO_DIR`
- `STRIDE_DEG`
- `LIFT_DEG`
- `KNEE_LIFT_DEG`

The supplied gait is a configurable baseline, not universal inverse kinematics. Servo offsets and directions depend on the mechanical layout.

## Bluetooth commands

Send commands at 9600 baud using an application or terminal:

| Command | Action |
|---|---|
| `F` | Forward |
| `B` | Backward |
| `L` | Turn left |
| `R` | Turn right |
| `S` | Stand / stop |
| `A` | Automatic obstacle avoidance mode |
| `M` | Return to manual mode |

Lowercase commands are accepted as well.

## Operation

1. Mount or support the robot so it cannot fall.
2. Connect the external servo supply and turn off the robot until wiring is checked.
3. Flash `hexabot.ino` to the Arduino Mega.
4. Open Serial Monitor at 115200 baud for diagnostics.
5. Pair and connect the Bluetooth module over UART at 9600 baud.
6. Test `S`, then test individual directions while the robot is supported.
7. Tune the neutral angles and directions for a stable standing posture.
8. Lower the robot gradually and adjust stride/lift values until the gait is stable.

## Obstacle avoidance

In automatic mode, the robot:

1. Moves forward.
2. Stops when the HC-SR04 detects an obstacle within 25 cm.
3. Moves backward briefly.
4. Turns briefly in an alternating direction.
5. Resumes forward walking.

The threshold is controlled by `OBSTACLE_CM`. Sensor timing, reverse duration, and turn duration can be adjusted with `SENSOR_INTERVAL_MS`, `AUTO_REVERSE_MS`, and `AUTO_TURN_MS`.

## Safety

- Keep fingers and clothing away from moving servos.
- Start with low-power, low-speed mechanical tests.
- Verify servo range limits before using full walking cycles.
- Secure battery, wiring, and servo mounting hardware.
- Use a separate software/e-stop or hardware master switch for reliable emergency stopping.
