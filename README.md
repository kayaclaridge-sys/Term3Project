Robot Control Software README
1. Project Overview
This repository contains the Arduino software used for the robotics challenge robot.  
The code is designed for an Arduino Giga R1 WiFi robot platform using:
QTR 9-channel RC line sensor array
Motoron I2C motor controllers
HC-SR04 ultrasonic distance sensors
WS1850S / MFRC522 I2C RFID reader
Servo-based planting / bead release mechanism
Start / stop buttons
Red / green status LEDs
The software is split into several test and task programs. Each program focuses on one robot behaviour, such as line following, RFID navigation, sensor debugging, servo planting, wall following, or emergency stop handling.
---
2. Software Files
File / Sketch	Purpose
`sensor\_motor\_integration\_test.ino`	Standard QTR line following with Motoron drive, junction detection, right-angle turns, hollow-cross handling, and line recovery.
`trial\_run2\_task2\_line\_tracking.ino`	Task 2 line tracking mode. The first T-junction is forced to turn right.
`trial\_run2\_task3\_rfid\_navigation.ino`	RFID-based navigation. Reads RFID tags, maps them to a 9 × 9 grid, selects route actions, and performs straight / left / right / stop decisions.
`integrated\_sensor\_debug\_servo.ino`	Debug sketch for QTR sensors, ultrasonic distance sensors, revive buttons, LEDs, and the servo planting mechanism.
`wall\_following.ino`	Task 6 wall following using ultrasonic sensors and PD control. Includes front obstacle avoidance and stop / start controls.
---
3. Required Arduino Libraries
Install the following libraries before uploading:
```cpp
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522\_I2C.h>
#include <Servo.h>
```
Required libraries:
Motoron library for Pololu Motoron I2C motor controllers
MFRC522_I2C library for the I2C RFID reader
Servo library for the planting / bead release mechanism
Built-in Wire library for I2C communication
---
4. Hardware Summary
Main Controller
Board: Arduino Giga R1 WiFi
I2C bus used by Motoron and RFID: Wire1
Serial baud rate: 115200
Motor Controllers
Component	I2C Address	Bus	Notes
Left Motoron	`0x10`	Wire1	Channel 1 = front-left, channel 2 = rear-left
Right Motoron	`0x11`	Wire1	Channel 1 = front-right, channel 2 = rear-right
Motor direction signs used in the code:
```cpp
LEFT\_MOTOR\_SIGN  = +1
RIGHT\_MOTOR\_SIGN = -1
```
This means the left and right Motoron commands are inverted differently so that positive robot speed means forward movement.
---
5. Pin Assignments
QTR 9-Channel Line Sensor
QTR Output	Arduino Pin
OUT1	D22
OUT2	D23
OUT3	D24
OUT4	D25
OUT5	D26
OUT6	D27
OUT7	D28
OUT8	D29
OUT9	D30
Emitter	D31
Buttons and LEDs
Component	Pin	Mode / Meaning
Start button	D32	`INPUT\_PULLUP`, pressed = LOW
Stop button	D33	`INPUT\_PULLUP`, pressed = LOW
Red LED	D34	ON when stopped / idle
Green LED	D35	ON when running
Ultrasonic Sensors
For the latest wall-following code, use the actual `#define` values:
Sensor	Trigger Pin	Echo Pin
Front ultrasonic	D36	D37
Left ultrasonic	D39	D38
Right ultrasonic	D41	D40
> Note: If older comments mention front ultrasonic D2/D3, ignore that comment and follow the actual `#define FRONT\_TRIG 36` and `#define FRONT\_ECHO 37`.
Servo Planting Mechanism
Component	Pin
Servo signal	D47
Servo limits:
```cpp
SERVO\_MIN\_US = 500
SERVO\_MAX\_US = 2500
SERVO\_RANGE\_DEG = 300
STEP\_ANGLE = 60
```
---
6. Main Software Architecture
The overall robot program follows this structure:
```text
setup()
  ├── Start Serial
  ├── Configure buttons and LEDs
  ├── Configure QTR / ultrasonic / RFID / servo pins
  ├── Start Wire1 I2C bus
  ├── Initialise Motoron controllers
  ├── Initialise RFID reader if used
  ├── Calibrate QTR sensors
  └── Enter idle state

loop()
  ├── Read Serial commands
  ├── Read start / stop buttons
  ├── Poll RFID reader if RFID mode is active
  ├── Read sensors
  ├── Update robot state machine
  ├── Send motor commands
  └── Update status LEDs
```
---
7. Line Following Behaviour
The line-following programs use the QTR sensor array to estimate the black line position.
Sensor Processing
Charge each RC sensor.
Measure discharge time.
Convert raw readings into calibrated values from `0` to `1000`.
Calculate weighted line position.
Compare the position against the centre value.
The centre position is calculated as:
```cpp
CENTER\_POSITION = (QTR\_SENSOR\_COUNT - 1) \* 1000 / 2
```
For 9 sensors, the centre is approximately `4000`.
PD Control
The line follower uses a PD controller:
```cpp
error = position - CENTER\_POSITION
derivative = error - lastError
turn = KP \* error + KD \* derivative
```
Then the turn correction is applied to the motors:
```cpp
leftSpeed  = base + turn
rightSpeed = base - turn
```
Line Following States
State	Meaning
`STATE\_IDLE`	Robot is stopped.
`STATE\_FOLLOW\_LINE`	Normal PD line following.
`STATE\_PRE\_TURN`	Moves forward briefly before a pivot turn.
`STATE\_TURN\_LEFT`	Performs a left pivot turn until the centre line is found.
`STATE\_TURN\_RIGHT`	Performs a right pivot turn until the centre line is found.
`STATE\_BRIDGE\_HOLLOW`	Drives across a hollow-cross / centre-gap area.
`STATE\_RECOVER\_LINE`	Spins toward the last known line position to recover.
Junction Detection
The robot detects:
T-junctions
Left right-angle bends
Right right-angle bends
Hollow crosses / centre gaps
Lost-line conditions
In Task 2 mode, the first T-junction is forced to turn right. After that, extra junctions can use debug decisions such as left, right, or stop.
---
8. RFID Navigation Behaviour
The RFID navigation program combines line following with RFID-based route decisions.
RFID Hardware
Item	Value
RFID I2C address	`0x28`
Bus	`Wire1`
Poll interval	`80 ms`
Reset pin	`-1`, not used
RFID Map
The program stores a 9 × 9 RFID grid:
```cpp
MAP\_ROWS = 9
MAP\_COLS = 9
MAX\_ROUTE\_POINTS = 81
```
Each RFID UID corresponds to one grid node.
Navigation Modes
Mode	Description
`NAV\_MODE\_TASK3\_FIXED`	Uses a fixed Task 3 action sequence.
`NAV\_MODE\_AUTO\_ROUTE`	Builds a route from a start UID to a goal UID.
Fixed Task 3 Action Sequence
```text
1. STRAIGHT
2. STRAIGHT
3. RIGHT
4. LEFT
5. STRAIGHT
```
After the fixed action sequence is complete, the next new RFID causes the robot to stop.
RFID Route Action Flow
```text
RFID detected
  └── Read UID
      └── Ignore repeated UID
          └── Match UID to route / fixed sequence
              └── Choose action
                  ├── STRAIGHT → continue line following
                  ├── LEFT     → enter PRE\_TURN, then TURN\_LEFT
                  ├── RIGHT    → enter PRE\_TURN, then TURN\_RIGHT
                  └── STOP     → stopRobot()
```
---
9. Servo / Planting Mechanism
The servo code controls a bead or planting release mechanism.
Servo Commands
Command	Action
`a`	Rotate servo clockwise by 60 degrees
`d`	Rotate servo counter-clockwise by 60 degrees
`x`	Reset servo to 0 degrees
The movement is smoothed by stepping the servo angle gradually:
```cpp
MOVE\_STEP\_DEG = 2
MOVE\_DELAY\_MS = 8
```
This prevents sudden motion and makes the release action more controlled.
---
10. Wall Following Behaviour
The wall-following program is used for Task 6.
Strategy
At startup, read the left and right ultrasonic sensors.
Choose the closer valid wall.
Follow that wall at a target distance.
Use a PD controller to correct the steering.
If the front sensor detects an obstacle, turn away from the wall.
Continue following once the front path is clear.
Main Wall-Following Parameters
Parameter	Value	Meaning
`TARGET\_DIST\_CM`	`11.0`	Target distance from robot edge to wall
`WALL\_DEADBAND\_CM`	`1.5`	Ignore small distance error
`KP`	`26.0`	Proportional wall-following gain
`KD`	`12.0`	Derivative wall-following gain
`BASE\_SPEED`	`600`	Normal cruise speed
`MAX\_SPEED`	`800`	Maximum motor command
`FRONT\_STOP\_CM`	`24.0`	Start avoidance if obstacle is closer than this
`FRONT\_CLEAR\_CM`	`34.0`	Resume following when front path is clear
`TURN\_MS`	`450`	Duration of avoidance turn
Wall Following States
State	Meaning
`STOPPED`	Motors stopped.
`FOLLOWING`	PD wall following is active.
`AVOIDING`	Front obstacle detected; robot turns away.
Wall Following PD Control
```cpp
error = wallDist - TARGET\_DIST\_CM
derivative = (error - prevError) / dt
correction = KP \* error + KD \* derivative
```
If following the left wall:
```cpp
leftSpeed  = base - correction
rightSpeed = base + correction
```
If following the right wall:
```cpp
leftSpeed  = base + correction
rightSpeed = base - correction
```
---
11. Emergency Handling and Kill Switch
The robot includes several stop and recovery behaviours.
Manual Stop
The robot can be stopped by:
Pressing the stop button on D33
Sending `s` in Serial Monitor
When stopped:
```text
Motors = 0
Robot state = IDLE / STOPPED
Red LED = ON
Green LED = OFF
```
Line Loss Recovery
If the QTR line is lost:
The robot briefly searches toward the last known line position.
If the line is not found within the recovery timeout, it stops.
If the line is found, it returns to `STATE\_FOLLOW\_LINE`.
Turn Timeout
If a pivot turn takes too long and the centre line is not found, the robot enters recovery mode instead of spinning forever.
Wall Following Obstacle Avoidance
If the front ultrasonic sensor detects an obstacle closer than `FRONT\_STOP\_CM`, the robot enters `AVOIDING`, turns away, then resumes once the front distance is greater than `FRONT\_CLEAR\_CM`.
---
12. Serial Commands
Standard Line Following / Task 2
Command	Function
`g`	Start full line-following / Task 2 mode
`2`	Start Task 2 mode
`o`	Start line-only mode without junction state machine
`s`	Stop robot
`c`	Recalibrate QTR sensors
`p`	Print one sensor snapshot
`m`	Toggle live sensor monitor
`l`	Set next T-junction decision to left
`r`	Set next T-junction decision to right
`x`	Set next T-junction decision to stop
`a`	Force left turn state
`d`	Force right turn state
`h` or `?`	Print help
RFID Navigation
Command	Function
`1`	Select fixed Task 3 action sequence
`2`	Select auto-route mode
`start <uid>`	Set auto-route start RFID
`goal <uid>`	Set auto-route goal RFID
`heading north/east/south/west`	Set starting heading
`g`	Start selected navigation mode
`o`	Start line-only mode, RFID actions disabled
`s`	Stop
`route`	Print active route or fixed action sequence
`rfid`	Print latest RFID
`qtr reset`	Reset and recalibrate QTR
Sensor Debug / Servo Test
Command	Function
`u`	Show QTR line sensor readings
`i`	Show HC-SR04 distance readings
`s`	Stop display
`a`	Servo clockwise 60 degrees
`d`	Servo counter-clockwise 60 degrees
`x`	Servo reset to 0 degrees
Wall Following
Command	Function
`g`	Start wall following
`s`	Stop wall following
`w`	Swap followed wall side
`d`	Print one distance sensor dump
---
13. How to Run
Uploading a Sketch
Open the required `.ino` file in Arduino IDE.
Select Arduino Giga R1 WiFi as the board.
Install required libraries.
Connect the robot by USB.
Upload the sketch.
Open Serial Monitor at 115200 baud.
Running Line Following
Place the robot on the black line.
Upload the line-following sketch.
Wait for QTR calibration.
Move the sensor array over both black tape and pale floor during calibration.
Press D32 or send `g`.
Press D33 or send `s` to stop.
Running RFID Navigation
Upload the RFID navigation sketch.
Make sure the RFID reader is connected to `Wire1`.
Select mode:
Send `1` for fixed Task 3 route.
Send `2` for auto route.
For auto route, set:
`start <uid>`
`goal <uid>`
`heading east`
Send `g` to start.
The robot follows the line and makes route decisions when RFID nodes are detected.
Running Wall Following
Upload `wall\_following.ino`.
Place the robot near a wall.
Watch idle distance readings in Serial Monitor.
Send `g` or press D32 to start.
Send `w` to swap wall side if needed.
Send `s` or press D33 to stop.
---
14. Calibration and Testing Checklist
Before every scored run:
Check battery voltage.
Check Motoron power and I2C wiring.
Confirm left and right motor directions.
Confirm QTR sensor sees both white floor and black tape.
Run QTR calibration.
Check ultrasonic readings in Serial Monitor.
Test D32 start button.
Test D33 stop button / kill switch.
Test red and green LEDs.
Test RFID detection before navigation.
Test servo movement before planting.
Keep the robot lifted when first testing motor direction.
---
15. Tuning Guide
Line Following
Symptom	Suggested Fix
Robot oscillates too much	Lower `KP` or raise `KD`
Robot reacts too slowly	Raise `KP`
Robot loses line on sharp turns	Lower speed or increase turn limit
Robot stops during turns	Increase recovery timeout or adjust turn speed
Line detection too sensitive	Raise `BLACK\_SENSOR\_THRESHOLD`
Line detection not sensitive enough	Lower `BLACK\_SENSOR\_THRESHOLD`
Wall Following
Symptom	Suggested Fix
Drifts away from wall	Raise `KP`
Wiggles around the target distance	Lower `KP` or raise `KD`
Hits front wall	Raise `FRONT\_STOP\_CM` or increase `TURN\_MS`
Avoidance turn is too large	Lower `TURN\_MS`
Avoidance turn is too small	Raise `TURN\_MS`
Robot stalls	Raise `MIN\_DRIVE\_SPEED`
---
16. Known Notes
Encoders are reserved in the pin plan but are not used in these sketches.
The line-following programs use open-loop motor speed with QTR feedback, not encoder feedback.
The wall-following program uses ultrasonic distance feedback, not line sensors.
The RFID navigation program only uses RFID actions when the robot is in `STATE\_FOLLOW\_LINE`.
Repeated RFID UIDs are ignored to prevent the same node from triggering multiple actions.
Always use the latest `#define` pin values in the code if a comment and a definition disagree.
---
17. Related Diagrams
The report should include:
Software overview diagram
Main control loop flowchart
Line following behaviour flowchart
RFID navigation / planting flowchart
Emergency handling / kill switch flowchart
Wall following behaviour explanation
These diagrams explain how the main software components interact and how each key behaviour moves through its state machine.
---
18. Short Project Description for Report
This robot software controls a multi-sensor autonomous robot for line following, RFID navigation, wall following, and planting actions. The QTR line sensor array provides line position feedback for PD line tracking, while the Motoron I2C controllers drive the left and right motors. RFID tags are used as grid nodes for route decisions, allowing the robot to choose straight, left, right, or stop actions. Ultrasonic sensors are used for wall following and front obstacle avoidance. A servo mechanism is used for the planting / bead release action. Safety is handled through a physical stop button, serial stop command, line-loss recovery, turn timeouts, and status LEDs.
