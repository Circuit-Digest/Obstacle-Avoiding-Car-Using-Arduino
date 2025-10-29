# Arduino Obstacle Avoiding Car

![Arduino Obstacle Avoiding Car](https://circuitdigest.com/sites/default/files/projectimage_mic/DIY-Obstacle-Avoiding-Car.png)

## Overview

The [Arduino Obstacle Avoiding Car](https://circuitdigest.com/microcontroller-projects/arduino-obstacle-avoiding-car) is an autonomous robot that can navigate its environment by detecting and avoiding obstacles in real-time. This intelligent vehicle uses a servo-mounted ultrasonic sensor to scan for objects in front, left, and right directions, allowing it to determine the best path forward without colliding with obstacles.

**Project Tutorial:** [Arduino Obstacle Avoiding Car - Circuit Digest](https://circuitdigest.com/microcontroller-projects/arduino-obstacle-avoiding-car)

## Features

- **Autonomous Navigation:** Real-time obstacle detection and avoidance
- **360° Scanning:** Servo motor rotates ultrasonic sensor to check left, right, and front directions
- **Visual Indicators:** RGB LEDs display movement status (green for forward, red for stop/reverse)
- **Motor Control:** Four-wheel drive with independent speed control using L298N motor driver
- **Smart Decision Making:** Calculates optimal path based on available space

## Components Required

| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Uno | 1 | Main microcontroller |
|[HC-SR04 Ultrasonic Sensor](https://circuitdigest.com/microcontroller-projects/interfacing-ultrasonic-sensor-hc-sr04-with-pic16f877a) | 1 | Distance measurement (40kHz ultrasonic waves) |
| MG90 Metal Gear Servo Motor | 1 | Sensor rotation for scanning |
| L298N Motor Driver | 1 | DC motor control |
| RGB [LEDs](https://circuitdigest.com/microcontroller-projects/interfacing-neopixel-led-strip-with-arduino) | 3 | Visual movement indicators |
| 2WD Arduino Car Kit | 1 | Laser cut chassis with 4 geared motors |
| 12V Li-ion Battery | 1 | Power source |
| LM2596 DC-DC Buck Converter | 1 | Voltage regulation (12V to 5V) |
| Power Switch | 1 | On/Off control |
| Breadboard/PCB | 1 | Circuit assembly |
| Jumper Wires | As needed | Connections |
| Screws, Nuts, Spacers | As needed | Assembly hardware |

## Pin Configuration

### Ultrasonic Sensor (HC-SR04)
- **VCC** → 5V (from LM2596)
- **GND** → GND
- **TRIG** → Digital Pin 2
- **ECHO** → Digital Pin 4

### Servo Motor (MG90)
- **Signal** → PWM Pin 5
- **VCC** → 5V (from LM2596)
- **GND** → GND

### RGB LEDs
- **Right LED:**
  - Green → Pin 9
  - Red → A3
  - GND → Common Ground
  
- **Center LED:**
  - Green → Pin 13
  - Red → A2
  - GND → Common Ground
  
- **Left LED:**
  - Green → Pin 10
  - Red → A1
  - GND → Common Ground

### L298N Motor Driver
- **IN1** → Pin 8 (Left Motor Control)
- **IN2** → Pin 7 (Left Motor Control)
- **IN3** → Pin 12 (Right Motor Control)
- **IN4** → Pin 11 (Right Motor Control)
- **ENA** → PWM Pin 6 (Left Motor Speed)
- **ENB** → PWM Pin 3 (Right Motor Speed)
- **12V Input** → 12V Battery Positive
- **GND** → Battery Negative

### Power Supply
- **12V Li-ion Battery** → LM2596 Input & L298N 12V Input
- **LM2596 5V Output** → Arduino 5V & Sensor/Servo Power
- **Common Ground** → All components

## Circuit Diagram

The complete circuit connects the ultrasonic sensor for obstacle detection, servo motor for sensor rotation, L298N motor driver for wheel control, and RGB LEDs for status indication. The LM2596 buck converter steps down 12V battery power to 5V for the Arduino, sensors, and servo motor.

## How It Works

### Operating Principle

1. **Power On:** When powered, the Arduino initializes all components
2. **Scanning Phase:** The servo motor rotates the ultrasonic sensor to scan three directions:
   - Left (180°)
   - Front (90°)
   - Right (0°)
3. **Distance Measurement:** The HC-SR04 sensor emits 40kHz ultrasonic pulses and calculates distance based on echo return time
4. **Decision Making:** 
   - If front distance ≥ threshold (12cm): Move forward
   - If obstacle detected: Stop and scan all directions
   - Choose direction with most clearance
5. **Navigation:** Execute turn (left/right) or continue forward
6. **Visual Feedback:**
   - **All Green:** Moving forward
   - **All Red:** Stopped
   - **Single Green:** Turning (green LED indicates turn direction)

### Distance Calculation

The ultrasonic sensor calculates distance using the formula:
```
Distance (cm) = (Pulse Duration / 29) / 2
```
Where:
- Pulse duration is measured in microseconds
- Speed of sound: 340 m/s
- Division by 29 converts to cm, division by 2 accounts for round trip

### Obstacle Detection Logic

```
If (Front Distance < 12cm):
    Stop()
    Scan Left, Front, Right
    
    If (Both Left & Right ≤ 25cm):
        Move Forward (force through)
    Else If (Left Distance > Right Distance):
        Turn Left
    Else:
        Turn Right
```

## Code Structure

### Main Functions

#### `setup()`
- Initializes all pin modes (INPUT/OUTPUT)
- Attaches servo motor to Pin 5
- Sets initial motor speeds via PWM
- Initializes serial communication (9600 baud) for debugging
- Sets all LEDs to OFF state

#### `loop()`
- Centers servo to look forward (90°)
- Measures front distance
- Moves forward if path is clear
- Stops when obstacle detected
- Calls `checkDirection()` to find best path
- Executes appropriate turn or forward movement

#### `getDistance()`
- Sends 10μs trigger pulse to HC-SR04
- Measures echo pulse duration
- Calculates and returns distance in cm
- Prints distance to serial monitor for debugging

#### `checkDirection()`
- Rotates servo to scan left (180°), front (90°), and right (0°)
- Stores distances in array: `[left, right, front]`
- Returns optimal turn direction:
  - `0` = Turn Left
  - `2` = Turn Right
  - `3` = Move Forward

#### Motor Control Functions

**`moveForward()`**
- Sets motor speeds (ENA: 60.5, ENB: 52)
- Turns all LEDs green
- Both motors rotate forward

**`turnLeft()`**
- Increases motor speed (100, 90) for sharper turn
- Left LED green, center & right red
- Left motor backward, right motor forward

**`turnRight()`**
- Increases motor speed (100, 90)
- Right LED green, center & left red
- Left motor forward, right motor backward

**`Stop()`**
- All LEDs turn red
- Brief reverse pulse for immediate stop
- Sets all motor speeds to 0

## Installation & Setup

### 1. Hardware Assembly

1. **Mount Components:**
   - Secure Arduino Uno to chassis using spacers
   - Mount L298N motor driver
   - Install ultrasonic sensor on servo motor
   - Position servo at front of chassis
   - Attach RGB LEDs to underside

2. **Wire Connections:**
   - Follow pin configuration table above
   - Use jumper wires or solder to PCB for permanent connections
   - Ensure proper polarity for motors and LEDs

3. **Power System:**
   - Connect 12V battery to LM2596 input
   - Wire LM2596 5V output to Arduino and sensors
   - Install power switch in battery positive line
   - Add optional battery level indicator and charging port

4. **Final Checks:**
   - Verify all connections against circuit diagram
   - Check for loose wires or potential short circuits
   - Ensure motors are securely mounted
   - Test servo rotation range

### 2. Software Installation

1. **Install Arduino IDE:**
   - Download from [arduino.cc](https://www.arduino.cc/en/software)
   - Install appropriate version for your OS

2. **Install Required Library:**
   ```
   Servo.h (usually pre-installed with Arduino IDE)
   ```

3. **Upload Code:**
   - Open Arduino IDE
   - Copy the complete code (provided in repository)
   - Select **Board:** Arduino Uno
   - Select **Port:** Correct COM port
   - **IMPORTANT:** Turn OFF power switch before uploading
   - Click Upload button
   - Wait for "Done uploading" message

### 3. Testing & Calibration

1. **Motor Speed Calibration:**
   - Adjust `ENA` and `ENB` values in code for balanced movement
   - Default values: ENA=53.5, ENB=45 (idle), ENA=60.5, ENB=52 (forward)
   
2. **Distance Threshold:**
   - Modify `MIN_DISTANCE_BACK` (default: 12cm) based on your requirements
   - Larger value = more cautious navigation
   
3. **Turn Timing:**
   - Adjust delay values in `turnLeft()` and `turnRight()` functions
   - Default: 425ms (left), 415ms (right)
   - Test on different surfaces for optimal turns

## Code Customization

### Adjustable Parameters

```cpp
// Distance threshold (line 21)
#define MIN_DISTANCE_BACK 12  // Change based on obstacle size

// Motor speeds (lines 57-58 for idle)
analogWrite(ENA, 53.5);  // Left motor idle speed
analogWrite(ENB, 45);    // Right motor idle speed

// Forward speeds (lines 155-156)
analogWrite(ENA, 60.5);  // Left motor forward speed
analogWrite(ENB, 52);    // Right motor forward speed

// Turn speeds (lines 171, 185)
analogWrite(ENA, 100);   // Turn speed
analogWrite(ENB, 90);

// Turn duration (lines 99, 105)
delay(425);  // Left turn duration in milliseconds
delay(415);  // Right turn duration in milliseconds
```

## Enhancements

This project includes several upgrades beyond basic functionality:

1. **Power Management:**
   - Power on/off switch
   - 12V DC charging socket
   - Li-ion battery level indicator
   - Slider switch for battery monitoring when power is off

2. **Visual Indicators:**
   - RGB LEDs show real-time movement status
   - Color-coded feedback for easy monitoring

3. **Modular Design:**
   - Easy component replacement
   - Expandable for additional sensors
   - Clean cable management

## Troubleshooting

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Motors not running | Power issue | Check battery charge and connections |
| Car only turns one direction | Motor speed imbalance | Calibrate ENA/ENB values |
| Sensor not detecting | Wrong wiring | Verify TRIG/ECHO pin connections |
| Servo not moving | Power or signal issue | Check 5V supply and Pin 5 connection |
| LEDs not lighting | Incorrect polarity | Verify anode/cathode connections |
| Car hits obstacles | Distance threshold too low | Increase MIN_DISTANCE_BACK value |
| Erratic movement | Loose connections | Secure all wire connections |

## Applications

This obstacle-avoiding car demonstrates fundamental autonomous robotics principles and can be adapted for:

- **Educational Purposes:** Teaching robotics, sensors, and programming
- **Warehouse Automation:** Goods transport in confined spaces
- **Food Delivery Robots:** Autonomous indoor navigation
- **Telemetric Robots:** Remote exploration and monitoring
- **Maze Solving:** Algorithm development and testing
- **Home Automation:** Mobile sensor platforms

## Future Improvements

- Add Bluetooth/WiFi control for manual override
- Implement line-following capability
- Add camera for computer vision
- Include GPS for outdoor navigation
- Integrate multiple ultrasonic sensors for better coverage
- Add pick-and-place mechanism
- Implement maze-mapping algorithms

## Safety Notes

⚠️ **Important Safety Guidelines:**

- Always disconnect power when uploading code or modifying hardware
- Ensure proper battery polarity to avoid damage
- Use appropriate voltage regulators (LM2596)
- Monitor battery temperature during charging
- Keep electronics away from water
- Test in controlled environment before autonomous operation
- Add bumpers for additional protection

## License

This project is open-source and available for educational and personal use. Please credit Circuit Digest when sharing or modifying this project.

## Credits

- **Original Tutorial:** [Circuit Digest](https://circuitdigest.com/microcontroller-projects/arduino-obstacle-avoiding-car)
- **Platform:** Arduino
- **Sensors:** HC-SR04 Ultrasonic Sensor
- **Motor Control:** L298N Driver Module

## Additional Resources

- [Arduino Robotics Projects](https://circuitdigest.com/arduino-robotics-projects)
- [HC-SR04 Sensor Guide](https://circuitdigest.com/microcontroller-projects/interfacing-hc-sr04-ultrasonic-sensor-with-arduino)
- [L298N Motor Driver Tutorial](https://circuitdigest.com/microcontroller-projects/interfacing-l298n-motor-driver-with-arduino)
- [Circuit Digest Robotics](https://circuitdigest.com/robotics-projects)
- [Arduino Projects](https://circuitdigest.com/arduino-projects)

---

**Project Source:** https://circuitdigest.com/microcontroller-projects/arduino-obstacle-avoiding-car

*Last Updated: October 2025*
