# AutonomousRobot
An autonomous robot designed and programmed to complete the Brutus (Ohio State's Mascot) Garden course using PID-controlled motion, sensor integration, and task sequencing.

- Completed full course in 1:56  
- Achieved all primary and secondary points  
- Placed Top 16 at Ohio State Engineering Showcase

# Key Features

- PID-controlled driving for precise movement using encoders
- Autonomous task sequencing (no human input after start)
- Color detection using CdS cell (red/blue light logic)
- Multi-actuator control (motors + servo arm)
- Sensor fusion (optical sensors + encoders + light sensor)
- Complex path planning with turns, arcs, and alignment routines

# Technologies & Hardware

- Language: C++
- Platform: FEH Proteus Robot Controller
- Libraries:
  - FEHMotor
  - FEHServo
  - FEHIO
  - FEHLCD
- Sensors:
  - Shaft encoders
  - CdS cell (light detection)
  - Optical line sensors
- Actuators:
  - Dual DC motors
  - Servo-controlled arm with a Servo motor 

# PID Motor Control

Implemented a custom PID controller to regulate motor velocity using encoder feedback.

- **Proportional (P):** Corrects immediate velocity error  
- **Integral (I):** Accumulates past error to eliminate drift  
- **Derivative (D):** Dampens oscillations  

This allowed for:
- Smooth acceleration (ramping)
- Accurate distance tracking
- Stable turning and arc motion

# Course Tasks

The robot autonomously completed:

1. Pressed start button after light detection
2. Rotated compost bin using servo arm
3. Retrieved and placed apple bucket on elevated platform
4. Opened and closed window via repeated arc alignment
5. Detected light color and pressed correct humidifier button
6. Flipped randomized lever using competition control system
7. Navigated to final position

All actions were completed within the 2-minute limit.

# Performance

- Completion Time: 1 minute 56 seconds
- Accuracy: 100% task completion
- Ranking: Top 16 (Ohio State Engineering Showcase)

# Challenges & Solutions

**Challenge:** Inconsistent motor speeds  
→ **Solution:** Implemented PID control with encoder feedback

**Challenge:** Precise turning and alignment  
→ **Solution:** Developed arc-turn and encoder-based turning functions

**Challenge:** Sensor noise in light detection  
→ **Solution:** Used threshold ranges for reliable color classification

**Challenge:** Time constraint (2 minutes)  
→ **Solution:** Optimized movement speeds and reduced unnecessary delays

# Code Structure

- `PID Control`: Velocity regulation for both motors
- `Drive Functions`: Forward, backward, turning, arc movement
- `Task Functions`: CompostBin, AppleBucket, Window, etc.
- `Sensor Functions`: Light detection, optosensor readings
- `Main ծր`: Executes full autonomous routine

<img width="856" height="548" alt="Screenshot 2026-03-30 143208" src="https://github.com/user-attachments/assets/21a99cac-0781-424e-bc62-b6b55af6b3a2" />
<img width="580" height="495" alt="Screenshot 2026-03-30 143319" src="https://github.com/user-attachments/assets/4ace7408-744f-4841-a95d-7716378fcc97" />
<img width="640" height="556" alt="Screenshot 2026-03-30 143310" src="https://github.com/user-attachments/assets/dd26419e-f09e-4981-a415-76c4a6ebe70b" />

