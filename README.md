# Internship-Project-Motion-Tracking
Motion tracking system using Pelco-D protocol over RS485 with image processing on Raspberry Pi, motor control via STM32 and distance measurement with Arduino Mega. Developed as part of an internship to explore multi-device communication with Raspberry Pi, STM32, and Arduino.

# Internship-task-codes
This repository includes:
- **STM32F103RB Master code**:
  sends commands to the slaves to control parameters such as angle, RPM, and step counts. Also requests and receives data from slaves regarding distance measurements, remaining steps, current angle position, RPM,     and the detected moving object's X and Y coordinates. This two-way communication enables precise coordination and real-time feedback within the motion tracking system.
- **STM32F411RE Slave code** (including step motor library Stepper from deep blue embedded):
  was developed to drive two servo motors and a stepper motor, demonstrating successful UART communication over RS485. This code handles incoming Pelco-D commands, translating them into precise motor movements to     enable motion tracking as part of the overall system.
- **Raspberry pi 5 slave code**:
  was developed to demonstrate robust master-slave communication on a real time motion detection and tracking image-processing application.
  Images and video frames are captured in real-time using the Basler camera connected to the Raspberry Pi and pypylon library.
  The captured data is processed with **OpenCV** in Python to perform motion detection and tracking.
  This setup enables precise and efficient image analysis necessary for controlling the motors and overall system response.
- **Arduino slave code** (The Arduino-based distance measurement component was implemented by a fellow intern team member and is included here for completeness.)

Developed during my internship at MS Spektral, July 2025.

# Acknowledgments:
Special Thanks to [deepbluembedded] for (https://deepbluembedded.com/stm32-stepper-motor-control-library-unipolar-28byj-48-uln2003/ ,and https://github.com/Khaled-Magdy-DeepBlue/STM32_Course_DeepBlue/tree/master/ECUAL/STEPPER) for their reference control of step motor.


