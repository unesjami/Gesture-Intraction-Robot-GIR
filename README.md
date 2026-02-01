# Gesture Interaction Robot (GIR)

🤖 Gesture-Controlled Humanoid Robot  
An interactive humanoid robot controlled by hand gestures, sound, and sensors, combining computer vision, embedded systems, and human–robot interaction.  
This project uses Python + OpenCV + MediaPipe for real-time gesture recognition and an Arduino for smooth servo control, audio feedback, and autonomous behaviors.  
  
🔹 Project Overview  
The system detects human hand gestures using a webcam and translates them into high-level commands.  
These commands are sent to an Arduino, which controls the robot’s hands, head, body, eyes, and audio responses.  
The robot can also react independently using onboard sensors such as an IR sensor and a microphone.  
  
🧠 Features  
✋ Real-time hand gesture recognition (left & right hand detection)  
🧮 Finger counting–based control logic  
🦾 Smooth, human-like servo movements using easing functions  
🎧 Voice & sound playback via DFPlayer Mini  
👀 Eye control with IR sensor + EEPROM memory  
👏 Double-clap detection to trigger an introduction animation  
🤖 Head, body, and arm control through intuitive gestures  
🔄 Failsafe behavior when no hand is detected  
  
🛠 Technologies Used  
Software: Python, OpenCV, MediaPipe, PySerial.
Hardware: Arduino Uno R3, Servo Motors (Hands, Head, Body), DFPlayer Mini (Audio module), IR Sensor (Eye control), Microphone Sensor (Clap detection), EEPROM (State memory)
  
🎮 Gesture Controls  
Gesture  Action  
4 fingers (Right Hand)  Raise / Lower right hand  
4 fingers (Left Hand)  Raise / Lower left hand  
Index finger movement  Control robot head direction  
Thumb + Index  Rotate robot body  
Double clap  Play introduction + animation  
IR hand near sensor  Toggle robot eyes ON/OFF  
  
🧩 System Architecture  
Webcam captures live video  
Python processes frames using MediaPipe  
Gestures are converted to text commands  
Commands are sent via Serial (Bluetooth / USB)  
Arduino executes motion, sound, and sensor logic  
