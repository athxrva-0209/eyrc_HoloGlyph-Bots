# Hologlyph Bots (e-yrc 2023-24)

Designing and implementing holonomic drive based robot and its control system using ROS2 and Python. Learning about holonomic drive, inverse kinematics, control systems, CAD modelling, circuit designing and more.

## About The Project
In this project, we shall make glyphs on an 8ft x 8ft arena with an overhead camera to assist these 3 holonommic robots!

To enable the robot to do more complex and interesting glyphs, we shall explore an exciting type of mobile locomotion, known as holonomic drive. To draw glyphs of such large dimensions autonomously we will surely need more than just one robot. And thus we shall have a team of three holonomic drive robots for the job.

Through this project, we first need to understand the kinematics of a holonomic drive robot. Then we design simple controllers to make the bots perform desired motion along curves. We first test these concepts in simulation and then move on to experience the fun of implementing this in hardware.

This project involves understand of key concepts such as inverse kinematics, control systems, CAD modelling, ROS, Gazebo, circuit designing.

## Tech Stack
- ROS2 Humble
- Gazebo
- Python3
- OpenCV
- Autodesk Fusion 360 (for CAD modelling)
- ROS cam packages (for interaction with overhead camera)
- MicroROS (for integrating ROS with esp32)

## Demo
### Lissajous Curve
Lissajous curve, also called Bowditch Curve, pattern produced by the intersection of two sinusoidal curves the axes of which are at right angles to each other.

The specific Lissajous curve for used is :

**x=200cos(t)**\
**y=150sin(4t)**\
**t=[0,2pi]**


<img src="https://github.com/aPR0T0/Eklavya-Copter-Control/assets/158272880/3d50a0b8-ea84-48d0-a145-1fb357ab5300" alt="Function Mode" width="400" height="300">

### The Flower
**r = 220cos(4θ)**\
<img src="https://github.com/aPR0T0/Eklavya-Copter-Control/assets/158272880/b8169162-1b20-41ab-902a-c75f2d0bba20" alt="Function Mode" height="300">



### Output  
<img src="https://github.com/aPR0T0/Eklavya-Copter-Control/assets/158272880/20576277-c26b-437a-89b9-ff5c5a52188a" alt="Function Mode" height="400">

<img src="https://github.com/athxrva-0209/eyrc_HoloGlyph-Bots/assets/158272880/f46bbbbd-a07b-4800-bf72-c39a1db894b4" alt="Function Mode" height="400">

## Designed Bot
Material used for fabrication of this structure was 3mm thick transparent acrylic sheets which was laser cut and put together to form this arrangement 
![hb_task_3a_3230](https://github.com/athxrva-0209/eyrc_HoloGlyph-Bots/assets/158272880/0b20d09b-8e17-4424-82ea-117d4a639ddc)

### Basic info about bots
Components used for hologlyph bots:
- [USB camera](https://tinyurl.com/e-camera)
- [Esp32](http://tiny.cc/e-esp32)
- [Omni wheel (38mm)](https://tinyurl.com/e-omni-wheel)
- [MG995 360 continous servo](https://tinyurl.com/e-servo)
- [MG90S microservo](https://tinyurl.com/e-servo-180)
- [Orange 2200 mah LiPo Battery](https://tinyurl.com/58uftjb4)
- [Buck converter XL4015](https://tinyurl.com/hp4rmcey)

## Contributors
- [Atharva Wadnere](https://github.com/athxrva-0209)
- [Shrikar Dongre](https://github.com/shrikardongre)
- Amey Jawale
- Sharayu Mane


