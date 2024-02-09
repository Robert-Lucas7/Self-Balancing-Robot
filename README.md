# Self-balancing robot
This is a project to develop a self-balancing robot using an arduino to control it.

## Process
To start with I created a simple model (two wheels and a box as the main part) and learnt how to control the robot using the 'differential drive' plugin and how to get the orientation data from the imu (using the imu plugin). I used Autodesk Inventor to create an accurate model and this is also how I obtained the inertial matrix for the robot (to get a more accurate simulation). Next I used a PID controller to control the robot in the Gazebo simulation and tuned the three constants (proportional, derivative, and integral) so that it balanced. Finally, I used an arduino and two DC motors to make the physical robot and implemented the code for it to balance.
