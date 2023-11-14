# Self-balancing robot
This is a project to develop a self-balancing robot using an arduino to control it.

## Process
To start with I created a simple model (two wheels and a box as the main part) and learnt how to control the robot using the 'differential drive' plugin and how to get the orientation data from the imu (using the imu plugin). Next I used a PID controller to control the robot in the Gazebo simulation and tuned the three constants (proportional, derivative, and integral) so that it balanced. After this, I wrote the Q-Learning algorithm that is used so that the Q-Table values can be used to control the physical robot. I used Autodesk Inventor to create an accurate model and this is also how I obtained the inertial matrix for the robot.

## Q-Learning algorithm

First, a table of random values (Q-Values ranging from 0 to 1) is initialised (the Q-Table). The Q-Values denote how favourable each action is in the current state and are modified based on how the action taken affects the observed state. 
The agent (robot in this case) gets an observation from its environment (pitch angle and chassis velocity). This observation is then discretised (placed into buckets) and the action to be taken is decided based on a policy (how the agent should act). My policy is that a random action is taken if a random number is less than the value of the variable 'epsilon' otherwise the best possible action is taken (the action with the highest Q-Value). Many iterations (episodes) are completed so that the Q-Values become the best that they can be.

### Q-Value update formula
Q<sub>new</sub>(s,a) = Q<sub>old</sub>(s,a) + learning_rate * (reward + discount_rate * maximum_expected_reward - Q<sub>old</sub>)
