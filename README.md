# Husarion Rosbot Autonomous Navigation

This project implements Q-Learning-based autonomous navigation for the Husarion Rosbot in a simulated environment. Through reinforcement learning, the robot learns to navigate efficiently, avoid obstacles, and refine its trajectory based on real-time sensor data.

Developed in collaboration with Nahuel Román, whose expertise and contributions were fundamental to this project.

🔗 Original repository: [Nahuel Román's University Portfolio](https://github.com/Nahuel7978/university_portfolio.git) (see "Q-Learning" section).

## Installation & Usage  
1. Open the project in **Webots**.  
2. Locate the **controller file** and ensure Webots is set to use it.  
3. Run the simulation to observe the robot’s learning-based navigation.  
4. You can manually adjust the **robot's initial position** and other parameters to test different scenarios.  

### Features
- Q-Learning for navigation – The robot learns optimal movement strategies through trial and error.
- Lidar-based obstacle detection – Uses laser scanning to perceive and avoid obstacles.
- Trajectory mapping – Compares the estimated path with the actual movement.
- Supervisor module – Provides real-time position tracking and evaluation.
