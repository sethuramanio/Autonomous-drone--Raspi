# Autonomous-drone with a robotic ARM--Raspberry pi 

Open source Contribution- This repository consists of the code that we developed for the making an autonomous drone with a robotic arm and controlling the same using a raspberry pi 3.

This was developed with a sole intention for mamking an open source contribution to original drone kit api which does not have indoor navigation without guided no gps mode and controlling everything with raspi - This is the Worlds first autonomous with a robotic arm on the top and a novel alogorithms to ensure stablity. 

1. arm.py contains all the codes with relevance to the 6DOF robotic arm 
2. fly.py contains the code where only the drone would move autonomously without the arm being functional
3. fly_pls.py contains all the codes where the drone can navigate according to the GPS coordinates with GPS fix
4. it_flew.py contains all the codes where the drone can navigate with local coordinates 
5. newFlight.py contains all the codes which integrate the end to end system for indoor/outdoor navigation with high resolution and robotic arm control. 

This code used deep RL algorithm for navigation and uses computer vision techniques for indentifying the target location


