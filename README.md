In this project, control strategies used for multirobot search and rescue operations are discussed. The control strategies are designed for three objectives. First, robots had to rendezvous and we needed to ensure that all agents can communicate with another while avoiding collisions. Upon rendezvous, the agents would switch their objective to enter the search area while avoiding Obstacles. Once at the facility, the agents would switch to exploration mode to search for survivors. The simulation and experimental results are provided to verify the control strategies.

So, the accomplishment of three tasks was needed for this mission: Initial deployment, Navigation to the cluttered environment, and finally, search and rescue.

INITIAL DEPLOYMENT: In this part, the robots had to rendezvous to ensure that the agents can communicate with one another while avoiding collision using minimum safety distance, and maximum interaction distance. The interaction between agents was modeled using a $\Delta$-disk graph. We used edge tension function to achieve the objective of this part.

NAVIGATING THE CLUTTERED ENVIRONMENT: The objective of this part is for agents to enter the search area while avoiding obstacles. They need to switch to a leader-follower formation mode to accomplish this mission.

SEARCH AND RESCUE: In this part, robots need to switch to exploration mode to search for survivors. It is needed for the agents to avoid collisions again. For collision avoidance, the same strategy as the initialization part was used. So, we used the same edge tension function with hysteresis to account for the collision. For covering the area and to roughly specify where the agents should be located in space, a continuous-time coverage algorithm was used. For doing so, we used the following time-varying density function and Lloyd’s algorithm.

The proposed algorithms were implemented in MATLAB first for simulation and then on the Robotarium platform provided by Georgia Tech. The mission was successfully accomplished in both simulation and experiment. The experimental results on the robots verify the performance of the proposed method in the real world. The result is attached as a video file.

_________________________________________________

# NEW VERSION

The simulator has been updated to work with the brand new GRITSBot X!  Gains accross the utilities have changes.  **Please check out the examples to update your own code.**

# MATLAB Version 

Currently, we know that the simulator works with MATLAB 2014b and higher.  The backwards compatibility issues are mostly due to changes in the way MATLAB handles figures in newer releases.

# Required Toolboxes 

We make heavy use of MATLAB's optimization toolbox function 'quadprog.'  Though this toolbox isn't necessarily required, all of our barrier function algorithms utilize this function.

# robotarium-matlab-simulator
MATLAB simulator for the Robotarium!  The purpose of the Robotarium simulator is to ensure that algorithms perform reasonably well before deployment onto the Robotarium.  Note that scripts created for the simulator can be directly deployed onto the Robotarium.  To ensure minimum modification after deployment, the simulator has been created to closely approximate the actual behavior of the Robotarium's agents. 

# Usage 

First, take a look at the "examples" folder for a few, simple examples.  Note that, to run these examples, you must first run the "init.m" script to add the requisite directories.  

# Documentation 

For example mathematical documentation, FAQs, and more, visit http://www.robotarium.org.
