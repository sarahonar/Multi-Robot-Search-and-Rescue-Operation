# Multi-Robot-Search-and-Rescue-Operation

In this project, control strategies used for multi-robot search and rescue operations are discussed. The control strategies are designed for three objectives. First, we do rendezvous and ensure that all agents can communicate with another while avoiding collisions. Upon rendezvous, the agents would switch their objective to enter the search area while avoiding Obstacles. Once at the facility, the agents would switch to exploration mode to search for survivors. The simulation and experimental results are provided to verify the control strategies.
The accomplishment of three tasks was needed for this mission: Initial deployment, Navigation to the cluttered environment, and finally, search and rescue.

INITIAL DEPLOYMENT: The robots had to rendezvous to ensure thatthe agents can communicate with one another while avoidingcollision using minimum safety distance, and maximum interaction distance. The interaction between agents was modeled using a $\Delta$-disk graph.

NAVIGATING THE CLUTTERED ENVIRONMENT: The objective of this part is for agents to enter the search area while avoiding obstacles. They need to switch to a leader-follower formation mode to accomplish this mission.

SEARCH AND RESCUE: In this part, robots need to switch to exploration mode to search for survivors. It is needed for the agents to avoid collisions again. For collision avoidance, the same strategy as the initialization part was used. So, we used the same edge tension function with hysteresis to account for the collision.
For covering the area and to roughly specify where the agents should be located in space, a continuous-time coverage algorithm was used. For doing so, we used the following timevarying density function and Lloyd’s algorithm.
