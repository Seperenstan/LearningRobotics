While studying kinematics, I've encountered various robot structures or important concepts.
This is some of what I've learned so far:
## 1. Robot Structures:

1.1 **Joint types** (Commonly used ones, there are more specialized joints as well):
   - **Revolute Joint**: allows rotation around a fixed axis (1 DOF)
   - **Prismatic Joint**: allows linear translation along an axis (1 DOF)
   - **Spherical Joint**: allows rotation around multiple axes (3 DOF)

When multiple joints & links are combined we can describe their graph topology:

1.2 **Topologies**:
   - **Serial**: joints and links are connected in a single chain (e.g. robotic arms)
   - **Branched**: chains diverge/branch from a common node (e.g. fingers in a hand)
   - **Parallel**: graph forms at least one closed loop (e.g. Stewart platform)

We can also categorize whether a robot is fixed to the world frame or mobile:

1.3 **Mobility**:
   - **Fixed-base Robots**: base is stationary (e.g. industrial robotic arms)
   - **Floating-base Robots**: base can move freely (e.g. humanoid robots)
   - **Mobile Robots**: Base can move only through a 2D plane in the environment (e.g. wheeled robots)

## 2. Configurations:

2.1 **Degrees of Freedom** (DOF):

The DOF of a robot describes the number of independent parameters that define its configuration. To describe floating & mobile robots, we often introduce the concept of a virtual link that connects the robot to the world frame (This adds e.g. 6 DOF for floating-base robots).

2.2 **Configurations Space**:

Since joint mobility is usually limited (e.g. a revolute joint can only rotate between -180° and +180°), the set of all possible configurations of a robot is called its configuration space. Each point in this space corresponds to a unique arrangement of the robot's joints and links, and is defined be the cartesian product of all individual joint parameter spaces (for serial & branched robots). 

For parallel mechanisms, the configuration space is more complex due to the closed-loop constraints imposed by the parallel structure.

To be continued...
