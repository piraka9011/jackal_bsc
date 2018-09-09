# Jackal Blended Shared Control
A blended shared controller for the Clearpath Jackal robot.

Contains files that can launch the BSC seperately or with simulated drift and latency.

### Launch files

Jackal Simulator stack: `roslaunch jackal_bsc jackal_sim.launch`

Odometry Navigation with Kinect: `roslaunch jackal_bsc odom_navigation.launch`

AMCL Navigation wih Kinect: `roslaunch jackal_bsc amcl_navigation.launch`

Standalone BSC: `roslaunch jackal_bsc bsc.launch`

BSC with drift: `roslaunch jackal_bsc drift.launch`

BSC with latency: `roslaunch jackal_bsc delay.launch`

### Classes and nodes
`BSCSolver` class is where the blending occurs. `BSCTwistStamped` adds time to twist msgs to make sure navigation/teleop/BSC commands are synced.

Use `delay_node.py` not the C++ version for delay (Strangely compilation varies from PC to PC, code is correct though). `drift_node.cpp` can be used.
