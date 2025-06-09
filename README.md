## 🛫 Run the UAV MPC Trajectory Tracking Simulation

```bash
# 1️⃣ Launch the Gazebo simulation with the runway and UAV model
roslaunch iq_sim runway_mpc.launch

# 2️⃣ Start the ArduPilot SITL instance (e.g., quadcopter firmware)
./startsitl.sh

# 3️⃣ Launch MAVROS and connect it to SITL
roslaunch iq_sim apm.launch

# 4️⃣ Run the MPC tracking node (live trajectory tracking from cube in Gazebo)
rosrun iq_gnc uav_mpc_track_vis.py

# 5️⃣ Open RViz to visualize the UAV position and tracking trajectory
rosrun rviz rviz
