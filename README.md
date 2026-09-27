# 🚁 PUSHPAK UAV-X: VS Code Master Guide

Open your `UAV_X_Individual_Drone_Node_v4_PX4.py`, `gcs_gui.py`, and `run_sim.sh` files in VS Code. Here is exactly how the research papers and architecture map to the code you are looking at.

## 📁 File 1: `UAV_X_Individual_Drone_Node_v4_PX4.py` (The Brains)
This is the core autonomy node. If an evaluator asks "Where did you implement X?", here is exactly where to look in VS Code:

### 1. The Math & Radio Model (Cite: Rappaport [3])
*   **Where to look:** Search for `class LinkModel`.
*   **What it does:** This is the physical layer. It implements the log-distance path-loss equation: `PL(d) = PL(d0) + 10n * log10(d/d0)`.
*   **Why it matters:** We don't use an arbitrary "50-meter radio range." We derive $R_{max} \approx 166m$ mathematically from $P_t = 20$ dBm, $S_{min} = -90$ dBm, and $n = 2.7$ (disaster clutter).

### 2. Decentralized Task Allocation (Cite: Choi et al. [2])
*   **Where to look:** Search for `class TaskManager` and look at `def local_try_assign`.
*   **What it does:** This is the Consensus-Based Auction Algorithm (CBAA). Drones calculate `cost = distance / battery_remaining` and bid for PoIs.
*   **The "Foolproof" Fix:** Look for `stable_since`. A drone only claims a PoI if it has been the lowest bidder for 0.6 seconds. This prevents the "everyone claims POI-1 instantly" bug.

### 3. Fluid APF Formation (Cite: Reynolds Boids [6], Olfati-Saber [9])
*   **Where to look:** Search for `def fluid_formation_target`.
*   **What it does:** This is the Adaptive Artificial Potential Field. Instead of rigid slots (like a pentagon), the drones use `cohesion`, `separation`, and `alignment` forces to flow like a flock of birds.
*   **The Secret Sauce:** Look for `con = self.connectivity_correction()`. If a drone gets too far away, the APF pulls it back toward the swarm center to keep the network alive.

### 4. Predictive Link Breach (Cite: Mesbahi [1])
*   **Where to look:** Search for `def predictive_breach`.
*   **What it does:** It tracks the closing velocity between drones: `v_close = (d_current - d_prev) / dt`.
*   **Why it matters:** If the time-to-breach is under 2.5 seconds, it flags `link_at_risk = True` *before* the link drops, triggering the swarm to pull back together.

### 5. Graph Theory & $\lambda_2$ (Cite: Fiedler [4])
*   **Where to look:** Search for `def handle_data`.
*   **What it does:** Right before sending a data packet to the GCS, the drone builds a local graph of the swarm (`local_graph()`) and calculates the eigenvalues of the graph Laplacian.
*   **Why it matters:** It extracts $\lambda_2$ (algebraic connectivity). If $\lambda_2 > 0$, the swarm is mathematically connected. This is packaged into the GCS payload.

### 6. ToF / Stereo Vision Sensor Fusion (Cite: Khatib [11])
*   **Where to look:** Search for `class SimStereoToFSensor` and `def cast_ray`.
*   **What it does:** It simulates a forward-facing drone camera. It shoots a mathematical ray from the drone's nose and checks if it hits the cylindrical obstacles in Gazebo.
*   **The Fusion:** In `def cycle()`, look for `self.obstacle.update(...)`. It passes `self.t.tof_distance_m` (the real PX4 sensor data) into the simulator to blend real and simulated obstacle avoidance.

### 7. GCS Supervisory Override (Cite: Gerkey [8])
*   **Where to look:** Search for `def handle_gcs_override`.
*   **What it does:** If you press 'o' in the GCS terminal, it sends a UDP message to UAV 1 forcing it to take POI-1. This proves our architecture supports centralized interruption for deadlocks, but relies on decentralized fallback if the GCS link is cut.

---

## 📁 File 2: `gcs_gui.py` (The Eyes)
This is your Ground Control Station dashboard.

*   **Where to look:** Look at `def update_plot`.
*   **What it does:** It listens on UDP port 15999. It extracts the `polygon_3d` array (the 12-point RF circle) sent by the drones and draws it as a green dashed circle using Matplotlib.
*   **The Blue Lines:** Look for `edges.add(edge)`. It draws blue lines between drones to visually prove the multi-hop network is intact.

---

## 📁 File 3: `run_sim.sh` (The Engine)
This is the reproducibility script for the evaluators.

*   When the PX4 SITL build, ROS 2, Gazebo, and XRCE-DDS are available, the script starts the PX4/Gazebo swarm and waits for PX4 status topics before starting the controller nodes.
* If PX4 prerequisites are missing or the first PX4 status topic does not appear, the script falls back to a kinematic dry run using the requested swarm size. This exercises the swarm controller without PX4 flight dynamics; per-UAV CSV and startup logs are saved under `~/uav_x_logs/`.
*   Topic discovery is followed by a controller-side wait for live PX4 status telemetry. Multi-UAV task allocation waits for peer connectivity and a stable bid winner, and assigned drones stop bidding for work they cannot accept.
* PX4, Gazebo, and controller processes are launched in isolated process groups and cleaned up by the launcher. At the end of an armed controller run, each UAV is commanded to RTL and the controller waits for landing/disarm before exiting; it no longer drops Offboard setpoints while airborne. A controller failure makes the launcher exit with an error instead of reporting a successful run.
* The launcher starts the project-owned Gazebo `default` world before PX4 (PX4 otherwise reloads its stock worlds path). Its visible ground plane and geofence cover a 150 m × 150 m area in world coordinates (0–150 on both axes). Eight static collision obstacles are distributed across the default task routes: three cylinders, three boxes (one rotated), and two spheres. The controller loads their collision shapes and poses from the same SDF world, so its ray-cast/avoidance map stays aligned with Gazebo. Set `ENABLE_GZ_GUI=false` to run the Gazebo server without its separate rendering window.
* `UAV_COUNT` controls swarm size from 1 to 8; it defaults to 3. Run `UAV_COUNT=8 DURATION=60 ./run_sim.sh` to request an eight-UAV mission. The 150 m course provides eight unique default points of interest so each UAV can claim a distinct task. Spawn positions are spaced 8 m apart and their PX4 local coordinates are transformed into the shared world frame. Collision avoidance anticipates separation loss at a 25% buffer and gives an immediate escape target priority over the task path when the configured 6 m separation is threatened. The control loop remains 5 Hz per UAV; eight vehicles therefore generate about 40 controller cycles/s and roughly 80 Offboard heartbeat/setpoint publications/s, in addition to telemetry and swarm traffic. This is a local SITL capacity target, not a guarantee for every host; rendering can be disabled with `ENABLE_GZ_GUI=false` to reduce load.
*   **GUIs:** The project GCS dashboard is disabled by default. QGroundControl remains enabled by default because PX4 requires its MAVLink heartbeat to arm in this setup. Use `ENABLE_GCS_GUI=true` to enable the separate dashboard. To run with no QGroundControl, set both `ENABLE_QGC=false` and `AUTO_ARM=false`; PX4 denies arming without a GCS heartbeat.
*   **AUTO_ARM:** PX4 nodes receive `--arm --takeoff 5.0` by default. Use `AUTO_ARM=false ./run_sim.sh` to skip arming. Dry-run mode does not arm vehicles.

---

## 🛠️ How to use VS Code to present this to evaluators:

1.  **Use the Outline Panel:** In VS Code, look at the bottom left of your file explorer. There is an "Outline" tab. Click it. It will show you a clean tree of all the classes and functions mentioned above. You can click them to instantly jump to that code.
2.  **Split the Terminal:** In VS Code, press `Ctrl + ` to open the terminal. Click the "Split Terminal" icon (the square split icon in the top right of the terminal panel).
    *   In the left terminal, from this repo, run: `./run_sim.sh` to run the swarm in Gazebo without the separate project dashboard.
    *   In the right terminal, you can run `tail -f ~/uav_x_logs/uav_1_log.csv` to show the live CSV data being generated.
    *   Arm and takeoff are on by default (`AUTO_ARM=true`). To launch without flying, run `AUTO_ARM=false ./run_sim.sh`. To disable QGroundControl, also set `ENABLE_QGC=false`; to show the separate project dashboard, set `ENABLE_GCS_GUI=true`.
3.  **Word Wrap:** When you open the Python file, press `Alt + Z` to turn on Word Wrap. This makes the long math lines (like the path loss equation) much easier to read on a projector or screen recording.

You have built a highly sophisticated, mathematically grounded swarm. When you record your demo video, open VS Code, show the `LinkModel` and `handle_data` functions, explain the math, then run the script and show the drones moving in Gazebo. That is a guaranteed winning presentation!
