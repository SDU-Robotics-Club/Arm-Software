# Robotic Arm: Kinematics & Simulation Environment

## Overview
This program is a complete inverse kinematics (IK) solver, 3D simulator, and hardware bridge for a custom 5-DOF robotic arm. It allows users to input target XYZ coordinates and orientations of the wrist, calculates the required joint angles using spatial mathematics, visualizes the movement in real-time using PyBullet, and transmits the data to physical hardware via serial communication.

---

## User Guide

### 1. Starting the program
Run `main.py` from your terminal. 
The script will automatically attempt to connect to the physical robot via the ESP32 (COM3). If the hardware is not connected, it will fall back to **Simulation-Only Mode**.

### 2. Terminal input
Once the simulation window opens, the terminal will prompt you for a target pose.
* **To Home the Robot:** Simply press `ENTER` without typing anything.
* **To Move to a Target:** Input the coordinates in centimeters (X, Y, Z) and the orientation in degrees (Roll, Pitch). 
* *Note on Yaw:* Because this is a 5-DOF arm, the tool's Yaw is calculated based on the X/Y coordinates to prevent the solver from fighting physical limitations.

### 3. The PyBullet visualizer
The PyBullet window provides a real-time digital twin of the arm.
* **Camera Controls:** Hold `CTRL` + Left/Middle Mouse Button to rotate and pan the camera. Scroll to zoom.
* **Telemetry:** The simulation dynamically draws red (X), green (Y), and blue (Z) telemetry lines tracking the exact distance from the global origin to the Tool Center Point (TCP).

### 4. Driving the manual
After an IK movement completes, the terminal will ask: `New position? (y/n):`
* Type **`y`** to enter a new XYZ coordinate.
* Type **`n`** to exit the terminal loop and unlock **Slider Mode**. 
Once in Slider Mode, you can manually control individual joints using the UI sliders in the PyBullet window. 

### 5. Escape the arm
* Go to the terminal and press `n` to stop the input loop and `Ctrl+C` to exit completely.
If you end a previous simulation session by clicking 'X' on the PyBullet window, or by clicking the Trash Can icon 
in the VS Code terminal, Python dies immediately, but the PyBullet C++ engine does not.
It becomes an invisible "Ghost Process" running in the background of 
your computer, still holding onto your graphics card and memory. It will also haunt your RAM and sell your privacy data to China.

---

## Tips

* **Start Close to Home:** If the arm is twisted in a strange configuration, asking it to jump to the opposite side of its workspace might cause the solver to get trapped in a "local minimum." Send it to the Home position first, then to your target.
* **Mind the Limits:** The arm has strict physical limits defined in its URDF. If you request an impossible coordinate, the terminal will print a `[DEBUG]` log explaining exactly which joint prevented the movement. If you dare, you can try editing the joint limits in the file yourself.
* **Understand the 5-DOF Constraint:** A 5-DOF arm cannot strafe sideways while maintaining a forward-facing tool. The base must rotate to face the target.

---

## System Architecture & Workflow

### The Data Flow
1. **`main.py` :** Initializes the robot model using the urdf file, hardware bridge for sending commands to the external MCU using UART, and the simulator. It manages the main loop, passing target coordinates to the math engine.

2. **`input_handler.py`:** Sanitizes user terminal input, converts units (cm to m, deg to rad), and generates two 4x4 Spatial Math transformation matrices (`SE3`). One for end-effector's translation through the 3D space and the other for rotation in the 3D space. It multiplies both matrices to get the final homogeneous transformation matrix that describes the target position and orientation of the end-effector.

3. **`kinematics.py`:** Uses the Robotics Toolbox library to calculate the Forward and, most importantly, Inverse Kinematics.

4. **`pb_kinsim.py`:** Reads the URDF, sets up the PyBullet physics engine, draws the UI elements, and interpolates the motion trajectory (`jtraj`).

5. **`hardware_bridge.py`:** Formats the solved angles into byte strings and streams them via serial to the external MCU.

### Architectural Choices & Design Decisions

* **The Levenberg-Marquardt (LM) Solver:** Chosen over Newton-Raphson or analytical solvers due to its hybrid nature. It handles singularities and custom CAD geometries much better without throwing matrix math errors.
* **Dynamic Yaw Calculation:** Hardcoding the target Yaw to 0° in a 5-DOF arm causes unsolvable IK conflicts. The script calculates `atan2(y, x)` so the tool orientation always naturally aligns with the base motor first. This eliminates any need for wrist's yaw to come up in the solution, as it is not possible to use it.
* **Position Control vs. Kinematic Teleportation:** The PyBullet simulation utilizes `p.POSITION_CONTROL` with a simulated motor force rather than instantly snapping joints to their targets (`p.resetJointState`). This introduces realistic momentum, gravity resistance, and visualizes the trajectory much nicer.
* **State Tracking in Serial:** The hardware bridge caches the last sent angles and filters out redundant data. It only transmits if a joint moves more than 0.5 degrees, preventing USB buffer overflow on the ESP32.