"""
PyBullet Kinematics Simulator Module
Handles the 3D visualization, physics engine, and UI sliders for the robotic arm.
"""

import pybullet as p
import pybullet_data # a folder with assets
import time
import math
import roboticstoolbox as rtb
import config

class kinSim:
    """
    Digital Twin Simulator for the robotic arm.
    
    PyBullet operates on a Client-Server API. This class acts as the Python Client, 
    sending mathematical commands to the underlying C++ physics server which can handle 
    gravity, momentum, collisions, and graphics rendering in the background.
    """

    def __init__(self, urdf_path: str, initial_q: list, hardware_bridge=None):
        """
        Initializes the PyBullet physics server, loads the URDF environment, 
        and generates the interactive UI sliders.

        Args:
            urdf_path (str): Path to the robot's URDF file.
            initial_q (list): Starting joint angles to initialize the robot's pose.
            hardware_bridge (mcu_bridge, optional): The serial connection to the physical robot.
        """
        self.bridge = hardware_bridge

        # Connect to the GUI physics server and load base assets
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # before loading the floor, we need to add the search path to the pybullet_data asset folder to find it
        self.planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
        p.setGravity(0, 0, -9.81)

        # Disable unnecessary visual overlays for a cleaner UI
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    
        # Filter out fixed joints to isolate the controllable motors
        self.controllable_joints = []
        num_joints = p.getNumJoints(self.robotId)

        for i in range(num_joints):
            info = p.getJointInfo(self.robotId, i)
            if info[2] == p.JOINT_REVOLUTE or info[2] == p.JOINT_PRISMATIC:
                self.controllable_joints.append(i)

        # Locate the configured end-effector (TCP) index
        self.end_effector_index = self.controllable_joints[-1]
        for i in range(num_joints):
            if p.getJointInfo(self.robotId, i)[1].decode("utf-8") == config.TCP_JOINT_NAME:
                self.end_effector_index = i
                break

        # Generate interactive UI sliders for manual control based on URDF joint limits
        self.sliders = []
        for i, joint_idx in enumerate(self.controllable_joints):
            info = p.getJointInfo(self.robotId, joint_idx)
            lower_limit_deg = math.degrees(info[8])
            upper_limit_deg = math.degrees(info[9])
            initial_deg = math.degrees(initial_q[i])

            slider = p.addUserDebugParameter(f"Joint {i+1}", lower_limit_deg, upper_limit_deg, initial_deg)
            self.sliders.append(slider)

        # Initialize empty telemetry visualization lines
        self.line_x = p.addUserDebugLine([0,0,0], [0,0,0])
        self.line_y = p.addUserDebugLine([0,0,0], [0,0,0])
        self.line_z = p.addUserDebugLine([0,0,0], [0,0,0])
        self.text_x = p.addUserDebugText("", [0,0,0])
        self.text_y = p.addUserDebugText("", [0,0,0])
        self.text_z = p.addUserDebugText("", [0,0,0])

        self.set_pose(initial_q)
        p.stepSimulation()
        self.update_telemetry()

    def set_pose(self, q: list):
        """Instantly teleports the joints to the given angles, ignoring physics."""
        for i, joint_idx in enumerate(self.controllable_joints):
            p.resetJointState(self.robotId, joint_idx, q[i])

    def update_telemetry(self):
        """
        Retrieves the real-time global position of the Tool Center Point (TCP) 
        and updates the X (Red), Y (Green), and Z (Blue) distance tracking lines.
        """
        state = p.getLinkState(self.robotId, self.end_effector_index)
        ex, ey, ez = state[4][0], state[4][1], state[4][2]

        # drawing the X distance (line from 0, ey ,ez)
        p.addUserDebugLine([0, ey, ez], [ex, ey, ez], lineColorRGB=[1,0,0], lineWidth=2, replaceItemUniqueId=self.line_x)
        p.addUserDebugText(f"X: {ex*100:.1f} cm", [ex/2, ey, ez + 0.02], textColorRGB=[1,0,0], replaceItemUniqueId=self.text_x)

        # drawing the Y distance (line from ex, 0 ,ez)
        p.addUserDebugLine([ex, 0, ez], [ex, ey, ez], lineColorRGB=[0,1,0], lineWidth=2, replaceItemUniqueId=self.line_y)
        p.addUserDebugText(f"Y: {ey*100:.1f} cm", [ex, ey/2, ez + 0.02], textColorRGB=[0,1,0], replaceItemUniqueId=self.text_y)

        # drawing the Z distance (line from ex, ey ,0)
        p.addUserDebugLine([ex, ey, 0], [ex, ey, ez], lineColorRGB=[0,0,1], lineWidth=2, replaceItemUniqueId=self.line_z)
        p.addUserDebugText(f"Z: {ez*100:.1f} cm", [ex, ey, ez/2], textColorRGB=[0,0,1], replaceItemUniqueId=self.text_z)

    def animate(self, start_q, target_q, dt: float=config.SIM_DT):
        """
        Animates a smooth transition between two poses using physics-based motor control.
        
        Args:
            start_q (list): Starting joint angles.
            target_q (list): Target joint angles.
            dt (float): The time delay between frames to synchronize simulation with real-time.
        """
        trajectory = rtb.jtraj(start_q, target_q, 50)
        steps = int(dt*240)
        
        for q_step in trajectory.q:
            for i, joint_idx in enumerate(self.controllable_joints):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robotId,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=q_step[i],
                    force=config.MAX_MOTOR_FORCE
                )
            
            # Step the internal physics engine enough times to match the real-time delay
            for _ in range(steps):
                p.stepSimulation()
            
            self.update_telemetry()
            time.sleep(dt)

    def keep_open(self):
        """
        Transitions the simulation into 'Manual Mode'. 
        Continuously reads UI slider values and drives the joints to those positions
        until the user triggers a KeyboardInterrupt (Ctrl+C).
        """
        print("[NOTICE] Simulation idle. Sliders are now active. Press Ctrl+C in the terminal to exit.")
        try:
            while True:
                target_angles_rad = []
                current_angles_deg = []

                # Read current requested positions from the UI sliders
                for slider_id in self.sliders:
                    target_angle_deg = p.readUserDebugParameter(slider_id)
                    current_angles_deg.append(target_angle_deg)
                    target_angles_rad.append(math.radians(target_angle_deg))

                # Transmit manual movements to physical hardware if connected
                if self.bridge:
                    self.bridge.send_angles(current_angles_deg, is_radians=False)

                # Command virtual motors
                for i, joint_idx in enumerate(self.controllable_joints):
                    p.setJointMotorControl2(
                        bodyUniqueId=self.robotId,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angles_rad[i],
                        force=config.MAX_MOTOR_FORCE 
                    )

                p.stepSimulation()
                self.update_telemetry()
                time.sleep(config.SIM_DT)

        except KeyboardInterrupt:
            p.disconnect()
            print("\n[SYSTEM] PyBullet disconnected.")