import spatialmath as sm
import numpy as np
import config

class kinSolver:
    """
    Handles all mathematical calculations for the robotic arm.
    Utilizes the Robotics Toolbox (RTB) to calculate Forward and Inverse Kinematics.
    """

    def __init__(self, robot):
        """
        Initializes the solver with the loaded URDF robot model.
        """
        self.robot = robot

    def get_ik(self, target_pose: sm.SE3, q0: list = None) -> list:
        """
        Calculates Inverse Kinematics (IK) using the Levenberg-Marquardt algorithm.
        
        Args:
            target_pose (sm.SE3): The 4x4 homogeneous transformation matrix of the target. 
            Contains the end-effector's final position and orientation.
            q0 (list): The current joint angles (used as a starting guess for the solver).
            
        Returns:
            list: A list of 5 joint angles (in radians) required to reach the target position and orientation.
            
        Raises:
            ValueError: If the target is physically out of reach or violates joint limits.
        """

        reach = config.MAX_REACH

        # Calculate straight-line distance from base to target
        tx, ty, tz = target_pose.A[0, 3], target_pose.A[1, 3], target_pose.A[2, 3]
        target_distance = np.sqrt(tx**2 + ty**2 + tz**2)
        
        if target_distance > reach:
            print(f"\n[ERROR] Target is out of reach!")
            print(f"-> Distance to target: {target_distance*100:.1f} cm")
            print(f"-> Maximum arm reach:  {reach*100:.1f} cm")
            raise ValueError("Workspace limit violation.")

        sol = self.robot.ikine_LM(target_pose, q0=q0, mask=config.DOF_MASK, end=config.TCP_LINK_NAME)
        
        if sol.success:
            return sol.q


        # solver fails -> debugging and gathering info on unresolved target

        wanted_angles = sol.q
        wanted_angles_deg = np.degrees(wanted_angles)
        limits = self.robot.qlim # self.robot.qlim pulls the J limits directly form the urdf file
        limit_violated = False
        
        print(f"\n[DEBUG] IK Failed: {sol.reason}") # debugging feedback
        print(f"[DEBUG] The solver wanted these angles:")
        print(f"[DEBUG] {np.round(wanted_angles_deg, 2)}")
        
        # comparing all 5 wanted angles to their respective limits and printing out which ones are violated
        for i in range(len(wanted_angles)):
            q_deg = wanted_angles_deg[i]
            lower_deg = np.degrees(limits[0, i])
            upper_deg = np.degrees(limits[1, i])
            
            # wanted angle < lower limit?
            if wanted_angles[i] < limits[0, i]:
                print(f"-> Joint {i+1} BOUNDARY: Wanted {q_deg:.1f}°, Min is {lower_deg:.1f}°")
                limit_violated = True
                
            # wanted angle > upper limit?
            elif wanted_angles[i] > limits[1, i]:
                print(f"-> Joint {i+1} is BOUNDARY: Wanted {q_deg:.1f}°, Max is {upper_deg:.1f}°")
                limit_violated = True
        
        if not limit_violated:
            print("-> No limits violated.")
            print("Unreachable wrist orientation.")

        raise ValueError("Trajectory aborted.")
        
    def get_fk(self, q:list) -> sm.SE3:
        """
        Calculates the Forward Kinematics (FK) for a given set of joint angles.
        Returns the exact 3D position and orientation of the end-effector.
        """
        return self.robot.fkine(q, end=config.TCP_LINK_NAME)