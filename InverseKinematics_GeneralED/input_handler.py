import spatialmath as sm
import math
import time
import config

def get_target_pos_from_terminal(kin) -> sm.SE3:
    """
    Prompts the user for target XYZ coordinates and end-effector orientation.
    
    Returns:
        sm.SE3: A 4x4 spatial transformation matrix representing the target position and orientation of the end-effector.
    """
    while True:
        try:
            print("\n--- Enter Target Position (ENTER -> Home) ---")

            x_home = input("X (cm): ")

            # Check if the user wants to return to the Home position
            if x_home.strip() == "":
                print("Sending to Home Positon...")
                return kin.get_fk([0.0, 0.0, 0.0, 0.0, 0.0])
            
            # Input XYZ target coordinates (cm)
            x = float(x_home) / 100.0
            y = float(input("Y (cm): ")) / 100.0
            z = float(input("Z (cm): ")) / 100.0

            # 1. ROLL
            if config.DOF_MASK[3] == 1:
                roll = math.radians(float(input("Roll (deg): ")))
            else:
                roll = 0.0
                
            # 2. PITCH
            if config.DOF_MASK[4] == 1:
                pitch = math.radians(float(input("Pitch (deg): ")))
            else:
                pitch = 0.0

            # 3. YAW
            if config.DOF_MASK[5] == 1:
                yaw = math.radians(float(input("Yaw (deg): ")))
            else:
                # 5-DOF (or less): We dynamically calculate Yaw so the wrist points 
                # at the target, preventing Levenberg-Marquardt solver crashes.
                if x == 0 and y == 0:
                    yaw = 0.0 # Prevent division-by-zero if reaching straight up
                else:
                    yaw = math.atan2(y, x)

            # Generate and multiply translation and rotation matrices
            target_translation = sm.SE3(x, y, z)
            target_rotation = sm.SE3.RPY(roll, pitch, yaw)

            return target_translation * target_rotation

        except ValueError:
            """
            handling the input of non-numeric values
            """
            print("\n[INPUT ERROR]] Invalid input element detected")
            time.sleep(1.0)
            continue
        
        except KeyboardInterrupt:
            print("\n[NOTICE] Phantom interrupt caught. Retrying input...")
            time.sleep(0.5)
            continue
