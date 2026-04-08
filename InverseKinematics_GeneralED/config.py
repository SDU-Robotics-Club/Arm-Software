import os
import roboticstoolbox as rtb

# URDF SETTINGS
# Put the URDF folder, containing the URDF file inside it, in the same directory as these scripts
URDF_FOLDER = ""
URDF_FILENAME = ""
URDF_PATH = os.path.join(os.path.dirname(__file__), URDF_FOLDER, URDF_FILENAME)

# If the Tool Center Point position was configured while exporting the URDF file:
TCP_LINK_NAME = "tcp_link"
TCP_JOINT_NAME = "tcp_joint"

# KINEMATICS SETTINGS
NUM_JOINTS = 
# IK Solver Mask [X, Y ,Z, Roll, Pitch, Yaw] (1 = Calculate, 0 = Ignore)
DOF_MASK = [1, 1, 1, 1, 1, 1]

# EXTERNAL HARDWARE (SERIAL)
SERIAL_PORT = "COM3"
SERIAL_BAUDRATE = 115200

# SIMULATION SETTINGS
MAX_MOTOR_FORCE = 500   # PyBullet simulated motor torque
SIM_DT = 0.05           # time step for animation pauses

# ARM'S WORKSPACE BOUNDARY
MAX_REACH = 0.70

def load_robot(urdf_path: str) -> rtb.ERobot:
    """Loads a robot from a URDF file."""
    try:
        # ERobot is the standard class for URDF-based robots in RTB
        robot = rtb.ERobot.URDF(urdf_path)
        print(f"Successfully loaded robot from {urdf_path}")
        return robot
    except Exception as e:
        print(f"Error loading URDF: {e}")
        raise