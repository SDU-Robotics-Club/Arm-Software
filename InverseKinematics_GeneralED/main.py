import config
import time
import math
from kinematics import kinSolver
from pb_kinsim import kinSim
from input_handler import get_target_pos_from_terminal
from hardware_bridge import mcu_bridge


def main():

    robot = config.load_robot(urdf_path=config.URDF_PATH)
    kin = kinSolver(robot)
    bridge = mcu_bridge(port=config.SERIAL_PORT, baudrate=config.SERIAL_BAUDRATE)
    sim = kinSim(config.URDF_PATH, initial_q=robot.q, hardware_bridge=bridge)

    print("\n--- Simulation Ready ---")
    
    while True:
        
        target_pose = get_target_pos_from_terminal(kin)
        current_q = robot.q # get the current joint angles of the robot, which will be used as the starting point (q0) for the IK solver and trajectory planner
        
        try:
            target_q = kin.get_ik(target_pose, q0=current_q)
            target_q_deg = [round(math.degrees(q), 2) for q in target_q]
            print(f"IK Solution Found: {target_q_deg}")
            
            bridge.send_angles(target_q, is_radians=True)
            sim.animate(current_q, target_q, dt=0.05)
            robot.q=target_q

        except ValueError:
            continue

        try:
            user_choice = input("\nNew position? (y/n): ").strip().lower()
            if user_choice == 'n':
                print("Closing terminal input.")
                break
    
        except KeyboardInterrupt:
            bridge.close()
            print("\n[NOTICE] Interrupted by terminal. Exiting simulation loop...")
            break

    sim.keep_open()

if __name__ == "__main__":
    main()