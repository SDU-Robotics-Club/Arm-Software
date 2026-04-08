import serial
import time
import math
import config

class mcu_bridge:
    """
    Manages serial UART communication between the Python simulation and the physical MCU.
    Implements state-tracking to prevent buffer overflow on the hardware side.
    """
    def __init__(self, port=config.SERIAL_PORT, baudrate=config.SERIAL_BAUDRATE):
        """
        Initializes the serial connection. Fails gracefully to 'Simulation-Only' mode 
        if the hardware is unplugged.
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.last_sent_angles = [None] * config.NUM_JOINTS # State tracker
        
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)   # Allow MCU to reboot upon serial connection
            print(f"\n[HARDWARE] Connected to the MCU on {self.port}!")
        except serial.SerialException:
            print(f"\n[HARDWARE] WARNING: Could not connect to {self.port}. Running in Simulation-Only mode.")

    def send_angles(self, angles, is_radians=True):
        """
        Formats and transmits joint angles to the MCU.
        
        Args:
            angles (list): The list of joint angles.
            is_radians (bool): True if angles are in radians (from IK), False if degrees (from UI).
        """
        if self.serial_conn is None:
            return
        # rad -> deg conversion
        if is_radians:
            angles_deg = [math.degrees(q) for q in angles]
        else:
            angles_deg = list(angles) # Already in degrees (from UI sliders)

        # STATE TRACKER: Only transmit if a joint has moved by more than 0.5 degrees
        changed = False
        for i in range(config.NUM_JOINTS):
            # If this is the first time sending, or the difference is > 0.5 deg
            if self.last_sent_angles[i] is None or abs(angles_deg[i] - self.last_sent_angles[i]) > 0.5:
                changed = True
                break

        if changed:
            formatted_angles = ", ".join([f"{q:.2f}" for q in angles_deg])
            command = f"<{formatted_angles}>\n"
            
            self.serial_conn.write(command.encode('utf-8'))
            self.last_sent_angles = angles_deg

    def close(self):
        """Safely terminates the serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("[HARDWARE] Disconnected.")