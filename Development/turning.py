# angle_turn_control.py
import serial
import time
import threading

class Colors:
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    RESET = '\033[0m'

class HexapodAngleController:
    def __init__(self, port='/dev/ttyACM0', baud=9600):
        self.arduino = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        
        self.current_yaw = None
        self.running = True
        self.pause_reading = False  # Flag to pause background reading
        self.read_lock = threading.Lock()
        
        # Start background thread to read IMU
        self.read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.read_thread.start()
        
        # Wait for first IMU reading
        print("Waiting for IMU data...")
        while self.current_yaw is None:
            time.sleep(0.1)
        print(f"IMU initialized. Current yaw: {self.current_yaw:.2f}°")
    
    def _read_serial(self):
        """Background thread to continuously read from Arduino"""
        while self.running:
            # Pause reading when motor commands are being sent
            if self.pause_reading:
                time.sleep(0.05)
                continue
                
            if self.arduino.in_waiting > 0:
                try:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line.startswith("IMU:"):
                        data = line.split(":")[1]
                        yaw, pitch, roll = data.split(",")
                        with self.read_lock:
                            self.current_yaw = float(yaw)
                    elif line.startswith("MOTOR:"):
                        msg = line.split(":", 1)[1]
                        print(f"{Colors.BLUE}[MOTOR] {msg}{Colors.RESET}")
                    elif line.startswith("SYSTEM:"):
                        msg = line.split(":", 1)[1]
                        print(f"{Colors.YELLOW}[SYSTEM] {msg}{Colors.RESET}")
                except:
                    pass
            time.sleep(0.01)
    
    def get_current_yaw(self):
        """Get the current yaw angle"""
        with self.read_lock:
            return self.current_yaw
    
    def send_motor_command(self, direction, amplitude):
        """
        Send a motor command and wait for completion
        
        Args:
            direction: "LEFT" or "RIGHT"
            amplitude: Motor amplitude (10-100)
        """
        # Pause background reading
        self.pause_reading = True
        time.sleep(0.1)  # Give background thread time to pause
        
        # Clear any buffered data
        self.arduino.reset_input_buffer()
        
        # Send command
        command = f"{direction}:{amplitude}\n"
        print(f"{Colors.BLUE}→ Sending: {command.strip()}{Colors.RESET}")
        self.arduino.write(command.encode())
        
        # Wait for motor action to complete (4 seconds)
        # Read any responses during this time
        start_time = time.time()
        motor_complete = False
        
        while time.time() - start_time < 5.0:  # 5 second timeout
            if self.arduino.in_waiting > 0:
                try:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line:
                        if line.startswith("IMU:"):
                            # Update yaw even during motor action
                            data = line.split(":")[1]
                            yaw, pitch, roll = data.split(",")
                            with self.read_lock:
                                self.current_yaw = float(yaw)
                        elif line.startswith("MOTOR:"):
                            msg = line.split(":", 1)[1]
                            print(f"{Colors.BLUE}[MOTOR] {msg}{Colors.RESET}")
                            if "complete" in msg.lower():
                                motor_complete = True
                                break
                        elif line.startswith("SYSTEM:"):
                            msg = line.split(":", 1)[1]
                            print(f"{Colors.YELLOW}[SYSTEM] {msg}{Colors.RESET}")
                except:
                    pass
            time.sleep(0.05)
        
        if not motor_complete:
            print(f"{Colors.YELLOW}⚠ Motor timeout - assuming complete{Colors.RESET}")
        
        # Additional settling time after motor action
        print("Waiting for robot to settle...")
        time.sleep(1.0)
        
        # Resume background reading
        self.pause_reading = False
        
        # Give time for fresh IMU reading
        time.sleep(0.3)
    
    def normalize_angle(self, angle):
        """Normalize angle to -180 to 180 range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def calculate_turn_error(self, current, target):
        """Calculate shortest path error between current and target angle"""
        error = target - current
        return self.normalize_angle(error)
    
    def turn_to_angle(self, target_angle, tolerance=5.0, max_iterations=20):
        """
        Turn the hexapod to a target angle using adaptive amplitude control
        
        Args:
            target_angle: Desired angle in degrees (-360 to 360)
            tolerance: Acceptable error in degrees (default 5°)
            max_iterations: Maximum number of turn attempts (default 20)
        """
        # Normalize target angle to -180 to 180
        target_angle = self.normalize_angle(target_angle)
        
        print(f"\n{Colors.GREEN}{'='*60}{Colors.RESET}")
        print(f"{Colors.GREEN}Starting turn to {target_angle:.2f}°{Colors.RESET}")
        print(f"{Colors.GREEN}{'='*60}{Colors.RESET}\n")
        
        start_yaw = self.get_current_yaw()
        print(f"Starting yaw: {start_yaw:.2f}°")
        print(f"Target yaw: {target_angle:.2f}°")
        
        amplitude = 80  # Start with full amplitude
        iteration = 0
        previous_error = None
        
        while iteration < max_iterations:
            current_yaw = self.get_current_yaw()
            error = self.calculate_turn_error(current_yaw, target_angle)
            
            print(f"\n{Colors.YELLOW}--- Iteration {iteration + 1} ---{Colors.RESET}")
            print(f"Current yaw: {current_yaw:.2f}°")
            print(f"Error: {error:.2f}°")
            print(f"Amplitude: {amplitude}")
            
            # Check if we've reached the target
            if abs(error) <= tolerance:
                print(f"\n{Colors.GREEN}✓ Target reached!{Colors.RESET}")
                print(f"Final yaw: {current_yaw:.2f}°")
                print(f"Final error: {error:.2f}°")
                return True
            
            # Check if we overshot (error changed sign)
            if previous_error is not None:
                if (previous_error > 0 and error < 0) or (previous_error < 0 and error > 0):
                    # We overshot! Reduce amplitude by half
                    amplitude = max(amplitude // 2, 10)  # Minimum amplitude of 10
                    print(f"{Colors.YELLOW}⚠ Overshoot detected! Reducing amplitude to {amplitude}{Colors.RESET}")
            
            # Determine turn direction
            if error > 0:
                direction = "LEFT"
                print(f"→ Turning LEFT with amplitude {amplitude}")
            else:
                direction = "RIGHT"
                print(f"→ Turning RIGHT with amplitude {amplitude}")
            
            # Send motor command and wait for completion
            self.send_motor_command(direction, amplitude)
            
            previous_error = error
            iteration += 1
        
        # Max iterations reached
        print(f"\n{Colors.RED}⚠ Max iterations reached{Colors.RESET}")
        current_yaw = self.get_current_yaw()
        final_error = self.calculate_turn_error(current_yaw, target_angle)
        print(f"Final yaw: {current_yaw:.2f}°")
        print(f"Final error: {final_error:.2f}°")
        return False
    
    def close(self):
        """Close the serial connection"""
        self.running = False
        time.sleep(0.1)
        self.arduino.close()
        print("Disconnected")

def main():
    print("=" * 60)
    print("Hexapod Angle-Based Turn Control")
    print("=" * 60)
    print("Enter target angles from -360 to 360 degrees")
    print("  Positive = Turn Right")
    print("  Negative = Turn Left")
    print("  Examples: 90, -90, 180, -180, 360")
    print("  Type 'quit' to exit")
    print("=" * 60)
    
    controller = HexapodAngleController()
    
    try:
        while True:
            user_input = input(f"\n{Colors.BLUE}Enter target angle (degrees): {Colors.RESET}")
            
            if user_input.lower() == 'quit':
                break
            
            try:
                target = float(user_input)
                
                # Validate range
                if target < -360 or target > 360:
                    print(f"{Colors.RED}Error: Angle must be between -360 and 360{Colors.RESET}")
                    continue
                
                # Perform the turn
                controller.turn_to_angle(target)
                
            except ValueError:
                print(f"{Colors.RED}Error: Please enter a valid number{Colors.RESET}")
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    finally:
        controller.close()

if __name__ == "__main__":
    main()
