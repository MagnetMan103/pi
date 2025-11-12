# motor_control.py
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

print("=" * 50)
print("Motor Control")
print("=" * 50)

while True:
    if arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8').strip()
            if line.startswith("MOTOR:") or line.startswith("SYSTEM:"):
                print(line)
        except:
            pass
    
    command = input("Command (1/2/3/4): ")
    if command.lower() == 'quit':
        break
    arduino.write((command + '\n').encode())

arduino.close()
