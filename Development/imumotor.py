# imu_monitor.py
import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

print("=" * 50)
print("IMU Monitor - Read Only")
print("=" * 50)

while True:
    if arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8').strip()
            if line.startswith("IMU:"):
                data = line.split(":")[1]
                yaw, pitch, roll = data.split(",")
                print(f"Yaw: {float(yaw):6.2f}° | Pitch: {float(pitch):6.2f}° | Roll: {float(roll):6.2f}°")
        except:
            pass
    time.sleep(0.01)
