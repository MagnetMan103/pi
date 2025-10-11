import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

print('Connected to Arduino')

while True:
    message = input("Enter message: ")
    if message.lower() == 'quit':
        break
    arduino.write((message + '\n').encode())
    print(f"Sent {message}")
    time.sleep(0.1)
    if arduino.in_waiting > 0:
        response = arduino.readline().decode(('utf-8').strip())
        print(f"Arduino says: {response}")
arduino.close()
print("Disconnected")
