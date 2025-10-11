import cv2

# Your USB camera is at index 0
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not access USB camera at /dev/video0")
else:
    print("USB Camera opened successfully!")
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Error reading frame from camera.")
            break
        
        # Display the camera feed
        cv2.imshow('USB Camera Feed', frame)
        
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print("Camera released.")
