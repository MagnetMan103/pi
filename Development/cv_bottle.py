import cv2
from ultralytics import YOLO
import time

def detect_bottles_yolo():
    
    # Load pre-trained YOLOv8 nano model (smallest/fastest)
    # Options: yolov8n.pt (nano), yolov8s.pt (small), yolov8m.pt (medium)
    model = YOLO('yolov8m.pt')
    
    print("Model loaded successfully!")
    print("Starting camera...")
    
    # Initialize camera
    
    camera = cv2.VideoCapture(1)
    frame_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'width: {frame_width} height {frame_height}')    

    if not camera.isOpened():
        print("Error: Could not access camera")
        return
    
    # Confidence threshold
    CONFIDENCE_THRESHOLD = 0.5  # 50% confidence
    
    print("\n" + "=" * 50)
    print("Bottle Detection Started")
    print(f"Confidence Threshold: {CONFIDENCE_THRESHOLD * 100}%")
    print("=" * 50 + "\n")
    
    while True:
        success, frame = camera.read()
        
        if not success:
            print("Warning: Failed to capture frame")
            time.sleep(0.05)
            continue
        
        # Run YOLO detection
        # classes=[39] filters for only 'bottle' class in COCO dataset
        # conf=CONFIDENCE_THRESHOLD sets minimum confidence
        results = model(frame, classes=[39], conf=CONFIDENCE_THRESHOLD, verbose=False)
        key = cv2.waitKey(1) & 0xFF
        display_statistics(results)        
        # Display the frame
    # Cleanup
    camera.release()
    cv2.destroyAllWindows()
    print("Camera closed. Goodbye!")


def display_statistics(results):
        # Get detection results
        bottles_detected = len(results[0].boxes)
        
        # Draw bounding boxes and labels on frame
        annotated_frame = results[0].plot()
        
        # Add custom information overlay
        # Background rectangle for text
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (5, 5), (350, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)
        
        # Display statistics
        cv2.putText(annotated_frame, f"Bottles Detected: {bottles_detected}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Print detailed detection info to console
        if bottles_detected > 0:
            print(f"\n[DETECTION] {bottles_detected} bottle(s) found:")
            for i, box in enumerate(results[0].boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  # Bounding box coordinates
                confidence = box.conf[0].cpu().numpy()       # Confidence score
                
                # Calculate center point
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                width = int(x2 - x1)
                height = int(y2 - y1)
                
                print(f"  Bottle {i+1}: Center=({center_x}, {center_y}), "
                      f"Size={width}x{height}px, Confidence={confidence*100:.1f}%")

        cv2.imshow('YOLOv8 Bottle Detection', annotated_frame)
if __name__ == "__main__":
    detect_bottles_yolo()
