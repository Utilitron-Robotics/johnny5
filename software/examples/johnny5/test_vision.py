from johnny5.vision import Johnny5Vision
import cv2
import numpy as np

def test_vision():
    print("\n=== Testing Johnny 5 Vision System ===")
    vision = Johnny5Vision()
    
    # Create a dummy image with a face (black square with white box)
    # In reality, you'd need a real face image for dlib to detect anything
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (200, 200), (400, 400), (255, 255, 255), -1)
    
    print("Processing dummy frame...")
    results = vision.process_frame(frame)
    
    print(f"Found {len(results)} faces")
    for face in results:
        print(f" - Name: {face.name}, Confidence: {face.confidence:.2f}")

if __name__ == "__main__":
    test_vision()