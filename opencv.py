import cv2
import mediapipe as mp
import serial
import time
import math

class HandController:
    def __init__(self, port='COM3', baudrate=9600):  # Change COM3 to your port
        # Initialize serial connection
        try:
            self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to Arduino on {port}")
        except Exception as e:
            print(f"Arduino connection failed: {e}")
            self.arduino = None
        
        # MediaPipe hands setup
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # Finger state thresholds
        self.finger_threshold = 0.05
        
    def calculate_finger_state(self, landmarks, finger_tip, finger_pip):
        """Check if finger is extended or bent"""
        tip_y = landmarks[finger_tip].y
        pip_y = landmarks[finger_pip].y
        
        # Finger is open if tip is above PIP joint (for vertical hand)
        return tip_y < pip_y - self.finger_threshold
    
    def get_hand_gesture(self, landmarks):
        """Get finger states from hand landmarks"""
        if not landmarks:
            return "00000"
        
        # Check each finger (thumb, index, middle, ring, pinky)
        thumb_open = self.calculate_finger_state(landmarks, 4, 2)      # Thumb tip vs thumb IP
        index_open = self.calculate_finger_state(landmarks, 8, 6)      # Index tip vs PIP
        middle_open = self.calculate_finger_state(landmarks, 12, 10)   # Middle tip vs PIP
        ring_open = self.calculate_finger_state(landmarks, 16, 14)     # Ring tip vs PIP
        pinky_open = self.calculate_finger_state(landmarks, 20, 18)    # Pinky tip vs PIP
        
        # Convert to string (1=closed, 0=open - matching Arduino expectation)
        gesture = ("1" if not thumb_open else "0" +
                  "1" if not index_open else "0" +
                  "1" if not middle_open else "0" +
                  "1" if not ring_open else "0" +
                  "1" if not pinky_open else "0")
        
        return gesture
    
    def send_to_arduino(self, command):
        """Send command to Arduino"""
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write((command + '\n').encode())
                return True
            except Exception as e:
                print(f"Send error: {e}")
                return False
        return False
    
    def process_frame(self, frame):
        """Process frame and detect hand gesture"""
        # Flip frame for mirror effect
        frame = cv2.flip(frame, 1)
        
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe
        results = self.hands.process(rgb_frame)
        
        gesture = "00000"  # Default: all fingers open
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Get gesture
                gesture = self.get_hand_gesture(hand_landmarks.landmark)
                
                # Send to Arduino
                if self.send_to_arduino(gesture):
                    # Display sent command on frame
                    cv2.putText(frame, f"Sent: {gesture}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display finger states
        finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        for i, state in enumerate(gesture):
            color = (0, 0, 255) if state == '1' else (0, 255, 0)  # Red=closed, Green=open
            status = "CLOSED" if state == '1' else "OPEN"
            cv2.putText(frame, f"{finger_names[i]}: {status}", (10, 70 + i*30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return frame
    
    def run(self):
        """Main loop"""
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("Cannot open camera")
            return
        
        print("Starting hand tracking...")
        print("Press 'q' to quit")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame")
                break
            
            # Process frame
            processed_frame = self.process_frame(frame)
            
            # Display
            cv2.imshow('Hand Controller', processed_frame)
            
            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        if self.arduino:
            self.arduino.close()

# Installation requirements (run these in terminal):
# pip install opencv-python mediapipe pyserial

if __name__ == "__main__":
    # Change 'COM3' to your Arduino port
    # Windows: COM3, COM4, etc.
    # Linux: /dev/ttyUSB0, /dev/ttyACM0
    # Mac: /dev/cu.usbmodemXXXX
    
    controller = HandController(port='COM3')  # CHANGE THIS TO YOUR PORT
    controller.run()
