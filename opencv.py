import cv2
import mediapipe as mp
import serial
import time
import argparse
import logging
from serial.serialutil import SerialException

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def parse_arguments():
    parser = argparse.ArgumentParser(description='Hand Gesture Control for Robotic Hand')
    parser.add_argument('--port', default='COM8', help='Serial port (default: COM8)')
    parser.add_argument('--camera', type=int, default=1, help='Camera index (default: 1)')
    parser.add_argument('--baud', type=int, default=9600, help='Baud rate (default: 9600)')
    parser.add_argument('--demo', action='store_true', help='Run in demo mode without serial connection')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
    return parser.parse_args()


def initialize_serial(port, baud_rate, demo_mode):
    """Initialize serial connection with robust error handling"""
    if demo_mode:
        logger.info("Running in DEMO mode - no serial connection")
        return None

    try:
        arduino = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        logger.info(f"Connected to Arduino on {port} at {baud_rate} baud")
        return arduino
    except SerialException as e:
        logger.error(f"Failed to connect to {port}: {e}")
        logger.info("Running in demo mode due to connection failure")
        return None


def send_serial_data(arduino, finger_data, demo_mode):
    """Send data to Arduino with error handling and retries"""
    data_str = ''.join(str(f) for f in finger_data)
    # ONLY CHANGE: Removed \n from this line
    message = f"${data_str}"  # CHANGED: was f"${data_str}\n"

    if demo_mode:
        logger.info(f"DEMO MODE - Would send: {message.strip()}")
        return True

    if arduino and arduino.is_open:
        try:
            arduino.write(message.encode())
            arduino.flush()  # Ensure data is sent
            logger.info(f"Sent to Arduino: {data_str}")
            return True
        except SerialException as e:
            logger.warning(f"Serial write failed, retrying: {e}")
            time.sleep(0.1)  # 100ms delay before retry
            try:
                arduino.write(message.encode())
                arduino.flush()
                logger.info(f"Sent to Arduino (retry): {data_str}")
                return True
            except SerialException as e2:
                logger.error(f"Serial write failed after retry: {e2}")
                return False
    return False


def fingers_up(landmarks):
    """Determine which fingers are up based on landmark positions"""
    tips_ids = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky
    fingers = []

    # Thumb (compare x-coordinates for right hand)
    if landmarks[tips_ids[0]].x < landmarks[tips_ids[0] - 1].x:
        fingers.append(1)
    else:
        fingers.append(0)

    # Other four fingers (compare y-coordinates)
    for id in range(1, 5):
        if landmarks[tips_ids[id]].y < landmarks[tips_ids[id] - 2].y:
            fingers.append(1)
        else:
            fingers.append(0)

    return fingers


def main():
    args = parse_arguments()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    logger.info(f"Starting Hand Gesture Control - Camera: {args.camera}, Port: {args.port}, Baud: {args.baud}")

    # Initialize camera
    cap = cv2.VideoCapture(args.camera)
    cap.set(3, 640)  # Width
    cap.set(4, 480)  # Height

    if not cap.isOpened():
        logger.error(f"Cannot open camera {args.camera}")
        return

    # Initialize MediaPipe Hands
    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5
    )

    logger.info("MediaPipe hand detector initialized successfully")

    # Initialize serial connection
    arduino = initialize_serial(args.port, args.baud, args.demo)

    logger.info("Hand Gesture Control Started! Press 'q' to quit")

    # FPS calculation
    fps_start_time = time.time()
    fps_frame_count = 0
    fps = 0

    try:
        while True:
            success, img = cap.read()
            if not success:
                logger.warning("Failed to read frame from camera")
                continue

            fps_frame_count += 1
            if time.time() - fps_start_time >= 1.0:
                fps = fps_frame_count
                fps_frame_count = 0
                fps_start_time = time.time()

            # Convert to RGB for MediaPipe
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = hands.process(img_rgb)

            # Display FPS on image
            cv2.putText(img, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Draw hand landmarks
                    mp_drawing.draw_landmarks(
                        img,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                        mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
                    )

                    # Get finger states
                    fingers = fingers_up(hand_landmarks.landmark)
                    logger.debug(f"Fingers detected: {fingers}")

                    # Only send data if FPS is reasonable to avoid overload
                    if fps >= 15:
                        send_serial_data(arduino, fingers, args.demo)
                    else:
                        logger.warning(f"Low FPS ({fps}), skipping serial send")

                    # Display finger states on image
                    cv2.putText(img, f"Fingers: {fingers}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Hand Gesture Control", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger.info("Quit signal received")
                break

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        if arduino and arduino.is_open:
            arduino.close()
            logger.info("Serial connection closed")
        logger.info("Hand Gesture Control stopped")


if __name__ == "__main__":
    main()
