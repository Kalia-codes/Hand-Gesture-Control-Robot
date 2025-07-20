import cv2
import mediapipe as mp
import RPi.GPIO as GPIO
import time

# ==== GPIO Pin Setup ====
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 18
RIGHT_MOTOR_FORWARD = 22
RIGHT_MOTOR_BACKWARD = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)

# ==== PWM Setup ====
pwm_LF = GPIO.PWM(LEFT_MOTOR_FORWARD, 100)
pwm_LB = GPIO.PWM(LEFT_MOTOR_BACKWARD, 100)
pwm_RF = GPIO.PWM(RIGHT_MOTOR_FORWARD, 100)
pwm_RB = GPIO.PWM(RIGHT_MOTOR_BACKWARD, 100)

pwm_LF.start(0)
pwm_LB.start(0)
pwm_RF.start(0)
pwm_RB.start(0)

# Set motor speed (0 to 100%)
SPEED = 40

# ==== Movement Functions ====
def stop():
    """Stop all motors."""
    pwm_LF.ChangeDutyCycle(0)
    pwm_LB.ChangeDutyCycle(0)
    pwm_RF.ChangeDutyCycle(0)
    pwm_RB.ChangeDutyCycle(0)
    print("Motors stopped")

def move_forward():
    """Move the robot forward."""
    pwm_LF.ChangeDutyCycle(SPEED)
    pwm_LB.ChangeDutyCycle(0)
    pwm_RF.ChangeDutyCycle(SPEED)
    pwm_RB.ChangeDutyCycle(0)
    print("Moving Forward")

def move_backward():
    """Move the robot backward."""
    pwm_LF.ChangeDutyCycle(0)
    pwm_LB.ChangeDutyCycle(SPEED)
    pwm_RF.ChangeDutyCycle(0)
    pwm_RB.ChangeDutyCycle(SPEED)
    print("Moving Backward")

def turn_left():
    """Turn the robot to the left."""
    pwm_LF.ChangeDutyCycle(0)
    pwm_LB.ChangeDutyCycle(SPEED)
    pwm_RF.ChangeDutyCycle(SPEED)
    pwm_RB.ChangeDutyCycle(0)
    print("Turning Left")

def turn_right():
    """Turn the robot to the right."""
    pwm_LF.ChangeDutyCycle(SPEED)
    pwm_LB.ChangeDutyCycle(0)
    pwm_RF.ChangeDutyCycle(0)
    pwm_RB.ChangeDutyCycle(SPEED)
    print("Turning Right")

# ==== Mediapipe Setup ====
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

# ==== Camera Setup ====
cap = cv2.VideoCapture(0)

# ==== Distance Logic ====
DISTANCE_TIME = 2.5  # Time (in seconds) to move 1 meter
forward_start_time = None
backward_start_time = None
last_action = "stopped"

# ==== Finger Counting Function ====
def count_fingers(hand_landmarks):
    """Count the number of fingers held up based on landmarks."""
    tips_ids = [4, 8, 12, 16, 20]
    fingers = []

    # Thumb
    if hand_landmarks.landmark[tips_ids[0]].x < hand_landmarks.landmark[tips_ids[0] - 1].x:
        fingers.append(1)  # Thumb is up
    else:
        fingers.append(0)  # Thumb is down

    # Other fingers
    for id in range(1, 5):
        if hand_landmarks.landmark[tips_ids[id]].y < hand_landmarks.landmark[tips_ids[id] - 2].y:
            fingers.append(1)  # Finger is up
        else:
            fingers.append(0)  # Finger is down

    return fingers  # [thumb, index, middle, ring, pinky]

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)
        current_time = time.time()

        if result.multi_hand_landmarks:
            for handLms in result.multi_hand_landmarks:
                fingers = count_fingers(handLms)
                total_up = sum(fingers)

                # Gesture: Fist (0 fingers up) => Move Forward
                if total_up == 0:
                    if last_action != "forward":
                        forward_start_time = current_time
                        move_forward()
                        last_action = "forward"
                    elif forward_start_time and (current_time - forward_start_time >= DISTANCE_TIME):
                        stop()
                        last_action = "stopped"
                        print("Forward 1 meter reached -> Stop")

                # Gesture: Index + Middle (2 fingers up) => Move Backward
                elif total_up == 2 and fingers[1] == 1 and fingers[2] == 1:
                    if last_action != "backward":
                        backward_start_time = current_time
                        move_backward()
                        last_action = "backward"
                    elif backward_start_time and (current_time - backward_start_time >= DISTANCE_TIME):
                        stop()
                        last_action = "stopped"
                        print("Backward 1 meter reached -> Stop")

                # Gesture: Only Thumb (1 finger up) => Turn Right
                elif total_up == 1 and fingers[0] == 1:
                    if last_action != "right":
                        turn_right()
                        last_action = "right"
                        print("Turning Right")

                # Gesture: Any 3 fingers up => Turn Left
                elif total_up == 3:
                    if last_action != "left":
                        turn_left()
                        last_action = "left"
                        print("Turning Left")

                # Gesture: 4 or 5 fingers => Stop
                elif total_up >= 4:
                    if last_action != "stopped":
                        stop()
                        last_action = "stopped"
                        print("Stop")

                # Draw hand landmarks
                mp_draw.draw_landmarks(frame, handLms, mp_hands.HAND_CONNECTIONS)

        # Show the frame
        cv2.imshow("Gesture Control", frame)

        # Quit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    stop()
    pwm_LF.stop()
    pwm_LB.stop()
    pwm_RF.stop()
    pwm_RB.stop()
    GPIO.cleanup()
