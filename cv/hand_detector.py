import time
import argparse
import sys
import cv2
import numpy as np
import mediapipe as mp

def draw_keypoints_and_box(frame, hand_landmarks, color=(255, 255, 255), draw_connections=True):
    h, w = frame.shape[:2]
    # draw landmarks and connections using MediaPipe drawing utils
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands
    if draw_connections:
        mp_drawing.draw_landmarks(
            frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
            mp_drawing.DrawingSpec(color=color, thickness=2, circle_radius=2),
            mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
        )
    else:
        for lm in hand_landmarks.landmark:
            cx, cy = int(lm.x * w), int(lm.y * h)
            cv2.circle(frame, (cx, cy), 3, color, -1)

    # compute bounding box from landmarks
    xs = [lm.x for lm in hand_landmarks.landmark]
    ys = [lm.y for lm in hand_landmarks.landmark]
    x_min = int(min(xs) * w)
    y_min = int(min(ys) * h)
    x_max = int(max(xs) * w)
    y_max = int(max(ys) * h)
    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)


def main():
    parser = argparse.ArgumentParser(description="MediaPipe Hands webcam keypoint detector")
    parser.add_argument("--source", type=int, default=0, help="Camera index (default 0)")
    parser.add_argument("--max-hands", type=int, default=2, help="Max number of hands to detect")
    parser.add_argument("--model-complexity", type=int, default=1, choices=[0,1,2], help="Model complexity (0,1,2)")
    parser.add_argument("--detection-conf", type=float, default=0.5, help="Minimum detection confidence")
    parser.add_argument("--tracking-conf", type=float, default=0.5, help="Minimum tracking confidence")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.source)
    if not cap.isOpened():
        print("Failed to open camera", args.source)
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=args.max_hands,
        model_complexity=args.model_complexity,
        min_detection_confidence=args.detection_conf,
        min_tracking_confidence=args.tracking_conf,
    )

    fps = 0.0
    prev_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(img_rgb)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    draw_keypoints_and_box(frame, hand_landmarks, color=(0, 255, 0))

            # FPS
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / (now - prev_time + 1e-8))
            prev_time = now
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

            cv2.imshow("mediapipe_hands", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

    finally:
        hands.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
