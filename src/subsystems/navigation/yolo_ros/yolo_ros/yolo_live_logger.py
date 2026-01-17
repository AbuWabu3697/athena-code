import cv2
import time
import csv
import os
from datetime import datetime
from ultralytics import YOLO
import signal

# ========= CONFIG =========
MODEL_PATH = "best.pt"   #  trained model
CAMERA_INDEX = 0         #  change this after running find_cameras.py

SAVE_VIDEO = False        # True = save annotated video, False = no video file
OUTPUT_DIR = "yolo_output"
VIDEO_FPS = 20.0         # output video fps (approx)
STOP = False
# ==========================

def handle_stop(sig, frame):
    global STOP
    STOP = True

signal.signal(signal.SIGINT, handle_stop)
signal.signal(signal.SIGTERM, handle_stop)

def main():
    # Make output directory
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Load YOLO model
    print(f"[INFO] Loading model from {MODEL_PATH}")
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    WEIGHTS_PATH = os.path.join(BASE_DIR, MODEL_PATH)   # if best.pt is in same folder as yolo_live_logger.py
    model = YOLO(WEIGHTS_PATH)
    print("[INFO] Classes:", model.names)

    # Open camera
    print(f"[INFO] Opening camera index {CAMERA_INDEX}")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[ERROR] Could not open camera. Try changing CAMERA_INDEX.")
        return

    # Get frame size for video writer
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Could not read initial frame from camera.")
        cap.release()
        return
    height, width = frame.shape[:2]

    # Setup video writer if needed
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_path = os.path.join(OUTPUT_DIR, f"yolo_run_{timestamp}.mp4")
    csv_path = os.path.join(OUTPUT_DIR, f"detections_log_{timestamp}.csv")

    if SAVE_VIDEO:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(video_path, fourcc, VIDEO_FPS, (width, height))
        print(f"[INFO] Saving video to {video_path}")
    else:
        writer = None

    # Setup CSV logging
    csv_file = open(csv_path, mode="w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "frame_id", "timestamp", "class_id", "class_name",
        "confidence", "x1", "y1", "x2", "y2", "width_px", "height_px"
    ])
    print(f"[INFO] Logging detections to {csv_path}")

    frame_id = 0
    fps = 0.0
    t_prev = time.time()

    print("[INFO] Press 'q' to quit.")
    while True:

        if STOP:
            break

        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to grab frame, stopping.")
            break

        t_start = time.time()

        # Run YOLO on the BGR frame
        results = model(frame, verbose=False)

        # Draw detections
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = model.names.get(cls_id, str(cls_id))

                # Draw box
                cv2.rectangle(frame, (x1, y1), (x2, y2),
                              (0, 255, 0), 2)

                # Label text
                label = f"{class_name} {conf:.2f}"
                cv2.putText(frame, label, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)

                # Log to CSV
                csv_writer.writerow([
                    frame_id,
                    datetime.now().isoformat(),
                    cls_id,
                    class_name,
                    f"{conf:.4f}",
                    x1, y1, x2, y2,
                    x2 - x1,
                    y2 - y1
                ])

        # FPS calculation
        t_end = time.time()
        fps = 1.0 / max(t_end - t_start, 1e-6)

        # Draw FPS on frame
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

        # Show frame
        #cv2.imshow("YOLO Live Logger", frame)

        # Write frame to video if enabled
        if writer is not None:
            writer.write(frame)

        frame_id += 1

        # Quit on 'q'
        #key = cv2.waitKey(1) & 0xFF
        #if key == ord('q'):
           # break

    # Cleanup
    cap.release()
    if writer is not None:
        writer.release()
    csv_file.close()
    cv2.destroyAllWindows()

    print("[INFO] Done.")


if __name__ == "__main__":
    main()
