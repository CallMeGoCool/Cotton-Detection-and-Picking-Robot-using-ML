from ultralytics import YOLO  # only used for class names if needed
import subprocess
import cv2
import numpy as np
import time

# -------- PARAMETERS --------
FOCAL_LENGTH = 850
REAL_WIDTH = 5
DETECTION_INTERVAL = 1.0

# -------- LOAD ONNX MODEL --------
net = cv2.dnn.readNetFromONNX("/home/randomjesus/Desktop/Final_sem_pjct/best_det.onnx")

# -------- CAMERA --------
width, height = 320, 240

cmd = [
    "rpicam-vid",
    "--width", str(width),
    "--height", str(height),
    "--framerate", "30",
    "--timeout", "0",
    "--codec", "yuv420",
    "--nopreview",
    "--output", "-"
]

pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)
frame_size = width * height * 3 // 2

def read_frame():
    raw = pipe.stdout.read(frame_size)
    if len(raw) != frame_size:
        return None
    yuv = np.frombuffer(raw, dtype=np.uint8).reshape((height * 3 // 2, width))
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

# -------- LOOP --------
last_detection_time = 0

while True:
    frame = read_frame()
    if frame is None:
        print("Camera ended")
        break

    current_time = time.time()

    if current_time - last_detection_time < DETECTION_INTERVAL:
        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) == 27:
            break
        continue

    last_detection_time = current_time

    h, w, _ = frame.shape
    center_x, center_y = w / 2, h / 2

    # -------- PREPROCESS --------
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (320, 320), swapRB=True, crop=False)
    net.setInput(blob)

    outputs = net.forward()

    # -------- POSTPROCESS --------
    # YOLOv8 ONNX output: [1, 84, N]
    outputs = outputs[0]
    outputs = np.transpose(outputs)  # shape: [N, 84]

    cottons = []
    output_strings = []

    for i, row in enumerate(outputs):
        scores = row[4:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        if confidence < 0.5:
            continue

        # Box format
        cx, cy, bw, bh = row[0:4]

        x1 = int((cx - bw / 2) * w / 320)
        y1 = int((cy - bh / 2) * h / 320)
        x2 = int((cx + bw / 2) * w / 320)
        y2 = int((cy + bh / 2) * h / 320)

        pixel_width = x2 - x1
        if pixel_width <= 0:
            continue

        # -------- DISTANCE --------
        distance = (FOCAL_LENGTH * REAL_WIDTH) / pixel_width

        # -------- POSITION --------
        center_box_x = (x1 + x2) / 2
        center_box_y = (y1 + y2) / 2

        dx = center_box_x - center_x
        dy = center_box_y - center_y

        X = (dx * distance) / FOCAL_LENGTH
        Y = (dy * distance) / FOCAL_LENGTH
        Z = distance

        label = f"cotton_{len(cottons)+1}"

        cottons.append((label, X, Y, Z, x1, y1, x2, y2, center_box_x, center_box_y))

        output_strings.append(
            f"{label}: {X:.1f}, {Y:.1f}, {Z:.1f}"
        )

    # -------- DRAW --------
    for c in cottons:
        label, X, Y, Z, x1, y1, x2, y2, cx, cy = c

        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 5, (0,0,255), -1)

        cv2.putText(frame, label, (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    # -------- PRINT --------
    if output_strings:
        print(" | ".join(output_strings))
        print("-----------")

    cv2.imshow("Detection", frame)

    if cv2.waitKey(1) == 27:
        break

pipe.terminate()
cv2.destroyAllWindows()
