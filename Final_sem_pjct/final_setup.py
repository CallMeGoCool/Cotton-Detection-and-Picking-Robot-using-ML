from ultralytics import YOLO
import subprocess
import cv2
import numpy as np
import time
import serial

# -------- PARAMETERS --------
FOCAL_LENGTH = 522
REAL_WIDTH = 5

DETECTION_INTERVAL = 0.5
MIN_CONF = 0.55
MAX_JUMP = 4  # cm threshold

# stability
STABLE_COUNT_REQUIRED = 2
stable_count = 0

prev_X, prev_Y, prev_Z = None, None, None
last_valid = None

# -------- SERIAL (ARDUINO) --------
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # allow Arduino reset

last_sent = None

def send_coords(x, y, z):
    global last_sent

    # convert to integers (your required format)
    xi = int(x)
    yi = int(y)
    zi = int(z)

    current = (xi, yi, zi)

    # send only if changed
    if current != last_sent:
        msg = f"{xi},{yi},{zi}\n"
        ser.write(msg.encode())
        print("Sent to Arduino:", msg.strip())
        last_sent = current

# -------- LOAD MODEL --------
model = YOLO("/home/randomjesus/Desktop/Final_sem_pjct/best.pt")

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
last_print_time = 0

while True:
    frame = read_frame()
    if frame is None:
        print("Camera ended")
        break

    current_time = time.time()

    h, w, _ = frame.shape
    center_x, center_y = w / 2, h / 2

    results = model(frame, imgsz=416, conf=0.3, verbose=False)

    best_box = None
    best_conf = 0

    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])
            if conf > best_conf:
                best_conf = conf
                best_box = box

    valid_detection = False

    if best_box is not None and best_conf >= MIN_CONF:

        x1, y1, x2, y2 = map(int, best_box.xyxy[0])
        pixel_width = x2 - x1

        if pixel_width > 0:

            Z = (FOCAL_LENGTH * REAL_WIDTH) / pixel_width
            Z *= 0.9

            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            dx = cx - center_x
            dy = cy - center_y

            X = (dx * Z) / FOCAL_LENGTH
            Y = (dy * Z) / FOCAL_LENGTH

            # -------- JUMP FILTER --------
            if prev_X is not None:
                jump = abs(X - prev_X) + abs(Y - prev_Y)

                if jump > MAX_JUMP:
                    stable_count = 0
                else:
                    stable_count += 1
                    valid_detection = True
            else:
                stable_count += 1
                valid_detection = True

            prev_X, prev_Y, prev_Z = X, Y, Z

            # -------- ONLY ACCEPT STABLE --------
            if stable_count >= STABLE_COUNT_REQUIRED:
                last_valid = (X, Y, Z, best_conf, x1, y1, x2, y2, cx, cy)

    # -------- DRAW + PRINT + SEND --------
    if last_valid is not None:
        X, Y, Z, conf, x1, y1, x2, y2, cx, cy = last_valid

        if current_time - last_print_time >= DETECTION_INTERVAL:
            last_print_time = current_time

            print(f"cotton_1: {X:.1f}, {Y:.1f}, {Z:.1f}, conf:{conf:.2f}")
            print("-----------")

            # ✅ SEND TO ARDUINO (only when printing)
            send_coords(X, Y, Z)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 5, (0,0,255), -1)

        cv2.putText(frame, f"{conf:.2f}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        cv2.putText(frame, f"Z:{Z:.1f}cm", (x1, y2 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

    cv2.imshow("Detection", frame)

    if cv2.waitKey(1) == 27:
        break

pipe.terminate()
cv2.destroyAllWindows()
ser.close()
