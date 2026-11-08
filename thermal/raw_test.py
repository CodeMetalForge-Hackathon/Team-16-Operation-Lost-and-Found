
import cv2

# Connect to HT-203U
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Configure YUY2 video
cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*"YUY2")
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 392)
cap.set(cv2.CAP_PROP_FPS, 25)

# Request unconverted frames
conversion_set = cap.set(
    cv2.CAP_PROP_CONVERT_RGB, 0
)

if not cap.isOpened():
    raise RuntimeError("Camera could not be opened")

print("Conversion setting accepted:", conversion_set)
print(
    "Conversion setting reported:",
    cap.get(cv2.CAP_PROP_CONVERT_RGB)
)

try:
    ret, frame = cap.read()

    if not ret:
        raise RuntimeError("Frame capture failed")

    print("Frame shape:", frame.shape)
    print("Data type:", frame.dtype)
    print("Bytes:", frame.nbytes)

    print(
        "FourCC:",
        int(cap.get(cv2.CAP_PROP_FOURCC))
    )

finally:
    cap.release()
