import cv2
import os
import time

# Camera index (change if needed)
CAMERA_INDEX = 0  # Use 1 or 2 if using an external camera

# Directory to save photos
SAVE_DIR = os.path.expanduser("~/Downloads/captured_images")

# Check if the save directory exists, if not, create it
if not os.path.exists(SAVE_DIR):
    print(f"📂 Creating directory: {SAVE_DIR}")
    os.makedirs(SAVE_DIR, exist_ok=True)
else:
    print(f"📂 Save directory already exists: {SAVE_DIR}")

# Open the camera
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("❌ Could not open camera.")
    exit()

print("📸 Type 'capture' in the terminal to take a photo. Press 'q' to quit.")

# Allow the camera to warm up
print("⏳ Warming up the camera...")
time.sleep(2)  # Give the camera time to adjust exposure

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame.")
        break

    cv2.imshow("Camera Feed", frame)

    # Get user input from the terminal
    user_input = input("📷 Type 'capture' to take a photo, or 'q' to quit: ").strip().lower()

    if user_input == "capture":
        print("⏳ Capturing image...")

        # **NEW FIX**: Grab multiple frames before saving
        for _ in range(10):  # Capture 10 frames to ensure exposure is set
            ret, frame = cap.read()
            time.sleep(0.1)  # Small delay between frames

        # Ensure the frame is valid before saving
        if ret:
            image_path = os.path.join(SAVE_DIR, "captured_image.jpg")
            cv2.imwrite(image_path, frame)
            print(f"✅ Photo saved at: {image_path}")
        else:
            print("❌ Failed to capture a valid image.")

    elif user_input == "q":
        break

cap.release()
cv2.destroyAllWindows()
print("👋 Camera closed.")
