import cv2
import os
import time
import subprocess
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

BASE_DIR = os.path.expanduser("~/robotws/src/final_demo")
CAPTURED_IMAGES_DIR = os.path.join(BASE_DIR, "captured_images")
STOCKFISH_PATH = os.path.join(BASE_DIR, "Stockfish/src/stockfish")

#Make directory if doesn't exist
if not os.path.exists(CAPTURED_IMAGES_DIR):
    print(f"Creating directory: {CAPTURED_IMAGES_DIR}")
    os.makedirs(CAPTURED_IMAGES_DIR, exist_ok=True)

#takes photo from camera and saves it
def capture_image():
    cap = cv2.VideoCapture(0) 

    if not cap.isOpened():
        print("Could not open camera.")
        return None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        cv2.imshow("Camera Feed", frame)

        user_input = input("Type 'capture' to take a photo, or 'q' to quit: \n").strip().lower()

        if user_input == "capture":
            print("Capturing image...\n")

            #multiple frames for exposure
            for _ in range(10):
                ret, frame = cap.read()
                time.sleep(0.1)
            
            #saves image with timestamp
            if ret:
                image_filename = f"captured_{int(time.time())}.jpg"
                image_path = os.path.join(CAPTURED_IMAGES_DIR, image_filename)
                cv2.imwrite(image_path, frame)
                print(f"Photo saved at: {image_path}\n")
                cap.release()
                cv2.destroyAllWindows()
                return image_path 

        elif user_input == "q":
            break


    cap.release()
    cv2.destroyAllWindows()
    return None

def attach_to_existing_chrome():
    """Attaches Selenium to an already open Chrome session."""
    options = webdriver.ChromeOptions()
    options.debugger_address = "127.0.0.1:9222"

    driver = webdriver.Chrome(options=options)
    print("Connected to existing Chrome session.\n")
    return driver

#uploads image to chessify and returns fen position
def process_image(driver, image_path):
    """Uploads an image, ensures the FEN updates, and clicks cancel."""
    wait = WebDriverWait(driver, 8)

    try:
        # Click the "Scan" button
        scan_button = wait.until(EC.element_to_be_clickable((By.CLASS_NAME, "white-button-scan")))
        driver.execute_script("arguments[0].click();", scan_button)

        # Locate and upload the file
        file_input = wait.until(EC.presence_of_element_located((By.XPATH, "//input[@type='file']")))
        file_input.send_keys(image_path)
        print(f"Uploaded file: {image_path}\n")

        # Wait for FEN to appear
        fen_box = wait.until(EC.presence_of_element_located((By.CLASS_NAME, "copyable")))

        # Ensure FEN updates properly
        initial_fen = fen_box.get_attribute("value")
        new_fen = initial_fen

        for _ in range(10):
            time.sleep(0.8)
            new_fen = fen_box.get_attribute("value")
            if new_fen != initial_fen and len(new_fen) > 10:
                break
        
        print(f"Extracted FEN: {new_fen}\n")

        # Click Cancel
        cancel_button = wait.until(EC.element_to_be_clickable((By.XPATH, "//button[contains(text(), 'Cancel')]")))
        driver.execute_script("arguments[0].click();", cancel_button)

        return new_fen

    except Exception as e:
        print(f"Error processing {image_path}: {e}")
        return None

#sends fen position to stockfish and returns best move
def query_stockfish(fen):
    """Runs Stockfish with the given FEN position and gets the best move."""
    try:
        process = subprocess.Popen(
            [STOCKFISH_PATH], 
            stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )

        process.stdin.write(f"position fen {fen}\n")
        process.stdin.write("go depth 20\n")
        process.stdin.flush()

        best_move = "No move found"
        while True:
            output_line = process.stdout.readline().strip()
            if output_line.startswith("bestmove"):
                best_move = output_line
                break
            elif "Segmentation fault" in output_line or "Illegal position" in output_line:
                print(f"Stockfish crashed on FEN: {fen}")
                best_move = "Stockfish crashed"
                break

        process.terminate()
        return best_move

    except Exception as e:
        print(f"Error running Stockfish: {e}")
        return "Error"

if __name__ == "__main__":
    
    #takes image
    captured_image_path = capture_image()

    if captured_image_path:
        driver = attach_to_existing_chrome()

        #gets fen from image
        fen = process_image(driver, captured_image_path)

        #sends fen to stockfish api
        if fen:
            best_move = query_stockfish(fen)
            print(f"Stockfish Best Move: {best_move}\n")

    print("✅ Done processing!")
