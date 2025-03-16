import subprocess
import time
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

IMAGE_PATH = "/Users/jacobschuster/Downloads/chess_vision 134/html_test/IMG_3467.JPG"

STOCKFISH_PATH = "/Users/jacobschuster/Downloads/chess_vision 134/Stockfish/src/stockfish"

def attach_to_existing_chrome():
    """Attaches Selenium to an already open Chrome session."""
    options = webdriver.ChromeOptions()
    options.debugger_address = "127.0.0.1:9222"

    driver = webdriver.Chrome(options=options)
    print("✅ Connected to existing Chrome session.")
    return driver

def process_image(driver, image_path):
    """Uploads an image, ensures the FEN updates, and clicks cancel."""
    wait = WebDriverWait(driver, 8)

    try:
        # Click the "Scan" button
        scan_button = wait.until(EC.element_to_be_clickable((By.CLASS_NAME, "white-button-scan")))
        driver.execute_script("arguments[0].click();", scan_button)  # JavaScript click
        print(f"✅ Clicked Scan button for {image_path}")

        # Locate and upload the file
        file_input = wait.until(EC.presence_of_element_located((By.XPATH, "//input[@type='file']")))
        file_input.send_keys(image_path)
        print(f"✅ Uploaded file: {image_path}")

        # Wait for FEN to appear
        fen_box = wait.until(EC.presence_of_element_located((By.CLASS_NAME, "copyable")))
        
        # Get initial FEN (before new one is loaded)
        initial_fen = fen_box.get_attribute("value")
        new_fen = initial_fen

        retries = 5  # checks 5 times
        for _ in range(retries):
            time.sleep(0.8)
            new_fen = fen_box.get_attribute("value")
            if new_fen != initial_fen and len(new_fen) > 10:
                break
        
        print(f"\n🎯 **Extracted FEN for {image_path}:**\n{new_fen}\n")

        # Click Cancel
        cancel_button = wait.until(EC.element_to_be_clickable((By.XPATH, "//button[contains(text(), 'Cancel')]")))
        driver.execute_script("arguments[0].click();", cancel_button)  # JavaScript click
        print(f"✅ Clicked Cancel button for {image_path}\n")

        return new_fen  # Return the updated FEN

    except Exception as e:
        print(f"❌ Error processing {image_path}: {e}")
        return None

def query_stockfish(fen):
    """Runs Stockfish with the given FEN position and gets the best move."""
    try:
        # Start Stockfish as a subprocess
        process = subprocess.Popen(
            [STOCKFISH_PATH], 
            stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )

        # Send FEN and "go depth 20" commands
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
                print(f"❌ Stockfish crashed on FEN: {fen}")
                best_move = "Stockfish crashed"
                break

        process.terminate()
        return best_move

    except Exception as e:
        print(f"❌ Error running Stockfish: {e}")
        return "Error"

if __name__ == "__main__":
    driver = attach_to_existing_chrome()

    # Process one image
    fen = process_image(driver, IMAGE_PATH)

    # If FEN was successfully extracted, send it to Stockfish
    if fen:
        best_move = query_stockfish(fen)
        print(f"🤖 **Stockfish Best Move:** {best_move}\n")

    print("✅ Done processing the image!")
