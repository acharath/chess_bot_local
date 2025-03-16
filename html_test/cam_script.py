from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

# Paths to your images
IMAGE_PATHS = [
    "/Users/jacobschuster/Downloads/chess_vision 134/html_test/board1.png",
    "/Users/jacobschuster/Downloads/chess_vision 134/html_test/board2.jpg"
]

def attach_to_existing_chrome():
    """Attaches Selenium to an already open Chrome session."""
    options = webdriver.ChromeOptions()
    options.debugger_address = "127.0.0.1:9222"  # Connect to the running Chrome instance

    driver = webdriver.Chrome(options=options)
    print("✅ Connected to existing Chrome session.")
    return driver

def process_image(driver, image_path):
    """Processes a single image: uploads, extracts FEN, and clicks cancel."""
    wait = WebDriverWait(driver, 8)  # Reduced wait time for faster execution

    try:
        # Click the "Scan" button
        scan_button = wait.until(EC.element_to_be_clickable((By.CLASS_NAME, "white-button-scan")))
        driver.execute_script("arguments[0].click();", scan_button)  # JavaScript click
        print(f"✅ Clicked Scan button for {image_path}")

        # Locate and upload the file
        file_input = wait.until(EC.presence_of_element_located((By.XPATH, "//input[@type='file']")))
        file_input.send_keys(image_path)
        print(f"✅ Uploaded file: {image_path}")

        # Wait for FEN to appear (instead of fixed sleep)
        fen_box = wait.until(EC.presence_of_element_located((By.CLASS_NAME, "copyable")))
        fen_text = fen_box.get_attribute("value")  # Get the text inside the input box
        print(f"\n🎯 **Extracted FEN for {image_path}:**\n{fen_text}\n")

        # Click Cancel
        cancel_button = wait.until(EC.element_to_be_clickable((By.XPATH, "//button[contains(text(), 'Cancel')]")))
        driver.execute_script("arguments[0].click();", cancel_button)  # JavaScript click
        print(f"✅ Clicked Cancel button for {image_path}\n")

    except Exception as e:
        print(f"❌ Error processing {image_path}: {e}")

if __name__ == "__main__":
    driver = attach_to_existing_chrome()

    # Process each image
    for image_path in IMAGE_PATHS:
        process_image(driver, image_path)

    print("✅ Done processing all images!")
