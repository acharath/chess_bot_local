from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time

def click_scan_button():
    """Automates Chessify by clicking the 'Scan' button."""
    options = webdriver.ChromeOptions()
    # Uncomment to run in headless mode
    # options.add_argument("--headless")

    driver = webdriver.Chrome(options=options)

    try:
        # Open Chessify
        driver.get("https://chessify.me/analysis")
        wait = WebDriverWait(driver, 15)  # Increased wait time

        # Close Cookie Consent Banner (if exists)
        try:
            cookie_button = wait.until(EC.element_to_be_clickable((By.XPATH, "//button[contains(text(), 'Accept')]")))
            cookie_button.click()
            print("✅ Closed cookie consent popup.")
            time.sleep(1)
        except Exception:
            print("⚠️ No cookie consent popup found, continuing...")

        # Click "Import/Export"
        import_button = wait.until(EC.element_to_be_clickable((By.XPATH, "//button[contains(text(), 'Import/ Export')]")))
        import_button.click()
        print("✅ Clicked Import/Export.")
        time.sleep(2)

        # Ensure "Game Formats" modal is fully visible
        wait.until(EC.presence_of_element_located((By.CLASS_NAME, "t-modal")))
        print("✅ Game Formats modal is detected.")

        # Click "Scan" tab
        try:
            scan_button = wait.until(EC.element_to_be_clickable((By.XPATH, "//li[contains(text(), 'Scan')]")))
            driver.execute_script("arguments[0].click();", scan_button)  # Use JavaScript click
            print("✅ Clicked Scan button.")
        except Exception as e:
            print(f"❌ Could not click Scan button: {e}")
            return None

        time.sleep(2)

    except Exception as e:
        print(f"❌ Error: {e}")
        return None

    finally:
        driver.quit()

if __name__ == "__main__":
    click_scan_button()
