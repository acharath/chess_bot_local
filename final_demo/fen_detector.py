# fen_detector.py

#!/usr/bin/env python3

import cv2
import tempfile
import time
import subprocess

import rclpy
from rclpy.node import Node

import cv_bridge

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

CHESSIFY_URL = "https://chessify.me/analysis"
CHROME_PATH = "/usr/bin/google-chrome"

class FenDetectorNode(Node):
    def __init__(self):
        super().__init__('fen_detector')

        self.get_logger().info("Starting FenDetectorNode...")

        # (Optional) Launch Chrome. If Chrome is already running with remote debugging,
        # remove this block or handle differently.
        try:
            subprocess.Popen([
                CHROME_PATH,
                "--remote-debugging-port=9222",
                "--user-data-dir=/tmp/chrome-debug",
                "--disable-popup-blocking",
                CHESSIFY_URL
            ])
            self.get_logger().info("Launched Chrome with remote debugging.")
        except Exception as e:
            self.get_logger().error(f"Failed to launch Chrome: {e}")

        # Give Chrome time to start listening on 9222
        time.sleep(3.0)

        # Attempt to attach
        self.driver = self.attach_to_chrome()
        if not self.driver:
            self.get_logger().error("Could not attach to Chrome. FEN extraction won't work.")

        # ROS image bridge
        self.bridge = cv_bridge.CvBridge()

        # Subscribe to images coming on /image_raw
        self.sub_image = self.create_subscription(Image, '/image_raw', self.image_callback, 1)

        # Subscribe to /fen_request
        self.sub_request = self.create_subscription(Bool, '/game_instr/fen_request', self.send_fen, 5)

        self.parse_image = False
        self.parse_time = time.time()
        self.delay = 3.0

        # Publish the FEN position
        self.pub_fen = self.create_publisher(String, 'fen_detector/fen_position', 5)

        self.get_logger().info("FenDetectorNode is running.")

    def attach_to_chrome(self):
        """
        Attempt to attach Selenium to an already open Chrome session
        listening on --remote-debugging-port=9222.
        Returns the driver if successful, otherwise None.
        """
        options = webdriver.ChromeOptions()
        options.debugger_address = "127.0.0.1:9222"
        try:
            driver = webdriver.Chrome(options=options)
            self.get_logger().info("Successfully attached to Chrome on port 9222.")
            return driver
        except Exception as e:
            self.get_logger().warn(f"Could not attach to Chrome: {e}")
            return None

    def send_fen(self, _):
        """
        Called whenever /fen_request receives a Bool.
        We set parse_image = True so the next received image triggers FEN extraction.
        """

        self.parse_image = True
        self.parse_time = time.time() + 2
        self.get_logger().info("FEN requested")

        # current_time = time.time()
        # if current_time - self.last_request > self.delay:
        #     self.parse_image = True
        #     self.last_request = current_time
        #     self.get_logger().info("FEN requested")

    def image_callback(self, msg):
        """
        1) Convert ROS -> OpenCV,
        2) Write to a temporary file,
        3) Upload that file to Chessify via Selenium,
        4) Extract FEN and publish it.
        """
        if self.parse_image and time.time() > self.parse_time:
            self.parse_image = False
            self.get_logger().info("inside image callback")
            if not self.driver:
                self.get_logger().warn("No valid Selenium driver. Skipping FEN extraction.")
                return

            # Convert from ROS to OpenCV (adjust encoding if needed)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            h, w = cv_image.shape[:2]
            desired_size = max(h, w)

            top = (desired_size - h) // 2
            bottom = desired_size - h - top
            left = (desired_size - w) // 2
            right = desired_size - w - left

            square_img = cv2.copyMakeBorder(
                cv_image,
                top,
                bottom,
                left,
                right,
                borderType=cv2.BORDER_CONSTANT,
                # value=[207, 156, 105] 
                value=[255, 255, 255] 
            )

            self.get_logger().info("WROTE IMAGE")
            cv2.imwrite("/home/robot/robotws/src/final_demo/final_demo/input.jpg", square_img)

            # Use a temporary file
            with tempfile.NamedTemporaryFile(suffix=".jpg") as tmp:
                cv2.imwrite(tmp.name, square_img)
                tmp.flush()

                fen = self.process_with_chessify(tmp.name)
                if fen:
                    self.get_logger().info(f"Extracted FEN: {fen}")
                    # Publish on a ROS topic
                    fen_msg = String()
                    fen_msg.data = fen
                    self.pub_fen.publish(fen_msg)

    def process_with_chessify(self, image_path):
        """
        1) Upload the provided image to Chessify,
        2) Wait for the FEN text to appear,
        3) Close the old Chessify tab, open a new one,
        4) Return the extracted FEN.
        """
        self.get_logger().info("inside")
        if not self.driver:
            return None

        try:
            wait = WebDriverWait(self.driver, 8)

            # 1) Click the "Scan" button
            scan_button = wait.until(
                EC.element_to_be_clickable((By.CLASS_NAME, "white-button-scan"))
            )
            self.driver.execute_script("arguments[0].click();", scan_button)

            # 2) Upload the file
            file_input = wait.until(
                EC.presence_of_element_located((By.XPATH, "//input[@type='file']"))
            )
            file_input.send_keys(image_path)
            self.get_logger().info("Uploaded file to Chessify.")

            # 3) Wait for the FEN box to appear and update
            fen_box = wait.until(
                EC.presence_of_element_located((By.CLASS_NAME, "copyable"))
            )

            initial_fen = fen_box.get_attribute("value")
            new_fen = initial_fen

            # Wait up to ~8 seconds total for the FEN to update
            for _ in range(10):
                time.sleep(0.8)
                new_fen = fen_box.get_attribute("value")
                if new_fen != initial_fen and len(new_fen) > 10:
                    break

            # 4) Click "Cancel" so the Chessify popup disappears
            # cancel_button = wait.until(
            #     EC.element_to_be_clickable((By.XPATH, "//button[contains(text(), 'Cancel')]"))
            # )
            # self.driver.execute_script("arguments[0].click();", cancel_button)
                
            done_button = wait.until(
                EC.element_to_be_clickable((By.XPATH, "//button[contains(text(), 'Done')]"))
            )
            self.driver.execute_script("arguments[0].click();", done_button)

            # ----------------------------
            # After finishing the scan, we assume there's exactly one Chessify tab open.
            # We'll open a new one and close the old one.
            # ----------------------------
            # old_tab = self.driver.current_window_handle

            # # 5) Open new tab with Chessify
            # self.driver.execute_script(f"window.open('{CHESSIFY_URL}', '_blank');")

            # time.sleep(1)  # Give Selenium a moment to register the new tab

            # all_handles = self.driver.window_handles
            # # Find whichever handle is not 'old_tab' (the new one)
            # new_tabs = [h for h in all_handles if h != old_tab]
            # if not new_tabs:
            #     self.get_logger().warn("No new tab found after opening Chessify. Possibly a race condition.")
            #     return new_fen if (new_fen and len(new_fen) > 10) else None

            # new_tab_handle = new_tabs[-1]  # Usually there's only one new tab

            # # 6) Switch to the new tab
            # self.driver.switch_to.window(new_tab_handle)

            # # 7) Close the old tab
            # if old_tab in self.driver.window_handles:
            #     self.driver.switch_to.window(old_tab)
            #     self.driver.close()

            # # 8) Switch again to the new tab
            # if new_tab_handle in self.driver.window_handles:
            #     self.driver.switch_to.window(new_tab_handle)

            # Return the extracted FEN
            return new_fen if (new_fen and len(new_fen) > 10) else None

        except Exception as e:
            self.get_logger().error(f"Error extracting FEN: {e}")
            return None

    def shutdown(self):
        # Gracefully close the Selenium driver if needed
        if self.driver:
            self.driver.quit()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FenDetectorNode()
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()
