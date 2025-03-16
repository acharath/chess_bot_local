#!/usr/bin/env python3
#
# board_detector.py
#

import cv2
import numpy as np

import rclpy
import cv_bridge

from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import ChessState, ChessSquare
from collections import defaultdict

DEBUG = True

class DetectorNode(Node):
    def __init__(self):
        super().__init__('board_detector')

        # Publisher for the debug image
        self.pub_board_rgb = self.create_publisher(Image, 'board_detector/image_annotated', 3)
        
        # Publisher for all 64 centers in a single message
        self.pub_chess_state = self.create_publisher(ChessState, 'board_detector/chess_state', 1)

        # OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # Subscribe to incoming images
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process_image, 1)

        # Center coordinates of Aruco markers
        self.x0 = -.1045
        self.y0 =  .4095

        self.get_logger().info("Detector (world coords) running...")

    def classify_coordinates(self, coordinates):
        """
        Given four (x, y) coordinates, label them as:
            upper left
            upper right
            bottom left
            bottom right
        """
        coordinates.sort(key=lambda p: (p[1], p[0]))  

        upper_left, upper_right = sorted(coordinates[:2], key=lambda p: p[0])
        lower_left, lower_right = sorted(coordinates[2:], key=lambda p: p[0])

        return {
            "upper left": upper_left,
            "upper right": upper_right,
            "bottom left": lower_left,
            "bottom right": lower_right,
        }

    def partition_coordinates(self, corner_points):
        """
        Given four (x, y) coordinates, create an 8x8 grid of (x, y) coordinates
        that equally partitions the original quadrilateral.
        """

        labeled_coordinates = self.classify_coordinates(corner_points.tolist())

        x1, y1 = labeled_coordinates["upper left"]
        x2, y2 = labeled_coordinates["bottom left"]
        x3, y3 = labeled_coordinates["bottom right"]
        x4, y4 = labeled_coordinates["upper right"]

        # first line is from top left to top right
        x_points = np.linspace(x1, x4, 8)
        y_points = np.linspace(y1, y4, 8)

        delta_x = (x2 - x1) / 7
        delta_y = (y2 - y1) / 7

        coordinates = []

        coordinates.append( [(x, y) for x, y in zip(x_points, y_points)] )

        for _ in range(7):
            x_points += delta_x
            y_points += delta_y

            coordinates.append( [(x, y) for x, y in zip(x_points, y_points)] )

        return coordinates

    def get_indicator_location(self, hsv_img):
        """
        Given an image in hsv pixel values, return the (x, y) coordinate and enclosing
        radius of the indicator piece.
        """

        player_lowerbound = np.array([85, 130, 170])
        player_upperbound = np.array([120, 190, 210])

        mask = cv2.inRange(hsv_img, player_lowerbound, player_upperbound)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().warn("Could not find turn indicator")
            return None
        
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        indicator = contours[0]

        (x, y), radius = cv2.minEnclosingCircle(indicator)

        return (int(x), int(y)), int(radius)

    
    def get_grayscale_value(self, image, center):
        h, w, _ = image.shape
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, center, 1, 255, thickness=-1)

        pixels_inside_circle = image[mask == 255]

        if len(pixels_inside_circle) == 0:
            return 0 

        avg_rgb = np.mean(pixels_inside_circle, axis=0)

        # rgb -> grayscale using the luminance formula
        avg_gray_value = 0.299 * avg_rgb[0] + 0.587 * avg_rgb[1] + 0.114 * avg_rgb[2]

        return avg_gray_value

    
    def process_image(self, msg):
        # Load the image
        assert msg.encoding == "rgb8"

        # Convert ROS -> OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        border_lowerbound = np.array([0, 70, 130])  # Lower bound for border
        border_upperbound = np.array([50, 170, 230])  # Upper bound for border

        # Create a mask to filter out purple regions
        mask = cv2.inRange(hsv, border_lowerbound, border_upperbound)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().warn("Could not find chessboard border")
            # Publish the new image
            if DEBUG:
                out_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
                self.pub_board_rgb.publish(out_msg)
            return

        # Sort contours by area and select the largest one (assumed to be the tape)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        tape_contour = contours[0]

        # Draw contour
        # cv2.drawContours(image, [tape_contour], -1, (0, 255, 0), 2)

        # Approximate the contour to get a quadrilateral
        peri = cv2.arcLength(tape_contour, True)
        approx = cv2.approxPolyDP(tape_contour, 0.02 * peri, True)

        if len(approx) != 4:
            self.get_logger().warn("Chessboard border not detected as a quadrilateral")
            # Publish the new image
            if DEBUG:
                out_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
                self.pub_board_rgb.publish(out_msg)
            return

        # Get the 4 corner points of the border
        tape_corners = np.squeeze(approx)

        # for corner in tape_corners:
        #     point = tuple(np.round(corner).astype(int))
        #     cv2.circle(image, point, 2, (255, 0, 0), -1)

        named_corners = self.classify_coordinates(tape_corners.tolist())
        center = np.mean(tape_corners, axis=0)

        inner_corners = np.array([
            np.float64(named_corners['upper left']) * 0.8 + 0.2 * center,
            np.float64(named_corners['bottom left']) * 0.8 + 0.2 * center,
            np.float64(named_corners['bottom right']) * 0.79 + 0.21 * center,
            np.float64(named_corners['upper right']) * 0.77 + 0.23 * center
        ])

        new_points = self.partition_coordinates(inner_corners)

        M = self.get_aruco_perspective_transform(image)

        robot_sum = 0
        player_sum = 0

        # Draw the detected corners
        for row in new_points:
            for point_idx, point in enumerate(row):
                # Ensure that the point is an integer tuple
                point = tuple(np.round(point).astype(int))

                if point_idx < 2:
                    robot_sum += self.get_grayscale_value(image, point)
                
                elif point_idx > 5:
                    player_sum += self.get_grayscale_value(image, point)

                cv2.circle(image, point, 2, (0, 255, 0), -1)

        if robot_sum > player_sum:
            self.get_logger().info("ROBOT IS WHITE")
        else:
            self.get_logger().info("ROBOT IS BLACK")

        # Begin constructing the chess message
        chess_msg = ChessState()
        
        # Determine whose turn it is
        turn_indicator = self.get_indicator_location(hsv)

        if M is not None:
            row_name = "A"
            col_name = "1"

            if turn_indicator is not None:
                (x, y), radius = turn_indicator

                x_world, y_world = self.apply_perspective_transform(M, x, y)
                chess_msg.indicator_x = float(x_world)
                chess_msg.indicator_y = float(y_world)

                robot_x, robot_y = inner_corners[1]
                player_x, player_y = inner_corners[2]

                dist_to_robot = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)
                dist_to_player = np.sqrt((x - player_x)**2 + (y - player_y)**2)

                if dist_to_robot < dist_to_player:
                    cv2.circle(image, (x, y), radius, (255, 0, 0), -1)
                    chess_msg.player_turn = chess_msg.ROBOT
                else:
                    cv2.circle(image, (x, y), radius, (0, 0, 255), -1)
                    chess_msg.player_turn = chess_msg.HUMAN

            for row in new_points:
                for p in row:
                    square_msg = ChessSquare()

                    u, v = tuple(np.round(p).astype(int))
                    x_world, y_world = self.apply_perspective_transform(M, u, v)
                    square_name = row_name + col_name

                    square_msg.x = float(x_world)
                    square_msg.y = float(y_world)
                    square_msg.name = square_name

                    chess_msg.squares.append(square_msg)

                    col_name = chr(ord(col_name) + 1)
                
                col_name = "1"
                row_name = chr(ord(row_name) + 1)
            
            self.pub_chess_state.publish(chess_msg)
        else:
            self.get_logger().warn("Not all Aruco markers detected")

        if DEBUG:
            out_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
            self.pub_board_rgb.publish(out_msg)

        # gray  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        # gray  = clahe.apply(gray)

        # mask_white    = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 80, 255]))
        # mask_black    = cv2.inRange(hsv, np.array([0, 0, 0]),   np.array([180, 255, 80]))
        # combined_mask = cv2.bitwise_or(mask_white, mask_black)

        # gray_masked  = cv2.bitwise_and(gray, gray, mask=combined_mask)
        # gray_blurred = cv2.GaussianBlur(gray_masked, (5, 5), 0)
        # edges        = cv2.Canny(gray_blurred, 30, 100)

        # circles = cv2.HoughCircles(
        #     edges,
        #     cv2.HOUGH_GRADIENT,
        #     dp=1.0,
        #     minDist=2,
        #     param1=200,
        #     param2=10,
        #     minRadius=2,
        #     maxRadius=10
        # )

        # MAX_DISTANCE       = 15
        # LOCATION_THRESHOLD = 15 
        # RADIUS_THRESHOLD   = 5  

        # square_centers = [pt for row in new_points for pt in row]
        
        # final_circles  = []  

        # if circles is not None:
        #     circles = np.uint16(np.around(circles))
        #     circles_list = circles[0].tolist()

        #     valid_circles = []
        #     for (cx, cy, r) in circles_list:
        #         dists = [np.linalg.norm(np.array([cx, cy]) - np.array(sc)) for sc in square_centers]
        #         if min(dists) <= MAX_DISTANCE:
        #             valid_circles.append((cx, cy, r))

        #     grouped_circles = defaultdict(list)
        #     for (cx, cy, r) in valid_circles:
        #         dists = [
        #             np.linalg.norm(np.array([cx, cy]) - np.array(sc))
        #             for sc in square_centers
        #         ]
        #         min_idx = int(np.argmin(dists))
        #         grouped_circles[min_idx].append((cx, cy, r))

        #     for center_idx, circle_list in grouped_circles.items():
        #         if not circle_list:
        #             continue

        #         all_x = [c[0] for c in circle_list]
        #         all_y = [c[1] for c in circle_list]
        #         all_r = [c[2] for c in circle_list]

        #         med_x = np.median(all_x)
        #         med_y = np.median(all_y)
        #         med_r = np.median(all_r)

        #         inliers = []
        #         for (cx, cy, r) in circle_list:
        #             dist_center = np.hypot(cx - med_x, cy - med_y)
        #             if dist_center <= LOCATION_THRESHOLD and abs(r - med_r) <= RADIUS_THRESHOLD:
        #                 inliers.append((cx, cy, r))

        #         if not inliers:
        #             continue

        #         all_x_in = [c[0] for c in inliers]
        #         all_y_in = [c[1] for c in inliers]
        #         all_r_in = [c[2] for c in inliers]

        #         final_x = np.median(all_x_in)
        #         final_y = np.median(all_y_in)
        #         final_r = np.median(all_r_in)

        #         final_circles.append((final_x, final_y, final_r))

        # def remove_all_overlapping_circles(circles, overlap_threshold=0.85):
        #     """
        #     Removes any circle that overlaps with another circle. If two circles overlap
        #     beyond overlap_threshold * (r1 + r2), both get removed from the final list.
        #     """
        #     if not circles:
        #         return []

        #     overlap_indices = set()
        #     for i in range(len(circles)):
        #         x1, y1, r1 = circles[i]
        #         for j in range(i+1, len(circles)):
        #             x2, y2, r2 = circles[j]
        #             dist = np.hypot(x2 - x1, y2 - y1)
        #             # Check overlap
        #             if dist < (r1 + r2) * overlap_threshold:
        #                 overlap_indices.add(i)
        #                 overlap_indices.add(j)

        #     filtered = [
        #         c for idx, c in enumerate(circles)
        #         if idx not in overlap_indices
        #     ]
        #     return filtered

        # final_circles = remove_all_overlapping_circles(final_circles)

        # for (cx, cy, r) in final_circles:
        #     cv2.circle(image, (int(cx), int(cy)), int(r), (255, 255, 255), thickness=2)


        # # Publish the new image
        # if DEBUG:
        #     out_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
        #     self.pub_board_rgb.publish(out_msg)


    def get_aruco_perspective_transform(self, frame):
        dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dict_4x4)

        if markerIds is None or len(markerIds) != 4 or set(markerIds.flatten()) != {1,2,3,4}:
            return None

        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

        uvMarkers = np.zeros((4,2), dtype='float32')
        for i in range(4):
            mID = markerIds[i,0]  # 1..4
            mean_corner = np.mean(markerCorners[i], axis=1).reshape(2)
            uvMarkers[mID-1,:] = mean_corner

        DX = .4145
        DY = .2465
        xyMarkers = np.float32([
            [self.x0 - DX, self.y0 + DY],  # ID=1
            [self.x0 + DX, self.y0 + DY],  # ID=2
            [self.x0 - DX, self.y0 - DY],  # ID=3
            [self.x0 + DX, self.y0 - DY],  # ID=4
        ])

        M = cv2.getPerspectiveTransform(uvMarkers, xyMarkers)
        return M

    def apply_perspective_transform(self, M, u, v):
        """
        Apply the 3x3 perspective transform M to pixel coords (u,v).
        Returns (xw, yw) in world coords.
        """
        uv = np.float32([[u, v]]).reshape(1,1,2)
        xy = cv2.perspectiveTransform(uv, M).reshape(2)
        return xy[0], xy[1]

    def shutdown(self):
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
