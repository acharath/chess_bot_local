"""Launch the USB camera node and ball detector.

This launch file is intended show how the pieces come together.
Please copy the relevant pieces.

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure the USB camera node
    node_usbcam = Node(
        name       = 'usb_cam', 
        package    = 'usb_cam',
        executable = 'usb_cam_node_exe',
        namespace  = 'board_detector',
        output     = 'screen',
        parameters = [{'camera_name':  'logitech'},
                      {'video_device': '/dev/video0'},
                      {'pixel_format': 'yuyv2rgb'},
                      {'image_width':  640},
                      {'image_height': 480},
                      {'framerate':    15.0}])

    # Configure the ball detector node
    node_balldetector = Node(
        name       = 'board_detector', 
        package    = 'final_demo',
        executable = 'board_detector',
        output     = 'screen',
        remappings = [('/image_raw', '/board_detector/image_raw')])


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the nodes.
        node_usbcam,
        node_balldetector,
    ])
