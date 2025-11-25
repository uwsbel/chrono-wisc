#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 20 17:40:16 2025

Subscribe images of the sinked wheel for estimating wheel sinkage

@author: Bo-Hsun Chen
"""

#!/usr/bin/env python3

import rclpy
from rclpy.executors import SingleThreadedExecutor
import sensor_msgs

import ros2_numpy as rnp # ros2-numpy for ROS to NumPy conversion

import cv2

############################
######## Parameters ########
############################
topic_name = "/rgba8_image"


# Shared state between callback and main loop
shared_state = {
	'latest_image': None,  # will hold NumPy array from ros2-numpy
}


def image_callback(msg):
	# Convert to NumPy array (H x W x C or H x W depending on encoding)
	img = rnp.numpify(msg)
	shared_state['latest_image'] = img

	node.get_logger().debug(f"Received image: shape={img.shape}, dtype={img.dtype}")


# node.get_logger().info(f'Subscribing to image topic: {topic_name}')

######################
######## MAIN ########
######################

rclpy.init()

# Single node that can hold multiple publishers/subscribers
node = rclpy.create_node("python_node")

image_sub = node.create_subscription(sensor_msgs.msg.Image, topic_name, image_callback, 10)

# Executor for the node
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)

try:
	node.get_logger().info('Starting main loop. Press Ctrl+C to exit.')

	while (rclpy.ok()):
		# Let ROS2 handle callbacks (which fill shared_state['latest_image'])
		executor.spin_once(timeout_sec=0.1)

		#### Do your image processing here ####
		img = shared_state.get('latest_image', None)
		if (img is not None):

			# ---- Your NumPy-based image processing here ----
			# Example: if it's a 3-channel image, convert to grayscale using OpenCV.
			# ros2-numpy usually gives something like (H, W, C) in RGB order.
			# If your encoding is 'bgr8', you may need to adjust this.

			img = cv2.flip(img, 0)
			img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
			window_name = 'Subscribed Image'
			cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
			cv2.resizeWindow(window_name, 640, 360)
			cv2.imshow(window_name, img)
			cv2.waitKey(1)
			
			shared_state['latest_image'] = None

		# Here you can also:
		# - publish other topics
		# - read sensors
		# - run other logic
	
except KeyboardInterrupt:
	node.get_logger().info('Shutting down (Ctrl+C received).')
	
finally:
	executor.shutdown()
	node.destroy_node()
	# rclpy.shutdown()
	cv2.destroyAllWindows()





