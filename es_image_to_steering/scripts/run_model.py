#!/usr/bin/env python3

import collections

import cv2
import keras
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from es_image_to_steering.cfg import EsSteeringConfig


# Smoothing classes
class MovingAverage:
    def __init__(self, window_size=10):
        self.value = collections.deque(maxlen=window_size)

    def smooth(self, new_value):
        self.value.append(new_value)
        return sum(self.value) / len(self.value)


class ExponentialMovingAverage:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.value = None

    def smooth(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value


# dynamic reconfigure callback
def dyn_rcfg_cb(config, level):
    global config_

    config_ = config

    return config


# Process images
def image_callback(ros_image):
    """Image to steering conversion"""

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

        # Crop out the top part of the image so that remaining image is 640x180 and predict
        cv_image = cv_image[300:480, 0:640]
        predicted_steering = model.predict(np.expand_dims(cv_image, axis=0), verbose=0)[0][0]

        # Apply smoothing
        predicted_steering = smoother.smooth(predicted_steering)

        # New message
        twist_msg = Twist()

        # Check if model is enabled and apply speed and steering
        if config_.enable:
            twist_msg.linear.x = config_.linear_speed
            twist_msg.angular.z = predicted_steering * config_.steering_gain

        twist_pub.publish(twist_msg)

    except CvBridgeError as e:
        print(e)
        return


bridge = CvBridge()
smoother = ExponentialMovingAverage(alpha=0.2)

if __name__ == "__main__":
    rospy.init_node("es_model", anonymous=True)

    # Dynamic reconfigure
    Server(EsSteeringConfig, dyn_rcfg_cb)

    # Load model
    model = keras.models.load_model(rospy.get_param("~model_path"), safe_mode=False)
    model.summary()

    # Publisher
    twist_pub = rospy.Publisher(rospy.get_param("~output_twist_topic"), Twist, queue_size=1)

    # Subscriber
    rospy.Subscriber(rospy.get_param("~input_image_topic"), Image, callback=image_callback, queue_size=1)

    rospy.loginfo("ES based image to steering node initialized")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
