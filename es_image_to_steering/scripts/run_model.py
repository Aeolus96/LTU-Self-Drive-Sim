#!/usr/bin/env python3

import cv2
import keras
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from es_image_to_steering.cfg import EsSteeringConfig

bridge = CvBridge()


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

        predicted_steering = model.predict(cv_image)[0][0]

        # New message
        twist_msg = Twist()

        # Check if model is enabled and apply speed and steering
        if config_.enable:
            twist_msg.linear.x = config_.linear_speed
            twist_msg.angular.z = predicted_steering

        twist_pub.publish(twist_msg)

    except CvBridgeError as e:
        print(e)
        return


if __name__ == "__main__":
    rospy.init_node("es_model", anonymous=True)

    # Dynamic reconfigure
    Server(EsSteeringConfig, dyn_rcfg_cb)

    # Load model
    model = keras.models.load_model(rospy.get_param("~model_path"), safe_model=False)
    model.summary()

    # Publisher
    twist_pub = rospy.Publisher(rospy.get_param("~output_twist_topic"), Twist, queue_size=1)

    # Subscriber
    rospy.Subscriber(rospy.get_param("~input_image_topic"), Image, callback=image_callback, queue_size=1)

    print("ES based image to steering node initialized")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
