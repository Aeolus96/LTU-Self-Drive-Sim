#!/usr/bin/env python3

import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

accumulated_distance = 0.0
previous_odom_msg = None
timer_started = False
time_limit_minutes = 60.0  # minutes
start_time = None


def odom_callback(odom_msg):
    global accumulated_distance, previous_odom_msg, timer_started, start_time, time_limit_minutes

    if previous_odom_msg is not None:
        # Calculate difference in distance (in x and y)
        dx = odom_msg.pose.pose.position.x - previous_odom_msg.pose.pose.position.x
        dy = odom_msg.pose.pose.position.y - previous_odom_msg.pose.pose.position.y
        distance = (dx**2 + dy**2) ** 0.5

        # Start the timer when the robot moves a significant distance
        if distance > 0.1:
            if not timer_started:
                rospy.loginfo("Timer started")
                timer_started = True
                start_time = time.time()  # Initialize global start_time

        if timer_started and start_time is not None:
            current_time = time.time()
            elapsed_time = current_time - start_time
            elapsed_minutes = elapsed_time / 60.0
            if elapsed_minutes >= time_limit_minutes:
                rospy.loginfo("Time limit reached")
                rospy.loginfo("Accumulated distance: %f meters", accumulated_distance)
                rospy.signal_shutdown("Time limit reached, shutting down the data collection node")
            else:
                # Update the accumulated distance if within time limit
                accumulated_distance += distance

        # Publish the accumulated distance
        distance_msg = Float32()
        distance_msg.data = accumulated_distance
        dist_pub.publish(distance_msg)

    previous_odom_msg = odom_msg


if __name__ == "__main__":
    rospy.init_node("data_collection", anonymous=True)

    # Publisher
    dist_pub = rospy.Publisher(rospy.get_param("~output_distance_topic"), Float32, queue_size=1)

    # Subscriber
    rospy.Subscriber(rospy.get_param("~input_odom_topic"), Odometry, callback=odom_callback, queue_size=1)

    rospy.loginfo("Data collection node initialized")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
