#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8


def random_walk():
    if random.random() < 0.5:
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = -0.3  # Turn right

    else:
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.3  # Turn left

    return cmd_vel_msg


def obstacle_check(msg):
    global is_obstacle_in_way
    global sensor_triggered
    if msg.data is not 0:
        is_obstacle_in_way = True
        sensor_triggered = msg.data

    else:
        is_obstacle_in_way = False

    print(is_obstacle_in_way)


if __name__ == "__main__":
    # Initialize Node
    rospy.init_node("pheeno_obstacle_avoidance")

    # Create Publishers
    pub = rospy.Publisher("/pheeno_01/cmd_vel",
                          Twist,
                          queue_size=100)

    # Create Subscribers
    sub = rospy.Subscriber("/pheeno_01/scan_mux",
                           Int8,
                           obstacle_check)

    is_obstacle_in_way = False
    sensor_triggered = 0
    sensors = {1: 0.3, 2: 0.3, 3: 0.3, 4: -0.3, 5: -0.3, 6: -0.3}
    rate = rospy.Rate(2)
    count = 0

    while not rospy.is_shutdown():
        if is_obstacle_in_way:
            ang_vel_msg = Twist()
            ang_vel_msg.angular.z = sensors[sensor_triggered]
            pub.publish(ang_vel_msg)

        else:
            if count is 0:
                ang_vel_msg = random_walk()
                pub.publish(ang_vel_msg)
                count += 1

            elif 1 <= count <= 15:
                pub.publish(ang_vel_msg)
                count += 1

            elif 15 < count < 35:
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.08
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 35:
                count = 0

        rate.sleep()
