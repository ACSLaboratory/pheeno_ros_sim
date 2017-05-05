#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import Twist
from pheeno_ros.msg import Pheeno_IR_Sensor


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

    # Pheeno number (starting from 0 instead of 1)
    pheeno = msg.pheeno - 1

    if msg.sensor_triggered is not 0:
        # Log obstacle info
        is_obstacle_in_way[pheeno] = True
        sensor_triggered[pheeno] = msg.sensor_triggered

    else:
        is_obstacle_in_way[pheeno] = False


if __name__ == "__main__":
    # Initialize Node
    rospy.init_node("pheeno_obstacle_avoidance")

    # Crate lists to hold objects
    pub = [0] * 5
    sub = [0] * 5

    # Create Publishers and Subscribers
    for i in range(0, 5):
        pheeno_num = "/pheeno_0" + str(i + 1)
        pub[i] = rospy.Publisher(pheeno_num + "/cmd_vel",
                                 Twist,
                                 queue_size=100)

        sub[i] = rospy.Subscriber(pheeno_num + "/scan_mux",
                                  Pheeno_IR_Sensor,
                                  obstacle_check)

    # Other important variables
    is_obstacle_in_way = [False] * 5
    sensor_triggered = [0] * 5
    sensors = {1: 0.3, 2: 0.3, 3: 0.3, 4: -0.3, 5: -0.3, 6: -0.3}
    rate = rospy.Rate(2)
    count = [0] * 5

    while not rospy.is_shutdown():
        for i in range(0, 5):
            if is_obstacle_in_way[i]:
                ang_vel_msg = Twist()
                ang_vel_msg.angular.z = sensors[sensor_triggered[i]]
                pub[i].publish(ang_vel_msg)

            else:
                if count[i] is 0:
                    ang_vel_msg = random_walk()
                    pub[i].publish(ang_vel_msg)
                    count[i] += 1

                elif 1 <= count[i] <= 15:
                    pub[i].publish(ang_vel_msg)
                    count[i] += 1

                elif 15 < count[i] < 35:
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = 0.08
                    pub[i].publish(cmd_vel_msg)
                    count[i] += 1

                elif count[i] is 35:
                    count[i] = 0

        rate.sleep()
