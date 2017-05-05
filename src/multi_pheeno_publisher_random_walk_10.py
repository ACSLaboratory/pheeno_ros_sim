#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import Twist


def random_walk():
    if random.random() < 0.5:
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = -0.3

    else:
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.3

    return cmd_vel_msg


if __name__ == "__main__":

    # Initialize Node
    rospy.init_node("pheeno_random_walk")

    # Create Publishers
    pub = [0] * 10
    pub[0] = rospy.Publisher("/pheeno_01/cmd_vel", Twist, queue_size=100)
    pub[1] = rospy.Publisher("/pheeno_02/cmd_vel", Twist, queue_size=100)
    pub[2] = rospy.Publisher("/pheeno_03/cmd_vel", Twist, queue_size=100)
    pub[3] = rospy.Publisher("/pheeno_04/cmd_vel", Twist, queue_size=100)
    pub[4] = rospy.Publisher("/pheeno_05/cmd_vel", Twist, queue_size=100)
    pub[5] = rospy.Publisher("/pheeno_06/cmd_vel", Twist, queue_size=100)
    pub[6] = rospy.Publisher("/pheeno_07/cmd_vel", Twist, queue_size=100)
    pub[7] = rospy.Publisher("/pheeno_08/cmd_vel", Twist, queue_size=100)
    pub[8] = rospy.Publisher("/pheeno_09/cmd_vel", Twist, queue_size=100)
    pub[9] = rospy.Publisher("/pheeno_10/cmd_vel", Twist, queue_size=100)

    rate = rospy.Rate(2)
    count = [0] * 10
    ang_vel_msg = [0] * 10

    while not rospy.is_shutdown():
        for i in range(0, 10):
            if count[i] is 0:
                ang_vel_msg[i] = random_walk()
                pub[i].publish(ang_vel_msg[i])
                count[i] += 1

            elif 1 <= count[i] <= 15:
                pub[i].publish(ang_vel_msg[i])
                count[i] += 1

            elif 15 < count[i] < 35:
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.08
                pub[i].publish(cmd_vel_msg)
                count[i] += 1

            elif count[i] is 35:
                count[i] = 0

        rate.sleep()
