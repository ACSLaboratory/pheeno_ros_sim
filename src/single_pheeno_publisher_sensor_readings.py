#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan


def obstacle_check(data):
    global is_obstacle_in_way
    global sensor_triggered
    if data.ranges[0] < 0.3:
        is_obstacle_in_way = True
        sensor_triggered = data.header.frame_id[19:21]


if __name__ == "__main__":
    # Initialize Node
    rospy.init_node("pheeno_ir_sensor_readings")

    # Create Publishers
    pub = rospy.Publisher("/pheeno_01/scan_mux", Int8, queue_size=1)

    # Create Subscribers
    sub_cam_center = rospy.Subscriber("/pheeno_01/scan_center",
                                      LaserScan,
                                      obstacle_check)
    sub_cam_back = rospy.Subscriber("/pheeno_01/scan_back",
                                    LaserScan,
                                    obstacle_check)
    sub_cam_cl = rospy.Subscriber("/pheeno_01/scan_cl",
                                  LaserScan,
                                  obstacle_check)
    sub_cam_cr = rospy.Subscriber("/pheeno_01/scan_cr",
                                  LaserScan,
                                  obstacle_check)
    sub_cam_left = rospy.Subscriber("/pheeno_01/scan_left",
                                    LaserScan,
                                    obstacle_check)
    sub_cam_right = rospy.Subscriber("/pheeno_01/scan_right",
                                     LaserScan,
                                     obstacle_check)

    # Other important variables
    is_obstacle_in_way = False
    sensor_triggered = None
    sensors = {'no': 0, 'ce': 1, 'cr': 2, 'ri': 3, 'ba': 4, 'le': 5, 'cl': 6}
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        if is_obstacle_in_way:
            pub.publish(sensors[sensor_triggered])

            # Reset boolean
            is_obstacle_in_way = False

        else:
            pub.publish(sensors['no'])

        rate.sleep()
