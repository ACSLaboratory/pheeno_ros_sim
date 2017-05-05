#!/usr/bin/env python

import rospy
import random
from pheeno_ros.msg import Pheeno_IR_Sensor
from sensor_msgs.msg import LaserScan


def obstacle_check(data):
    global is_obstacle_in_way
    global sensor_triggered
    if data.ranges[0] < 0.3:
        # Pheeno number starting from 0 instead of 1
        pheeno = int(data.header.frame_id[8]) - 1

        # Log obstacle info
        is_obstacle_in_way[pheeno] = True
        sensor_triggered[pheeno] = data.header.frame_id[19:21]


if __name__ == "__main__":
    # Initialize Node
    rospy.init_node("pheeno_ir_sensor_readings")

    # Crate lists to hold objects
    pub = [0] * 5
    sub_cam_center = [0] * 5
    sub_cam_back = [0] * 5
    sub_cam_cl = [0] * 5
    sub_cam_cr = [0] * 5
    sub_cam_left = [0] * 5
    sub_cam_right = [0] * 5

    # Create Publishers and Subscribers
    for i in range(0, 5):
        pheeno_num = "/pheeno_0" + str(i + 1)
        pub[i] = rospy.Publisher(pheeno_num + "/scan_mux",
                                 Pheeno_IR_Sensor,
                                 queue_size=1)
        sub_cam_center[i] = rospy.Subscriber(pheeno_num + "/scan_center",
                                             LaserScan,
                                             obstacle_check)
        sub_cam_back[i] = rospy.Subscriber(pheeno_num + "/scan_back",
                                           LaserScan,
                                           obstacle_check)
        sub_cam_cl[i] = rospy.Subscriber(pheeno_num + "/scan_cl",
                                         LaserScan,
                                         obstacle_check)
        sub_cam_cr[i] = rospy.Subscriber(pheeno_num + "/scan_cr",
                                         LaserScan,
                                         obstacle_check)
        sub_cam_left[i] = rospy.Subscriber(pheeno_num + "/scan_left",
                                           LaserScan,
                                           obstacle_check)
        sub_cam_right[i] = rospy.Subscriber(pheeno_num + "/scan_right",
                                            LaserScan,
                                            obstacle_check)

    # Other variables
    is_obstacle_in_way = [False] * 5
    sensor_triggered = [0] * 5
    sensors = {'no': 0, 'ce': 1, 'cr': 2, 'ri': 3, 'ba': 4, 'le': 5, 'cl': 6}
    pheeno_list = {"pheeno_01": 1, "pheeno_02": 2, "pheeno_03": 3,
                   "pheeno_04": 4, "pheeno_05": 5}
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        for i in range(0, 5):
            if is_obstacle_in_way[i]:
                message = Pheeno_IR_Sensor()
                message.pheeno = i + 1
                message.sensor_triggered = sensors[sensor_triggered[i]]
                pub[i].publish(message)

                # Reset Boolean
                is_obstacle_in_way[i] = False

            else:
                message = Pheeno_IR_Sensor()
                message.pheeno = i + 1
                message.sensor_triggered = sensors['no']
                pub[i].publish(message)

        rate.sleep()
