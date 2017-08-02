#!/usr/bin/env python

import rospy
import random
import argparse
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


def get_args():
    """ Get arguments from rosrun for individual deployment. """
    parser = argparse.ArgumentParser(
        description="Obstacle avoidance python script."
    )

    # Required arguments
    parser.add_argument("-n", "--number",
                        action="store",
                        required=False,
                        help="Add a pheeno number namespace.",
                        default="")

    # The rationale behind rospy.myargv()[1:] is provided here:
    # https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/ErXVWhRmtNA
    return parser.parse_args(rospy.myargv()[1:])


def random_turn():
    """ Random turn direction.

    Returns a random turn direction for the pheeno based on a uniform
    distribution.

    """
    if random.random() < 0.5:
        turn_direction = -0.07  # Turn left
        rospy.loginfo("Turning Left!")

    else:
        turn_direction = 0.07  # Turn right
        rospy.loginfo("Turning Right!")

    return turn_direction


def obstacle_callback(msg, ir_location):
    global is_obstacle_in_way
    global sensor_triggered
    global sensor_limits
    global sensor_values
    if msg.data < sensor_limits[ir_location]:
        sensor_triggered = ir_location
        sensor_values[ir_location] = msg.data
        is_obstacle_in_way[ir_location] = True

    else:
        sensor_values[ir_location] = msg.data
        is_obstacle_in_way[ir_location] = False


def avoid_obstacle():
    global is_obstacle_in_way
    global sensor_values
    global sensors
    global sensor_triggered

    if is_obstacle_in_way["center"] is True:
        if (sensor_values["right"] < 10 and
                sensor_values["left"] < 10):
            linear = 0
            angular = random_turn()

        if sensor_values["right"] < sensor_values["left"]:
            linear = 0
            angular = -0.07  # Turn Left

        else:
            linear = 0
            angular = 0.07  # Turn Right

    elif (is_obstacle_in_way["cl"] is True and
            is_obstacle_in_way["cr"] is True):
        linear = 0
        angular = random_turn()

    # If individual IR sensors are triggered.
    else:
        linear = 0
        angular = sensors[sensor_triggered]

    return linear, angular


if __name__ == "__main__":
    try:
        # Get arguments from argument parser.
        input_args = get_args()
        if input_args.number is "":
            pheeno_number = ""

        else:
            pheeno_number = "/pheeno_" + str(input_args.number)

        # Global Variables
        global is_obstacle_in_way
        global sensor_triggered
        global sensor_limits
        global sensor_values

        is_obstacle_in_way = {"center": False, "cr": False, "right": False,
                              "back": False, "left": False, "cl": False}
        sensor_triggered = 0
        sensors = {"center": 0.07, "cr": -0.07, "right": -0.07,
                   "back": -0.07, "left": 0.07, "cl": 0.07}
        sensor_limits = {"center": 20.0, "cr": 10.0, "right": 10.0,
                         "back": 15.0, "left": 10.0, "cl": 10.0}
        sensor_values = {"center": 0, "cr": 0, "right": 0,
                         "back": 0, "left": 0, "cl": 0}

        # Initialize Node
        rospy.init_node("pheeno_obstacle_avoidance")

        # Create Publishers and Subscribers
        pub = rospy.Publisher(
            pheeno_number + "/cmd_vel", Twist, queue_size=100)
        sub_ir_center = rospy.Subscriber(
            pheeno_number + "/scan_center", Float32, obstacle_callback,
            callback_args="center")
        sub_ir_back = rospy.Subscriber(
            pheeno_number + "/scan_back", Float32, obstacle_callback,
            callback_args="back")
        sub_ir_right = rospy.Subscriber(
            pheeno_number + "/scan_right", Float32, obstacle_callback,
            callback_args="right")
        sub_ir_left = rospy.Subscriber(
            pheeno_number + "/scan_left", Float32, obstacle_callback,
            callback_args="left")
        sub_ir_cr = rospy.Subscriber(
            pheeno_number + "/scan_cr", Float32, obstacle_callback,
            callback_args="cr")
        sub_ir_cl = rospy.Subscriber(
            pheeno_number + "/scan_cl", Float32, obstacle_callback,
            callback_args="cl")

        # Other important variables
        turn_direction = random_turn()
        cmd_vel_msg = Twist()
        rate = rospy.Rate(10)
        saved_time = rospy.get_rostime().secs
        current_duration = 0

        while not rospy.is_shutdown():
            # Find current duration
            current_duration = rospy.get_rostime().secs - saved_time

            if current_duration <= 10:
                if True in is_obstacle_in_way.values():
                    linear, angular = avoid_obstacle()
                    cmd_vel_msg.linear.x = linear
                    cmd_vel_msg.angular.z = angular

                else:
                    cmd_vel_msg.linear.x = 0
                    cmd_vel_msg.angular.z = turn_direction

            elif current_duration < 20:
                if True in is_obstacle_in_way.values():
                    linear, angular = avoid_obstacle()
                    cmd_vel_msg.linear.x = linear
                    cmd_vel_msg.angular.z = angular

                else:
                    cmd_vel_msg.linear.x = 0.05
                    cmd_vel_msg.angular.z = 0

            else:
                # Reset variables
                saved_time = rospy.get_rostime().secs
                turn_direction = random_turn()

            # Publish to cmd_vel Topic and sleep
            pub.publish(cmd_vel_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting 'pheeno_obstacle_avoidance' node.")
