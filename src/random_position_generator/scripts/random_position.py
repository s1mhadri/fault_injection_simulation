#!/usr/bin/env python3

import rospy
from random import uniform
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point


def randomize_cube_position():
    rospy.wait_for_service("/gazebo/set_model_state")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    # # # SET CUBES POSITION # # #
    model_name_red = "cube"  # Replace with your cube model name
    x_start_min, x_start_max = 0.3, 0.5  # 0.3, 0.5 Adjust the desired range for x position
    y_start_min, y_start_max = -0.4, -0.2  # -0.4, -0.2 Adjust the desired range for y position
    min_z, max_z = 0.0001, 0.001  # Adjust the desired range for z position
    x_cord = uniform(x_start_min, x_start_max)
    y_cord = uniform(y_start_min, y_start_max)

    red_cube_state = ModelState()
    red_cube_state.model_name = model_name_red
    red_cube_state.pose = Pose()

    red_cube_state.pose.position.x = x_cord
    red_cube_state.pose.position.y = y_cord
    red_cube_state.pose.position.z = uniform(min_z, max_z)

    x_final_min, x_final_max = 0.3, 0.5   # 0.3, 0.5
    y_final_min, y_final_max = 0.2, 0.4   # 0.2, 0.4
    x_final = uniform(x_final_min, x_final_max)
    y_final = uniform(y_final_min, y_final_max)

    try:
        set_state(red_cube_state)
        rospy.set_param("x_start", x_cord)
        rospy.set_param("y_start", y_cord)
        rospy.set_param("x_final", x_final)
        rospy.set_param("y_final", y_final)

        rospy.logwarn_once("Cube position randomized.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call Gazebo service: %s", str(e))


if __name__ == "__main__":
    rospy.init_node("random_position")
    randomize_cube_position()
