#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from franka_bimanual_controllers.cfg import dual_arm_teachingConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {target_coupling_stiffness}, {execution_factor}, {panda_left_enable_correction}, {panda_right_enable_correction}, {dual_enable_correction}, {panda_left_feedback_factor_position}, {panda_left_feedback_factor_stiffness}, {panda_right_feedback_factor_position}, {panda_right_feedback_factor_stiffness}, {attractor_distance_threshold}, {trajectory_distance_threshold}, {correction_length_scale}, {correction_window_size}""".format(**config))
    return config
 
if __name__ == "__main__":
    rospy.init_node("dual_teaching", anonymous = False)
 
    srv = Server(dual_arm_teachingConfig, callback)
    rospy.spin()
