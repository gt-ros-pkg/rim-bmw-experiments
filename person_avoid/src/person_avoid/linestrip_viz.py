#! /usr/bin/python

from collections import deque

import numpy as np

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def main():
    from ur_py_utils.arm_iface import ArmInterface
    from ur_kin_py.kin import Kinematics
    from hrl_geom.pose_converter import PoseConv

    rospy.init_node("linestrip_viz")

    line_strip = Marker()
    line_strip.header.frame_id = '/base_link'
    line_strip.ns = 'ee_trace'
    line_strip.type = Marker.LINE_STRIP
    line_strip.scale.x = 0.02
    line_strip.color.g = 1.0
    line_strip.color.a = 1.0
    markers = MarkerArray()

    pub = rospy.Publisher('visualization_marker_array', MarkerArray)

    arm = ArmInterface()
    kin = Kinematics(Kinematics.UR10_ID)

    pos_hist = deque()
    max_hist_len = 10

    r = rospy.Rate(5.)
    while not rospy.is_shutdown():
        line_strip.header.stamp = rospy.Time.now()
        q_cur = arm.get_q()
        x_cur = kin.forward(q_cur)
        pos_cur, _ = PoseConv.to_pos_quat(x_cur)
        if len(pos_hist) == max_hist_len:
            pos_hist.popleft()
        pos_hist.append(pos_cur)
        line_strip.points = []
        for pos in pos_hist:
            pt = Point()
            pt.x = pos[0]; pt.y = pos[1]; pt.z = pos[2]
            line_strip.points.append(pt)
        if len(pos_hist) == max_hist_len:
            markers.markers = [line_strip]
            pub.publish(markers)
        r.sleep()

if __name__ == "__main__":
    main()
