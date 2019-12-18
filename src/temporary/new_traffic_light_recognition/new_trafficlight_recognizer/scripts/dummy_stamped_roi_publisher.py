#!/usr/bin/env python

import rospy

from sensor_msgs.msg import RegionOfInterest
from autoware_msgs.msg import StampedRoi

class DummyStampedRoiPublisher(object):

    def __init__(self):
        self.seq = 0

        self.roi_list = rospy.get_param('~roi', [[0,0,100,100]])
        print(self.roi_list)

        self.frame_id = "base_link"
        self.dynamic_signal = 1
        self.pub = rospy.Publisher('~output', StampedRoi, queue_size=1)
        rate = rospy.get_param('~rate', 1)
        self.timer = rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def publish(self, event):
        stamped_roi_msg = StampedRoi()
        stamped_roi_msg.header.seq = self.seq
        stamped_roi_msg.header.frame_id = self.frame_id
        stamped_roi_msg.header.stamp = event.current_real

        for i, proj_roi in enumerate(self.roi_list):
            roi = RegionOfInterest()
            roi.x_offset = proj_roi[0]
            roi.y_offset = proj_roi[1]
            roi.width = proj_roi[2]
            roi.height = proj_roi[3]
            stamped_roi_msg.signals.append(1)
            stamped_roi_msg.roi_array.append(roi)

        self.pub.publish(stamped_roi_msg)

if __name__ == '__main__':
    rospy.init_node('dummy_stamped_roi_publisher')
    DummyStampedRoiPublisher()
    rospy.spin()
