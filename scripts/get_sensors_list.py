#!/usr/bin/env python

from ros_decawave.msg import Tag, Anchor, AnchorArray, Acc
import rospy

devices_list = set()

def devices(data):

    global devices_list
    
    for anchor in data.anchors:
        id_ = anchor.header.frame_id
        devices_list = devices_list.union(set([id_]))

    print(devices_list)


if __name__ == "__main__":

    rospy.init_node('list_devices', anonymous=True)
    rospy.Subscriber('/dwc4b8/tag_status', AnchorArray, devices)

    rospy.spin()



