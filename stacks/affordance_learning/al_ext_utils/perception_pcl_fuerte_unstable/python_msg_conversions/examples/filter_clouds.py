import roslib; roslib.load_manifest('python_msg_conversions')
import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

from python_msg_conversions import pointclouds

def cloud_cb(cloud_msg):
    rospy.loginfo('Filtering cloud')
    cloud_arr = pointclouds.pointcloud2_to_array(cloud_msg, split_rgb=True)

    # filter points by color
    filtered_cloud_arr = cloud_arr[(cloud_arr['r'] < 128) * (cloud_arr['g'] < 50) * (cloud_arr['b'] < 50)]

    # filter out far away points
    filtered_cloud_arr = filtered_cloud_arr[filtered_cloud_arr['z'] < 1.5]
    
    filtered_cloud_msg = pointclouds.array_to_pointcloud2(
        filtered_cloud_arr, stamp=cloud_msg.header.stamp, frame_id=cloud_msg.header.frame_id, merge_rgb=True)
    cloud_pub.publish(filtered_cloud_msg)

# parse args
sub_topic = sys.argv[1]
pub_topic = sub_topic + '_filtered'

# create node
rospy.init_node('cloud_filterer', anonymous=True)
rospy.loginfo('Subscribing to: %s' % sub_topic)
rospy.loginfo('Publishing filtered clouds on: %s' % pub_topic)
cloud_sub = rospy.Subscriber(sub_topic, PointCloud2, cloud_cb)
cloud_pub = rospy.Publisher(pub_topic, PointCloud2)

rospy.spin()

