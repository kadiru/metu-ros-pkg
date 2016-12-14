import roslib; roslib.load_manifest('python_msg_conversions')
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

from python_msg_conversions import pointclouds

rospy.init_node('pub_random_clouds', anonymous=True)

pub = rospy.Publisher('/random_clouds', PointCloud2)

npoints = 1000

while not rospy.is_shutdown():
    points_arr = np.zeros((npoints,), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('r', np.uint8),
        ('g', np.uint8),
        ('b', np.uint8)])
    points_arr['x'] = np.random.random((npoints,))
    points_arr['y'] = np.random.random((npoints,))
    points_arr['z'] = np.random.random((npoints,))
    points_arr['r'] = np.floor(np.random.random((npoints,))*255)
    points_arr['g'] = 0
    points_arr['b'] = 255

    #print points_arr
    cloud_msg = pointclouds.array_to_pointcloud2(points_arr, stamp=rospy.Time.now(), frame_id='/base_link', merge_rgb=True)
    pub.publish(cloud_msg)
    rospy.sleep(0.001)
