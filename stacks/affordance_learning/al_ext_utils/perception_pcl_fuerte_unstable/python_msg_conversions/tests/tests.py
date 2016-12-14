import roslib; roslib.load_manifest('python_msg_conversions')

def point_arrays_equal(points1, points2):
    if not points1.dtype == points2.dtype:
        return False

    for field_name in points1.dtype.names:
        if not (points1[field_name] == points2[field_name]).all():
            return False
    return True

def test_roundtrip():
    from python_msg_conversions import pointclouds
    import numpy as np    

    npoints = 100

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

    cloud_msg = pointclouds.array_to_pointcloud2(points_arr, merge_rgb=True)
    new_points_arr = pointclouds.pointcloud2_to_array(cloud_msg, split_rgb=True)

    assert(point_arrays_equal(points_arr, new_points_arr))

