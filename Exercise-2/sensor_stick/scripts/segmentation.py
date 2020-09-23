#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # DONE: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # DONE: Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    filtered_cloud = vox.filter()

    # DONE: PassThrough Filter
    passthrough = filtered_cloud.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.5
    axis_max = 1.6
    passthrough.set_filter_limits(axis_min, axis_max)
    filtered_cloud = passthrough.filter()

    # DONE: RANSAC Plane Segmentation
    seg = filtered_cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # DONE: Extract inliers and outliers
    cloud_table = filtered_cloud.extract(inliers, negative=False)
    cloud_objects = filtered_cloud.extract(inliers, negative=True)

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # DONE: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # DONE: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)


if __name__ == '__main__':

    # DONE: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # DONE: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # DONE: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # DONE: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()