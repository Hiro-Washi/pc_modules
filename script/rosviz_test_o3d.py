#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class PointCloudVisualizer:
    def __init__(self, topic):
        self.point_cloud_o3d = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.add_geometry(self.point_cloud_o3d)
        self.sub = rospy.Subscriber(topic, PointCloud2, self.callback)

    def callback(self, msg):
        # Convert the ROS point cloud message to a numpy array
        cloud_arr = np.array(
            list(pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)))

        # Update the point cloud for Open3D
        self.point_cloud_o3d.points = o3d.utility.Vector3dVector(cloud_arr)

        # Update the visualization
        self.vis.update_geometry(self.point_cloud_o3d)
        self.vis.poll_events()
        self.vis.update_renderer()

if __name__ == "__main__":
    rospy.init_node('point_cloud_visualizer')

    # Specify your point cloud topic
    point_cloud_topic = "/camera/depth/points" #"/camera/depth/image_rect_raw"

    vis = PointCloudVisualizer(point_cloud_topic)

    # Keep the node running until it's shut down
    rospy.spin()
