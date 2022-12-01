#!/usr/bin/env python
"""
simple service
"""
"""
need catkin create package
need workspace
"""
import rospy
import random
from time import sleep

from geometry_msgs.msg import Vector3

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
#from matplotlib import pyplot as plt
from plant_arm_project.srv import *



class VisNode:
    def __init__(self):

        return

    def start(self, node_name="vis_node", service_name="vis"):
        rospy.init_node(node_name)
        rospy.loginfo("vis_node running")


        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                self.found_rgb = True
                break
        if not self.found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = 1.0 #1 meter
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Create pointcloud object
        self.pc = rs.pointcloud()

        '''
        pub = rospy.Publisher(node_name, Vector3, queue_size=10)
        rospy.init_node(node_name)
        rospy.loginfo(f"vis_node running: {pub.name})")

        r = rospy.Rate(10)
        msg = Vector3()
        msg.x = random.randint(0,10)
        msg.y = random.randint(0,10)
        msg.z = random.randint(0,10)

        while not rospy.is_shutdown():
            msg.x = random.randint(0,10)
            msg.y = random.randint(0,10)
            msg.z = random.randint(0,10)
            rospy.loginfo(msg)
            pub.publish(msg)
            r.sleep()       
        return
        '''

    def get_point(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            print ("NOT ALIGNED")

        # Get pointcloud from depth image
        points = self.pc.calculate(aligned_depth_frame)

        # Get individual points from pointcloud
        vtx = np.asanyarray(points.get_vertices()).reshape(480, 640)

        # Extract Color and Depth Image
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Generate mask from binary slices of individual color channels
        filtered = (color_image[:,:,0] < 80) & (color_image[:,:,1] < 128) #& (color_image[:,:,2] < 128)
        
        # Slice Color Image with mask
        color_image[:, :, 0] = color_image[:, :, 0] * filtered
        color_image[:, :, 1] = color_image[:, :, 1] * filtered
        color_image[:, :, 2] = color_image[:, :, 2] * filtered

        # Slice Depth Image with mask
        depth_image = depth_image * filtered
        depth_image = depth_image * filtered
        depth_image = depth_image * filtered

        # Slice pointcloud with mask 
        vtx = vtx[filtered > 0] # [x,][0] = (x_1, y_1, z_1)

        # Covert pointcloud from array of tupples to full numpy array [3, x]
        vtxStack = np.stack([vtx['f0'],vtx['f1'],vtx['f2']]) 

        # print(np.shape(depth_image), np.shape(vtx))
        # print(np.shape(vtx[filtered > 0]))

        # Calculate average point from all remaining points
        self.vtxAverage = np.mean(vtxStack, axis = 1)

        # Print for diagnostics
        print(self.vtxAverage)       


    # def _node_request_handler(self, msg):
    #     return
    #     # return pointResponse(msg.request)


    def get_rand_int():
        return 

    def service_call(self):
        print("here")
        rospy.wait_for_service("planthony")
        try:
            move_arm = rospy.ServiceProxy("planthony", PlantLocation)
            point_to_pass = Vector3()
            point_to_pass.x = self.vtxAverage[0] -.06
            point_to_pass.y = self.vtxAverage[1]-.06
            point_to_pass.z = self.vtxAverage[2]
            move_arm(point_to_pass)
            print("service call done")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def main():
    vis = VisNode()
    vis.start()
    while True:
        vis.get_point()
        vis.service_call()

        # Wait 5 seconds 
        sleep(5)
    #rospy.spin()


if __name__ == "__main__":
    main()
