import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import itertools
import cv2



#++++++++This lab Assignemnt finished by ANASS NASSIRI in lab class++++++++++++++++



####### ############### ################### #####################

# Get two depth frames from the device you use now (check the previous labs)
# Visualize the two frames
# get the intrinsic camera parameters
# convert them to PCD and assign them to variables (using the intrinsic camera parameters)
####### ############### ################### #####################



####### ############### ################### #####################
    #voxel_size = 0.0005
    # Apply voxel downsampling to reduce the number of points


####### ############### ################### #####################
pcd_plane_o3d_1 = o3d.io.read_point_cloud("point_cloud_dept5.pcd")
pcd_plane_o3d_2 = o3d.io.read_point_cloud("point_cloud_dept6.pcd")
color_1 = [1, 0, 0]  # Red color
color_2 = [0, 1, 0]  # Red color
pcd_plane_o3d_1.paint_uniform_color(color_1)
pcd_plane_o3d_2.paint_uniform_color(color_2)

o3d.visualization.draw_geometries([pcd_plane_o3d_1])
o3d.visualization.draw_geometries([pcd_plane_o3d_2])



    # Perform ICP registration
threshold = 0.005  # Distance threshold
transformation_init = np.identity(4)

    ####### ############### ################### #####################
    # https://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_icp.html
    # you don't have to apply all the parameters of the function, you need to use:
    # the sourse pcd and the target , the threshold , the init transformation and TransformationEstimationPointToPoint
#first_icp=o3d.pipelines.registration.registration_icp(pcd_plane_o3d_1, pcd_plane_o3d_2,transformation_init,threshold,o3d.pipelines.registration.TransformationEstimationPointToPoint())
    ####### ############### ################### #####################
first_icp = o3d.pipelines.registration.registration_icp(
    pcd_plane_o3d_1,  # source
    pcd_plane_o3d_2,  # target
    threshold,
    transformation_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)



    ####### ############### ################### #####################
    # print the transformation matrix by assign the previous function of ICP registration to a variable (for example: Var)
    # print (Var.transformation)
    ####### ############### ################### #####################

print("this the",first_icp.transformation)



    ####### ############### ################### #####################

    # Transform the second point cloud to align with the first one
    # use this method:  pcd_2 .transform(Var.transformation)
    ####### ############### ################### #####################
pcd_plane_o3d_2.transform(first_icp.transformation)



    ####### ############### ################### #####################
    # Visualize the aligned point clouds
    #o3d.visualization.draw_geometries([pcd1, pcd2])
    ####### ############### ################### #####################
o3d.visualization.draw_geometries([pcd_plane_o3d_1, pcd_plane_o3d_2])

print("finish")



"""The selection of ICP parameters involves balancing accuracy and computational efficiency.
Start with a moderate voxel size to reduce noise and computation time, choose a max correspondence distance based on expected alignment, 
set tight convergence criteria for precision, and use Point-to-Plane for surface alignment. 
Adjust these parameters iteratively based on the specific characteristics of your point clouds
and the desired accuracy of the alignment."""