import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import copy
import cv2

if __name__ == "__main__":

    ## Script parameters

    save_pcd = True
    visualize_closest_pcd_file = True

    # Camera intinsic Param
    #change the parameters according to the device you use.
#this is param for aaus

    #FX_DEPTH = 570.3422090067767
    #FY_DEPTH = 570.3422180043582
    #CX_DEPTH = 320.0
    #CY_DEPTH = 240.0
#this is pam for azure
    FX_DEPTH = 505.2602233886719
    FY_DEPTH = 505.3645935058594
    CX_DEPTH = 321.06671142578125
    CY_DEPTH = 331.1296691894531

    # Path to the folder containing the depth image file
    path = 'C:/Users/Ibtissam/OneDrive/Bureau/'
    #filename = 'Lab07/Depth_Asus.jpg'
    filename = 'Lab07/Depth_Asus5_near.jpg'

    # Load depth image
    depth_image = o3d.io.read_image(path + filename)
    depth_image = np.asarray(depth_image)
    depth_image = depth_image * 10 ** (-3)  # convert to meters
    print("Shape of the depth image:", depth_image.shape)
    
    # convert to grayscale depth for colored depth (maybe Azure) #######
    # depth_image = cv2.convertScaleAbs(depth_image, alpha=(255.0 / 5.0))  # Scale if original was in 16-bit
    # depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
    print("New shape of the depth image:", depth_image.shape)
    
    # Visualize depth image
    fig, axs = plt.subplots(1, 1)
    axs.imshow(depth_image, cmap="gray")
    axs.set_title('Depth image')
    plt.show()




    # Convert depth image to point cloud
    pcd = []
    height, width = depth_image.shape
    for y_pixel in range(height):
        for x_pixel  in range(width):
            z_world = depth_image[y_pixel][x_pixel]
            if z_world  == 0:
                continue
            x_world = (x_pixel - CX_DEPTH) * z_world / FX_DEPTH
            y_world = (y_pixel - CY_DEPTH) * z_world / FY_DEPTH
            pcd.append([x_world, y_world, z_world])

    # Create a pointcloud object and visualize

    pcd_o3d_from_depth = o3d.geometry.PointCloud()
    pcd_o3d_from_depth.points = o3d.utility.Vector3dVector(pcd)

#write and save the pcd
    o3d.visualization.draw_geometries([pcd_o3d_from_depth])
    if save_pcd:
        o3d.io.write_point_cloud("point_cloud_dept_near.pcd", pcd_o3d_from_depth, write_ascii=True)

    if not visualize_closest_pcd_file:
        exit()

