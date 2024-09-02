import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import itertools
import cv2

#ansewrd by anass nassiri in lab class

if __name__ == '__main__':

    ### Parameters ###

    # Script parameters
    o3d_visualization = False
    visualize_closest_pcd_file = True
    save_pcd = True
    pcd_plane=[]

    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###
    #load the depth image and plot it (use the part from the Previous script of PCD)
    depth_image = cv2.imread('Depth_Asus5_near.jpg', cv2.IMREAD_UNCHANGED)
    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###



    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###
    # Camera parameters (Use the parameters which you got from the scripts )
    #FX_DEPTH =
    #FY_DEPTH =
    #CX_DEPTH =
    #CY_DEPTH =

    FX_DEPTH = 570.3422090067767
    FY_DEPTH = 570.3422180043582
    CX_DEPTH = 320.0
    CY_DEPTH = 240.0
    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###




######## plot image with the polygon

    # Get the bounds of the plane in the image
    # Get the 4 point delimiting the plane interactivily
    # (hint: matplotlib ginput can be useful for this)
    fig, axs = plt.subplots(1, 1)
    axs.imshow(depth_image, cmap="gray")
    axs.set_title('Depth image (select 4 points to delimit the plane)')
    bounds = np.asarray(plt.ginput(4, timeout=-1))
    plt.close()

    x_pixel_min = np.min(bounds[:, 0])
    x_pixel_max = np.max(bounds[:, 0])
    y_pixel_min = np.min(bounds[:, 1])
    y_pixel_max = np.max(bounds[:, 1])
    fig, axs = plt.subplots(1, 1)
    axs.imshow(depth_image, cmap="gray")
    axs.set_title('Depth image with polygon')
    plt.plot([x_pixel_min, x_pixel_max, x_pixel_max, x_pixel_min, x_pixel_min], [
        y_pixel_min, y_pixel_min, y_pixel_max, y_pixel_max, y_pixel_min], 'r-')
    plt.show()


    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###
    #!!!#   ### Convert the dept image to point cloud ### (use previous script PCD)
        ## Script parameters

    save_pcd = False
    visualize_closest_pcd_file = True

    # Camera intinsic Param
    #change the parameters according to the device you use.

    FX_DEPTH = 570.3422090067767
    FY_DEPTH = 570.3422180043582
    CX_DEPTH = 320.0
    CY_DEPTH = 240.0

    # Path to the folder containing the depth image file
    path = 'C:/Users/Ibtissam/OneDrive/Bureau/'
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
        o3d.io.write_point_cloud("point_cloudd.pcd", pcd_o3d_from_depth, write_ascii=True)

    if not visualize_closest_pcd_file:
        exit()

    #****###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###
    # Hint: convert both the whole point cloud and the point cloud inside the polygon
            # Build pcd of the points inside the polygon drawn
    if (x_pixel > x_pixel_min and x_pixel < x_pixel_max) and (y_pixel > y_pixel_min and y_pixel < y_pixel_max):
        pcd_plane.append([x_world, y_world, z_world])

    # Create point cloud objects
    pcd_o3d = o3d.geometry.PointCloud()
    pcd_o3d.points = o3d.utility.Vector3dVector(pcd)

    pcd_plane_o3d = o3d.geometry.PointCloud()
    pcd_plane_o3d.points = o3d.utility.Vector3dVector(pcd_plane)


    ### Plane fitting ###
    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###

   # Fit a plane using the points
    # Hint: http://www.open3d.org/docs/0.13.0/tutorial/geometry/pointcloud.html#Plane-segmentation
    #write the code using the link using Open3D to segment a plane from a point cloud using the Random Sample Consensus (RANSAC) algorithm.

   ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###
   # Load point cloud
    pcd_plane_o3d = o3d.io.read_point_cloud("point_cloud_dept_near.pcd")
    print("this point_cloud ",pcd_plane_o3d)

    # Apply RANSAC plane segmentation
    plane_model, inliers = pcd_plane_o3d.segment_plane(distance_threshold=900,
                                            ransac_n=3,
                                            num_iterations=1000)
    print("this is inliers",len(inliers))

    # Extract plane parameters
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    # Extract plane points
    plane_points = pcd_plane_o3d.select_by_index(inliers)

    # Visualize result (optional)
    o3d.visualization.draw_geometries([plane_points])

# Example mesh
    #
    # [0: top_left]   ___________[1: top_right]
    #                 |        /|
    #                 |  (0) /  |
    #                 |    / (1)|
    #                 |  /      |
    # [2: bottom_left]|/________|[3: bottom_right]

    # calculate the maximum and minimum bounds of the point cloud pcd_plane_o3d.
    # The get_max_bound() and get_min_bound() functions return the largest and smallest coordinates along each axis
    # that encapsulate all the points in the cloud
    pcd_max_bounds = pcd_plane_o3d.get_max_bound()
    pcd_min_bounds = pcd_plane_o3d.get_min_bound()

    x_world_min = pcd_min_bounds[0]
    x_world_max = pcd_max_bounds[0]

    y_world_min = pcd_min_bounds[1]
    y_world_max = pcd_max_bounds[1]

    # Get the z coordinate of the plane for each point in the plane
    #X and Y coordinates are set based on the combinations of x_world_min, x_world_max, y_world_min, and y_world_max.
    # we need to calculate the Z coordinate, is computed using the plane equation to solve for z, z= (-(ax+by+d)/c)
    #for Example: z coordeinate will be = (-d - a * x_world_max - b * y_world_max) / c


    corner_top_right = [x_world_max, y_world_max, (-d - a * x_world_max - b * y_world_max) / c]
###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###
    ## Complete the other 4 values :
    corner_top_left = [x_world_min, y_world_max, (-d - a * x_world_min - b * y_world_max) / c]
    corner_bottom_right = [x_world_max, y_world_min, (-d - a * x_world_max - b * y_world_min) / c]
    corner_bottom_left = [x_world_min, y_world_min, (-d - a * x_world_min - b * y_world_min) / c]   
    
###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###




#----------------------------------------------------------
# Create point cloud objects
pcd_plane_fit = [corner_top_left, corner_top_right, corner_bottom_left, corner_bottom_right]
pcd_plane_fit_points = o3d.utility.Vector3dVector(pcd_plane_fit)

# Define your mesh creation here, with correct triangle definition for a rectangle
np_vertices = np.array([corner_top_right, corner_top_left, corner_bottom_right, corner_bottom_left])
np_triangles = np.array([[0, 1, 2], [1, 2, 3]])  # Assuming these are the correct indices for your corners

mesh = o3d.geometry.TriangleMesh()
mesh.vertices = o3d.utility.Vector3dVector(np_vertices)
mesh.triangles = o3d.utility.Vector3iVector(np_triangles)

# Error computation
errors = []
error_image = np.zeros((height, width))

# Loop through each point in the point cloud
for point in pcd_plane_o3d.points:
    # Calculate the perpendicular distance from the point to the plane
    error = a * point[0] + b * point[1] + c * point[2] + d
    errors.append(error)

    # Mapping Errors to an Image
    if point[2] != 0:
        # Convert the 3D point coordinates X,Y,Z to 2D pixel coordinates using the intrinsic parameters of the depth camera
        x_pixel = int((point[0] * FX_DEPTH / point[2]) + CX_DEPTH)
        y_pixel = int((point[1] * FY_DEPTH / point[2]) + CY_DEPTH)
        if 0 <= x_pixel < width and 0 <= y_pixel < height:
            error_image[y_pixel, x_pixel] = error * 1000  # Convert error from meters to mm

# Convert errors from meters to mm
errors = np.array(errors) * 1000

# Calculate error metrics
max_error = np.max(errors)
min_error = np.min(errors)
mean_error = np.mean(errors)
std_dev = np.std(errors)
median_error = np.median(errors)
abs_max_error = np.max(np.abs(errors))

print(f"Max error [mm]: {max_error:.2f}")
print(f"Min error [mm]: {min_error:.2f}")
print(f"Mean error [mm]: {mean_error:.2f}")
print(f"Standard deviation [mm]: {std_dev:.2f}")
print(f"Median error [mm]: {median_error:.2f}")
print(f"Abs max error [mm]: {abs_max_error:.2f}")

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



# # Create point cloud objects

#     pcd_plane_fit = [corner_top_left, corner_top_right, corner_bottom_left, corner_bottom_right]
#     pcd_plane_fit_points = o3d.utility.Vector3dVector(pcd_plane_fit)



#     # Define your mesh creation here, with correct triangle definition for a rectangle
#     np_vertices = np.array([corner_top_right, corner_top_left, corner_bottom_right, corner_bottom_left])
#     np_triangles = np.array([[0, 1, 2], [1, 2, 3]])  # Assuming these are the correct indices for your corners

#     mesh = o3d.geometry.TriangleMesh()
#     mesh.vertices = o3d.utility.Vector3dVector(np_vertices)
#     mesh.triangles = o3d.utility.Vector3iVector(np_triangles)


#     # Error computation
#     #calculates the perpendicular distance (error) of each point in a point cloud to a fitted plane,
#     # and then maps these error values onto an image
#     errors = []
#     error_image = np.zeros((height, width))
#     #loops through each point in the point cloud. Each point has coordinates x y z
#     for point in pcd_plane_o3d.points:
#         #calculates the perpendicular distance from the point to the plane defined based on the equation ax+by+cz+d=0
#         error = a * point[0] + b * point[1] + c * point[2] + d
#         errors.append(error)

#         ###Mapping Errors to an Image###
#         # ensures that the Z coordinate of the point is not zero
#         # to avoid division by zero when converting 3D points to 2D pixel coordinates using perspective projection formulas
#         if point[2] != 0:
#             #convert the 3D point coordinates X,Y,Z to 2D pixel coordinates using the intrinsic parameters of the depth camera
#             x_pixel = int((point[0] * FX_DEPTH / point[2]) + CX_DEPTH)
#             y_pixel = int((point[1] * FY_DEPTH / point[2]) + CY_DEPTH)
#             if 0 <= x_pixel < width and 0 <= y_pixel < height:
#                 error_image[y_pixel, x_pixel] = error * 1000  # Convert error from meters to mm

#     # Convert errors from meters to mm
#     errors = np.array(errors) * 1000


# ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###
# ### calculate the error matric values using numpy functions 
# ## EX: median_error = np.median(errors)

# ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###    ###


#     print(f"Max error [mm]: {max_error:.2f}")
#     print(f"Min error [mm]: {min_error:.2f}")
#     print(f"Mean error [mm]: {mean_error:.2f}")
#     print(f"Standard deviation [mm]: {std_dev:.2f}")
#     print(f"Median error [mm]: {median_error:.2f}")
#     print(f"Abs max error [mm]: {abs_max_error:.2f}")

