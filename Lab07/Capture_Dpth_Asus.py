import numpy as np
import cv2
from primesense import openni2

# Initialize OpenNI
openni2.initialize(r"C:\Program Files\OpenNI2\Redist")  # Adjust the path as necessary according to ur PC

# Open the device
dev = openni2.Device.open_any()

# Create and start the color stream
color_stream = dev.create_color_stream()
color_stream.start()

# Create and start the depth stream
depth_stream = dev.create_depth_stream()
depth_stream.start()
video_mode = depth_stream.get_video_mode()
video_mode.resolutionX = 640
video_mode.resolutionY = 480
video_mode.fps = 30
depth_stream.set_video_mode(video_mode)
def capture_color():
    # The read_frame() method fetches the latest available frame from the stream.
    frame = color_stream.read_frame()
    # retrieve the raw frame data, data consists of RGB values for each pixel,  stored as triplets in the buffer
    frame_data = frame.get_buffer_as_triplet()
    #raw frame data is converted into a NumPy array. This array uses 8-bit unsigned integers for each color channel
    color_img = np.frombuffer(frame_data, dtype=np.uint8)
    color_img.shape = (frame.height, frame.width, 3)
    color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)  # Convert from RGB to BGR for OpenCV
    return color_img

def capture_depth():
    frame = depth_stream.read_frame()
    #retrieves the raw depth data from the captured frame as a buffer of 16-bit unsigned integers. Each integer in this buffer represents the depth at a specific pixel in the frame
    frame_data = frame.get_buffer_as_uint16()
    depth_img = np.frombuffer(frame_data, dtype=np.uint16)
    depth_img.shape = (frame.height, frame.width)
    depth_img = np.asarray(depth_img, dtype=np.float32)  # depth image is converted to a floating-point for normalization
    cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)  # Normalize depth data : n. This step scales the depth values so that the smallest depth value in the image becomes 0, and the largest depth value becomes 1.
    return depth_img


    # Re-fetch the video mode to confirm changes
# Capture color and depth images

depth_image = capture_depth()

# Convert the normalized depth image to 8-bit for display
depth_image_8bit = (depth_image * 255).astype(np.uint8)



# Display the captured images
#cv2.imwrite('C:/Users/User/Desktop/lab3/Depth_Asus.jpg', depth_image_8bit)
cv2.imwrite('C:/Users/Ibtissam/OneDrive/Bureau/Lab07/Depth_Asus5.jpg', depth_image_8bit)

cv2.imshow("Depth Image", depth_image_8bit)
cv2.waitKey(0)

# Stop and close streams0
color_stream.stop()
depth_stream.stop()
openni2.unload()

cv2.destroyAllWindows()
