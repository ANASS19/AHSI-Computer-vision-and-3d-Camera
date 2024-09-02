from pyk4a import PyK4A, Config, ColorResolution, DepthMode
import cv2
import numpy as np

def capture_depth_image():
   # Initialize the device with a configuration
   config = Config(
       color_resolution=ColorResolution.OFF,  # Turn off color camera as we're only interested in depth
       depth_mode=DepthMode.WFOV_2X2BINNED,  # Set depth mode

       synchronized_images_only=False,  # Color camera is off, so no need to synchronize
   )
   k4a = PyK4A(config=config)
   k4a.start()

   # Capture a depth image
   capture = k4a.get_capture()
   if capture.depth is not None:
       depth_image = capture.depth
       # Convert depth image to a visual format
       depth_image = depth_image.astype(np.float32)
       # Normalization to 0-255 for visualization
       depth_image = (depth_image - depth_image.min()) / (depth_image.max() - depth_image.min()) * 255
       depth_image = depth_image.astype(np.uint8)
       # Display the depth image
       cv2.imwrite('C:/Users/Ibtissam/OneDrive/Bureau/Lab07/depth_8.jpg', depth_image)
       cv2.imshow('Depth Image', depth_image)

       cv2.waitKey(0)
       cv2.destroyAllWindows()
   else:
       print("Failed to capture depth image")

   k4a.stop()

if __name__ == "__main__":
   capture_depth_image()

