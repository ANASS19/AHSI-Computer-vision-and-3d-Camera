from primesense import openni2
import numpy as np
def get_xtion_intrinsics():
    openni2.initialize(r"C:\Program Files\OpenNI2\Redist")  # Adjust the path as necessary according to ur PC
    dev = openni2.Device.open_any()
    depth_stream = dev.create_depth_stream()
    depth_stream.start()

    video_mode = depth_stream.get_video_mode()
    print(f"Current depth resolution: {video_mode.resolutionX}x{video_mode.resolutionY}")

    # Set the desired resolution ( 320x240) or VGA (640x480)
    video_mode.resolutionX = 640
    video_mode.resolutionY = 480
    video_mode.fps = 30
    depth_stream.set_video_mode(video_mode)



    video_mode = depth_stream.get_video_mode()
    print(f"Updated depth resolution: {video_mode.resolutionX}x{video_mode.resolutionY}")
    resolution_x = depth_stream.get_video_mode().resolutionX
    resolution_y = depth_stream.get_video_mode().resolutionY
    fov_horizontal = depth_stream.get_horizontal_fov()  # Horizontal field of view in radians
    fov_vertical = depth_stream.get_vertical_fov()  # Vertical field of view in radians
    fx = resolution_x / (2 * np.tan(fov_horizontal / 2))
    fy = resolution_y / (2 * np.tan(fov_vertical / 2))
    cx = resolution_x / 2
    cy = resolution_y / 2
    print("Intrinsic Parameters:")
    print(f"fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}")
    depth_stream.stop()

if __name__ == "__main__":
    get_xtion_intrinsics()
