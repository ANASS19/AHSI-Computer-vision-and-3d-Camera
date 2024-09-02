import cv2
from pykinect_azure.k4a import _k4a
import pykinect_azure as pykinect
from pykinect_azure import K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH, k4a_float2_t

if __name__ == "__main__":

	# Initialize the library, if the library is not found, add the library path as argument
	pykinect.initialize_libraries()

	# Modify camera configuration
	device_config = pykinect.default_configuration
	device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
	device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
	device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
	# print(device_config)

	# Start device
	device = pykinect.start_device(config=device_config)
	mtx = device.calibration.get_matrix(camera=_k4a.K4A_CALIBRATION_TYPE_DEPTH)
	print("Parameters of Depth Camera:")
	print(mtx)
	mtxc = device.calibration.get_matrix(camera=_k4a.K4A_CALIBRATION_TYPE_COLOR)
	print("Parameters of Color Camera:")
	print(mtxc)
