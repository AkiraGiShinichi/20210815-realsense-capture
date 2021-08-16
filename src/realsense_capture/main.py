"""
This is a skeleton file that can serve as a starting point for a Python
console script. To run this script uncomment the following lines in the
``[options.entry_points]`` section in ``setup.cfg``::

    console_scripts =
         fibonacci = realsense_capture.skeleton:run

Then run ``pip install .`` (or ``pip install -e .`` for editable mode)
which will install the command ``fibonacci`` inside your current environment.

Besides console scripts, the header (i.e. until ``_logger``...) of this file can
also be used as template for Python modules.

Note:
    This skeleton file can be safely removed if not needed!

References:
    - https://setuptools.readthedocs.io/en/latest/userguide/entry_point.html
    - https://pip.pypa.io/en/stable/reference/pip_install
"""

import argparse
import logging
import sys

import pyrealsense2 as rs
from typing import List, Tuple
import numpy as np
from enum import Enum

from realsense_capture import __version__

__author__ = "akiragishinichi"
__copyright__ = "akiragishinichi"
__license__ = "MIT"

_logger = logging.getLogger(__name__)


# ---- Python API ----
# The functions defined in this section can be imported by users in their
# Python scripts/interactive interpreter, e.g. via
# `from realsense_capture.skeleton import fib`,
# when using this Python module as a library.


def fib(n):
    """Fibonacci example function

    Args:
      n (int): integer

    Returns:
      int: n-th Fibonacci number
    """
    assert n > 0
    a, b = 1, 1
    for i in range(n - 1):
        a, b = b, a + b
    return a

# ----------------------------- Helper functions ----------------------------- #


class Device:
    """Store information of device (pipeline, pipeline-profile, align, product-line)

    Args:
        pipeline (rs.pipeline): Pipeline
        pipeline_profile (rs.pipeline_profile): Pipeline-profile
        align (rs.align): Alignment
        product_line (str): Product-line
    """

    def __init__(self, pipeline: rs.pipeline,
                 pipeline_profile: rs.pipeline_profile, align: rs.align,
                 product_line: str):
        self.pipeline = pipeline
        self.pipeline_profile = pipeline_profile
        self.align = align
        self.product_line = product_line


def enumerate_connected_devices(context: rs.context) -> List[Tuple[str, str]]:
    """Enumerate the connected Intel Realsense devices

    Args:
        context (rs.context): The context created for using the realsense library

    Returns:
        List[Tuple[str, str]]: List of (serial-number, product-line) of devices which are connected to the PC
    """
    connect_device = []

    for d in context.devices:
        if d.get_info(rs.camera_info.name).lower() != 'platform camera':
            device_serial = d.get_info(rs.camera_info.serial_number)
            product_line = d.get_info(rs.camera_info.product_line)
            device_info = (device_serial, product_line)
            connect_device.append(device_info)
    return connect_device

# TODO: What are good values to filter?


def post_process_depth_frame(
        depth_frame, decimation_magnitude=1.0, spatial_magnitude=2.0,
        spatial_smooth_alpha=0.5, spatial_smooth_delta=20,
        temporal_smooth_alpha=0.4, temporal_smooth_delta=20):
    """
    Filter the depth frame acquired using the Intel RealSense device
    Parameters:
    -----------
    depth_frame          : rs.frame()
                           The depth frame to be post-processed
    decimation_magnitude : double
                           The magnitude of the decimation filter
    spatial_magnitude    : double
                           The magnitude of the spatial filter
    spatial_smooth_alpha : double
                           The alpha value for spatial filter based smoothening
    spatial_smooth_delta : double
                           The delta value for spatial filter based smoothening
    temporal_smooth_alpha: double
                           The alpha value for temporal filter based smoothening
    temporal_smooth_delta: double
                           The delta value for temporal filter based smoothening
    Return:
    ----------
    filtered_frame : rs.frame()
                     The post-processed depth frame
    """
    # Post processing possible only on the depth_frame
    assert (depth_frame.is_depth_frame())

    # Available filters and control options for the filters
    decimation_filter = rs.decimation_filter()
    spatial_filter = rs.spatial_filter()
    temporal_filter = rs.temporal_filter()

    filter_magnitude = rs.option.filter_magnitude
    filter_smooth_alpha = rs.option.filter_smooth_alpha
    filter_smooth_delta = rs.option.filter_smooth_delta

    # Apply the control parameters for the filter
    decimation_filter.set_option(filter_magnitude, decimation_magnitude)
    spatial_filter.set_option(filter_magnitude, spatial_magnitude)
    spatial_filter.set_option(filter_smooth_alpha, spatial_smooth_alpha)
    spatial_filter.set_option(filter_smooth_delta, spatial_smooth_delta)
    temporal_filter.set_option(filter_smooth_alpha, temporal_smooth_alpha)
    temporal_filter.set_option(filter_smooth_delta, temporal_smooth_delta)

    # Apply the filters
    filtered_frame = decimation_filter.process(depth_frame)
    filtered_frame = spatial_filter.process(filtered_frame)
    filtered_frame = temporal_filter.process(filtered_frame)

    return filtered_frame


class SingleInstanceMetaClass(type):
    def __call__(cls, *args, **kwargs):
        try:
            return cls.__instance
        except AttributeError:
            cls.__instance = super(
                SingleInstanceMetaClass, cls).__call__(
                *args, **kwargs)
            return cls.__instance


def get_depth_at_pixel(depth_frame: rs.frame,
                       pixel_x: int, pixel_y: int) -> int:
    """Get the depth value at the desired image point

    Args:
        depth_frame (rs.frame): The depth frame containing the depth information of the image coordinate
        pixel_x (int): The x value of the image coordinate
        pixel_y (int): The y value of the image coordinate

    Returns:
        int: Depth value at the desired pixel
    """
    return depth_frame.as_depth_frame().get_distance(round(pixel_x), round(pixel_y))


def convert_depth_pixel_to_metric_coordinate(depth: float, pixel_x: float, pixel_y: float,
                                             camera_intrinsics: rs.intrinsics) -> Tuple[float, float, float]:
    """Convert the depth and image point information to metric coordinates

    Args:
        depth ([float]): The depth value of the image point
        pixel_x (float): The x value of the image coordinate
        pixel_y (float): The y value of the image coordinate
        camera_intrinsics (rs.intrinsics): The intrinsic values of the imager in whose coordinate system the depth_frame is computed

    Returns:
        (X, Y, Z) (Tuple[float, float, float]): Coordinate of pixel
        X (float): The x coordinate value in meters
        Y (float): The y coordinate value in meters
        Z (float): The z coordinate value in meters
    """
    X = (pixel_x - camera_intrinsics.ppx) / camera_intrinsics.fx * depth
    Y = (pixel_y - camera_intrinsics.ppy) / camera_intrinsics.fy * depth
    Z = depth
    return X, Y, Z


def convert_depth_frame_to_points(depth_image: np.ndarray,
                                  camera_intrinsics: rs.intrinsics,
                                  depth_scale: float = 0.001) -> Tuple[np.ndarray]:
    """Convert depth frame to a 3D point cloud

    Args:
        depth_image (np.ndarray): Depth image
        camera_intrinsics (rs.intrinsics): Camera intrinsics
        depth_scale (float, optional): Scale factor of depth. Defaults to 0.001.

    Returns:
        (x, y, z) (Tuple[np.ndarray]): 3 list of x coordinates, y coordinates and z coordinates
        x (np.ndarray): x coordinates in meters
        y (np.ndarray): y coordinates in meters
        z (np.ndarray): z coordinates in meters
    """
    height, width = depth_image.shape

    nx = np.linspace(0, width - 1, width)
    ny = np.linspace(0, height - 1, height)
    u, v = np.meshgrid(nx, ny)

    x = (u.flatten() - camera_intrinsics.ppx) / camera_intrinsics.fx
    y = (v.flatten() - camera_intrinsics.ppy) / camera_intrinsics.fy

    z = depth_image.flatten() * depth_scale
    x = np.multiply(x, z)
    y = np.multiply(y, z)

    return x, y, z


def convert_pointcloud_to_depth(pointcloud, camera_intrinsics):
    """Convert the world coordinate to a 2D image coordinate

    :param pointcloud: numpy array with shape 3xN
    :type pointcloud: numpy array with shape 3xN
    :param camera_intrinsics: [description]
    :type camera_intrinsics: [type]
    :return: (x, y)
    :x: The x coordinates in image
    :y: The y coordiantes in image
    :rtype: tuple(array, array)
    """
    assert (pointcloud.shape[0] == 3)

    x_ = pointcloud[0, :]
    y_ = pointcloud[1, :]
    z_ = pointcloud[2, :]

    m = x_[np.nonzero(z_)] / z_[np.nonzero(z_)]
    n = y_[np.nonzero(z_)] / z_[np.nonzero(z_)]

    x = m * camera_intrinsics.fx + camera_intrinsics.ppx
    y = n * camera_intrinsics.fy + camera_intrinsics.ppy

    return x, y


def get_boundary_corners_2D(points):
    pass


def get_clipped_pointcloud(pointcloud, boundary):
    pass
# ---------------------------------------------------------------------------- #


# ------------------------------- Main content ------------------------------- #
class DataType(Enum):
    FRAMES = 1
    COLOR_FRAME = 2
    DEPTH_FRAME = 3
    COLOR_IMAGE = 4
    DEPTH_IMAGE = 5
    IMAGES = 6


class RealsenseCapture(metaclass=SingleInstanceMetaClass):
    """Class to manage the Intel Realsense capture.

    Args:
        id (int, optional): Id of connected Realsense device. Defaults to 0.
        color_size (Tuple, optional): Size of color frame. Defaults to (640, 480).
        depth_size (Tuple, optional): Size of depth frame. Defaults to (640, 480).
        fps (int, optional): FPS of capture. Defaults to 30.
        serial (str, optional): Serial-number of desired device. Defaults to None.
    """

    def __init__(
            self, id: int = 0,
            color_size: Tuple[int, int] = (640, 480),
            depth_size: Tuple[int, int] = (640, 480),
            fps: int = 30, serial: str = None) -> None:  #
        self._depth_size = depth_size
        self._color_size = color_size
        self._fps = fps

        self._context = rs.context()
        self._available_devices = enumerate_connected_devices(self._context)
        self._device_id = id
        if serial is not None:
            self._serial = serial
            self._device_id = self.get_device_id_from_serial(self._serial)
        self._device_serial, self._product_line = self.get_device_info_from_id(
            self._device_id)

        self._enabled_device = None

        color_width, color_height = self._color_size
        depth_width, depth_height = self._depth_size
        self._config = rs.config()
        self._config.enable_stream(
            rs.stream.color, color_width, color_height, rs.format.rgb8, fps)
        self._config.enable_stream(
            rs.stream.depth, depth_width, depth_height, rs.format.z16, fps)

        self._camera_is_open = False
        self._frames = None

    def enable_device(self, enable_ir_emitter: bool = False):
        """Enable an Intel Realsense device
        Or providing exact device-serial, or providing device-id for convenience

        Args:
            enable_ir_emitter (bool, optional): Enable/Disable the IR-Emitter of the device. Defaults to False.

        Examples:
            realsense_capture.enable_device(0) # 1st method
            realsense_capture.enable_device(device_serial='f12345') # 2nd method
        """
        try:
            pipeline = rs.pipeline()

            self._config.enable_device(self._device_serial)
            pipeline_profile = pipeline.start(self._config)

            # Set the acquisition parameters
            sensor = pipeline_profile.get_device().first_depth_sensor()
            if sensor.supports(rs.option.emitter_enabled):
                sensor.set_option(rs.option.emitter_enabled, 1
                                  if enable_ir_emitter else 0)
            # Create an align object
            # rs.align allows us to perform alignment of depth frames to others frames
            # The "align_to" is the stream type to which we plan to align depth frames.
            align_to = rs.stream.color
            align = rs.align(align_to)

            self._enabled_device = Device(
                pipeline, pipeline_profile, align, self._product_line)

            self._camera_is_open = True
            print(f'\n    RealsenseCapture - initialized')
        except:
            print(f'\n    RealsenseCapture - initialized not success')

    def warm_up(self, dispose_frames_for_stablisation: int = 30) -> None:
        """Dispose some frames for camera-stablisation

        Args:
            dispose_frames_for_stablisation (int, optional): Number of disposing frames. Defaults to 30.
        """
        for _ in range(dispose_frames_for_stablisation):
            _ = self.read()

    def read(self, return_depth=False, depth_filter=None):
        """Read data from camera

        :param return_depth: Whether return depth image or not, defaults to False
        :type return_depth: bool, optional
        :param depth_filter: [description], defaults to None
        :type depth_filter: [type], optional
        :return: Whether having data, and data
        :rtype: tuple(bool, array or list of array)
        """
        try:
            frames = self._enabled_device.pipeline.wait_for_frames()
            # Align the depth frame to color frame
            self._frames = self._enabled_device.align.process(frames)

            if return_depth:  # Return RGB image and Depth image
                return True, self.get_data_according_type(
                    DataType.IMAGES, depth_filter)
            else:  # Return RGB image only
                return True, self.get_data_according_type(DataType.COLOR_IMAGE)
        except:
            self._camera_is_open = False
            print(f'\n    RealsenseCapture - read: error')
            return False, None

    def isOpened(self):
        """Check whether the camera is open(ready to use)

        :return: Is open or not
        :rtype: bool
        """
        return self._camera_is_open

    def release(self):
        """Release/Disable cameras
        """
        print(f'\n    RealsenseCapture - release')
        self._config.disable_all_streams()

    def get_intrinsics(self, frame_type: DataType = DataType.COLOR_FRAME):
        """Get intrinsics of a frame(depth ? color)
        :In this case, after alignment, intrinsics of depth and color frames
        :are the same

        :param frame_type: Type of frame, defaults to DataType.COLOR_FRAME
        :type frame_type: DataType, optional
        :return: intrinsics
        :rtype: rs.intrinsics
        """
        assert frame_type == DataType.COLOR_FRAME or frame_type == DataType.DEPTH_FRAME

        if frame_type == DataType.COLOR_FRAME:
            frame = self.get_data_according_type(DataType.COLOR_FRAME)
        elif frame_type == DataType.DEPTH_FRAME:
            frame = self.get_data_according_type(DataType.DEPTH_FRAME)

        if frame is None:
            return None

        intrinsics = frame.get_profile().as_video_stream_profile().get_intrinsics()
        return intrinsics

    def get_depth_scale(self) -> float:
        """Get depth-scale of the connected device

        Returns:
            float: Depth-scale
        """
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = self._enabled_device.pipeline_profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        return depth_scale

    def get_depth_to_color_extrinsics(self):
        """Get extrinsics from depth frame to color frame

        :return: Extrinsics
        :rtype: rs.extrinsics
        """
        color_frame = self.get_data_according_type(DataType.COLOR_FRAME)
        depth_frame = self.get_data_according_type(DataType.DEPTH_FRAME)

        if color_frame is None or depth_frame is None:
            return None

        extrinsics = depth_frame.get_profile().as_video_stream_profile(
        ).get_extrinsics_to(color_frame.get_profile())
        return extrinsics

    def get_depth_frame(self, depth_filter: object = None):
        """Get depth frame

        Args:
            depth_filter (object, optional): Function to filter depth frame. Defaults to None.

        Returns:
            rs.depth_frame: Depth frame after filtered.
        """
        if self._frames is None:
            return None

        depth_frame = self._frames.get_depth_frame()
        if depth_filter is not None:
            depth_frame = depth_filter(depth_frame)
        return depth_frame

    def get_data_according_type(self, data_type: DataType = DataType.FRAMES,
                                depth_filter=None):
        """Get data according to type

        Args:
            data_type (DataType, optional): Expected type of data. Defaults to DataType.FRAMES.
            depth_filter ([type], optional): Function to filter depth frame. Defaults to None.

        Returns:
            rs.frame|ndarray|Tuple(ndarray): Data
        """
        if self._frames is None:
            return None

        if data_type == DataType.FRAMES:
            return self._frames
        elif data_type == DataType.COLOR_FRAME:
            return self._frames.get_color_frame()
        elif data_type == DataType.DEPTH_FRAME:
            return self.get_depth_frame(depth_filter)
        elif data_type == DataType.COLOR_IMAGE:
            return np.asarray(self._frames.get_color_frame().get_data())
        elif data_type == DataType.DEPTH_IMAGE:
            return np.asarray(self.get_depth_frame(depth_filter).get_data())
        elif data_type == DataType.IMAGES:
            color_image = np.asarray(self._frames.get_color_frame().get_data())
            depth_image = np.asarray(self.get_depth_frame(depth_filter)
                                     .get_data())
            return (color_image, depth_image)

    def get_device_id_from_serial(self, serial: str) -> int:
        """Get device Id from desired serial-number

        Args:
            serial (str): Serial-number of Realsense device.

        Returns:
            int: Id of device corresponding to serial-number.
                 If device not found, return -1
        """
        assert self._available_devices > 0, "No device found."

        for i, device_info in self._available_devices:
            if serial in device_info:
                return i

        print(f"Device serial {serial} not found")
        return -1

    def get_device_info_from_id(self, id: int = 0) -> Tuple[str, str]:
        """Get device information from desired Id

        Args:
            id (int, optional): Desired Id. Defaults to 0.

        Returns:
            Tuple[str, str]: Serial-number and Product-line
        """
        assert len(self._available_devices) > 0, "No device found."

        if id < 0 or id > len(self._available_devices):
            print("Device id is out of available range.")
            return None, None
        else:
            return self._available_devices[id]
# ---------------------------------------------------------------------------- #


class Observation(Enum):
    COLOR = 1
    DEPTH = 2


def to_pick_out(arrays, conditions):
    assert isinstance(arrays, tuple), 'Not be tuple of arrays'
    return [array[conditions] for array in arrays]

# ---- CLI ----
# The functions defined in this section are wrappers around the main Python
# API allowing them to be called directly from the terminal as a CLI
# executable/script.


def parse_args(args):
    """Parse command line parameters

    Args:
      args (List[str]): command line parameters as list of strings
          (for example  ``["--help"]``).

    Returns:
      :obj:`argparse.Namespace`: command line parameters namespace
    """
    parser = argparse.ArgumentParser(
        description="Just a Fibonacci demonstration")
    parser.add_argument(
        "--version",
        action="version",
        version="realsense-capture {ver}".format(ver=__version__),
    )
    # parser.add_argument(dest="n", help="n-th Fibonacci number",
    #                     type=int, metavar="INT")
    parser.add_argument(
        "-v",
        "--verbose",
        dest="loglevel",
        help="set loglevel to INFO",
        action="store_const",
        const=logging.INFO,
    )
    parser.add_argument(
        "-vv",
        "--very-verbose",
        dest="loglevel",
        help="set loglevel to DEBUG",
        action="store_const",
        const=logging.DEBUG,
    )
    return parser.parse_args(args)


def setup_logging(loglevel):
    """Setup basic logging

    Args:
      loglevel (int): minimum loglevel for emitting messages
    """
    logformat = "[%(asctime)s] %(levelname)s:%(name)s:%(message)s"
    logging.basicConfig(
        level=loglevel, stream=sys.stdout, format=logformat,
        datefmt="%Y-%m-%d %H:%M:%S")


def main(args):
    """Wrapper allowing :func:`fib` to be called with string arguments in a CLI fashion

    Instead of returning the value from :func:`fib`, it prints the result to the
    ``stdout`` in a nicely formatted message.

    Args:
      args (List[str]): command line parameters as list of strings
          (for example  ``["--verbose", "42"]``).
    """
    args = parse_args(args)
    setup_logging(args.loglevel)
    _logger.debug("Starting Realsense capture...")
    # print("The {}-th Fibonacci number is {}".format(args.n, fib(args.n)))

    # Initialize capture
    realsense_capture = RealsenseCapture(
        id=0,
        depth_size=(640, 480),
        color_size=(640, 480),
        fps=30)  # L515
    realsense_capture.enable_device()
    realsense_capture.warm_up()

    _logger.info("Script ends here")


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    # ^  This is a guard statement that will prevent the following code from
    #    being executed in the case someone imports this file instead of
    #    executing it as a script.
    #    https://docs.python.org/3/library/__main__.html

    # After installing your project with pip, users can also run your Python
    # modules as scripts via the ``-m`` flag, as defined in PEP 338::
    #
    #     python -m realsense_capture.skeleton 42
    #
    run()
