import pyrealsense2 as rs
import numpy as np
import cv2


#Basic camera interface that can be extended to use different API-s. Realsense example below
class ICamera:
    def open(self):
        pass
    def close(self):
        pass
    def has_depth_capability(self) -> bool:
        pass
    def get_color_frame(self):
        pass
    def get_frames(self):
        pass


# Camera implementation using the pyrealsense2 provided API 
class RealsenseCamera(ICamera):
    def __init__(self, config):
        conf_dict = config.get_module_dict("camera")

        self.rgb_width = conf_dict["rgb_width"]
        self.rgb_height = conf_dict["rgb_height"]
        self.rgb_framerate = conf_dict["rgb_framerate"]
        self.exposure = conf_dict["exposure"]
        self.white_balace = conf_dict["white_balace"]

        self.depth_width = conf_dict["depth_width"]
        self.depth_height = conf_dict["depth_height"]
        self.depth_framerate = conf_dict["depth_framerate"]

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.rgb_width, self.rgb_height, rs.format.bgr8, self.rgb_framerate)
        
        self.depth_enabled = conf_dict["depth_enabled"]

        if self.depth_enabled:
            self.config.enable_stream(rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16, self.depth_framerate)
            
        self.align = rs.align(rs.stream.color)
        self.depth_scale = -1

    def open(self):
        profile = self.pipeline.start(self.config)
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.enable_auto_exposure, False)
        color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        color_sensor.set_option(rs.option.white_balance, self.white_balace)
        color_sensor.set_option(rs.option.exposure, self.exposure)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def close(self):
        self.pipeline.stop()
    
    def get_color_frame(self):
        frames = self.pipeline.wait_for_frames()
        return np.asanyarray(frames.get_color_frame().get_data())
    
    def has_depth_capability(self) -> bool:
        return self.depth_enabled

    def get_frames(self, aligned = False):
        frames = self.pipeline.wait_for_frames()
        if aligned:
            frames = self.align.process(frames)
            return np.asanyarray(frames.get_color_frame().get_data()), np.asanyarray(frames.get_depth_frame().get_data())
        else:
            return np.asanyarray(frames.get_color_frame().get_data()), None
