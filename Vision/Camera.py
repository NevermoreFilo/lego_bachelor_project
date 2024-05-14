#!/usr/bin/env python
# Librerie standard python
import time
import imageio
import pyrealsense2 as rs
import numpy as np


class Camera(object):
    def __init__(self):
        super(Camera, self).__init__()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8, 30)
        pipeline = rs.pipeline()
        pipeline.start(config)
        align = rs.align(rs.stream.color)

        self.pipeline = pipeline
        self.align = align

    def get_frame(self):
        unaligned_frames = self.pipeline.wait_for_frames()
        frames = self.align.process(unaligned_frames)
        color = frames.get_color_frame()
        color_image = np.asanyarray(color.get_data())
        color_image = color_image[..., ::-1]
        imageio.imwrite("/home/cross/progetto_tirocinio/lego_ws/src/lego/src/Vision/robot.png", color_image)
        return color_image
