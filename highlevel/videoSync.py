#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# VideoRecorder.py

from __future__ import print_function, division
import numpy as np
import sys
import cv2
import threading
import time
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

WIDTH = 1280
HEIGHT = 720
WIDTH_2K = 2560
HEIGHT_2K = 1440

class Recorder(Node):
    def __init__(self, filename, running_scenario, scale_size=1):
        super().__init__('video_recorder')
        self.filename = filename
        REC_FOLDER = "experiment_video/" + running_scenario + "/camera_1/"
        if not os.path.exists(os.path.dirname(REC_FOLDER + filename)):
            os.makedirs(os.path.dirname(REC_FOLDER + filename))
        self.video_thread = self.VideoRecorder(self, REC_FOLDER + filename, scale_size)

        self.subscription = self.create_subscription(
            Bool,
            'travelerleg_start',
            self.start_stop_callback,
            10
        )
        self.is_recording = False

    def start_stop_callback(self, msg):
        if msg.data and not self.is_recording:
            self.startRecording()
        elif not msg.data and self.is_recording:
            self.stopRecording()
            self.saveRecording()

    def startRecording(self):
        self.video_thread.start()
        self.is_recording = True

    def stopRecording(self):
        self.video_thread.stop()
        self.is_recording = False

    def saveRecording(self):
        self.video_thread.showFramesResume()


    class VideoRecorder():
        "Video class based on openCV"

        def __init__(self, recorder, name, scale_size, fourcc="MJPG", frameSize=(1920, 1090), camindex=0, fps=30):
            self.recorder = recorder
            self.open = True
            self.duration = 0
            self.device_index = camindex
            self.fps = fps                          # fps should be the minimum constant rate at which the camera can
            self.fourcc = fourcc                    # capture images (with no decrease in speed over time; testing is required)
            self.video_filename = name + ".avi"     # video formats and sizes also depend and vary according to the camera used
            self.video_cap = cv2.VideoCapture("/dev/video0")
            self.frame_size = frameSize
            self.scale = scale_size

            self.video_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH*self.scale)
            self.video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT*self.scale)
            self.video_writer = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
            self.video_out = cv2.VideoWriter(self.video_filename, self.video_writer, self.fps, self.frame_size)
            self.frame_counts = 1
            self.start_time = time.time()

            self.openCvVidCapIds = []

        def record(self):
            width = int(self.video_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.video_cap.get(cv2.CAP_PROP_FPS)

            "Video starts being recorded"
            counter = 1
            while self.open:
                ret, video_frame = self.video_cap.read()
                if ret:
                    cv2.imshow('frame', video_frame)

                    # scale the frame
                    dim = tuple([int(1 * x) for x in self.frame_size])
                    frame_resized = cv2.resize(video_frame, dim)

                    self.video_out.write(frame_resized)
                    self.frame_counts += 1
                    counter += 1
                    self.duration += 1/self.fps
                    cv2.waitKey(1)
                else:
                    break

            #Release Video
            self.video_out.release()
            self.video_cap.release()
            cv2.destroyAllWindows()
            self.video_out = None

        def stop(self):
            "Finishes the video recording therefore the thread too"
            self.open=False

        def start(self):
            "Launches the video recording function using a thread"
            self.thread = threading.Thread(target=self.record)
            self.thread.start()

        def showFramesResume(self):
            frame_counts = self.frame_counts
            elapsed_time = time.time() - self.start_time
            recorded_fps = self.frame_counts / elapsed_time

if __name__ == '__main__':
    rclpy.init()
    recorder = Recorder("test1", "MiniRhex")
    rclpy.spin(recorder)
    rclpy.shutdown()
