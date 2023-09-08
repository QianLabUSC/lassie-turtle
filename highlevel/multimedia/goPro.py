import time
import argparse
from pathlib import Path
from typing import Optional

# from rich.console import Console

from open_gopro import WiredGoPro, Params
from open_gopro.util import setup_logging, add_cli_args_and_parse


# console = Console()


class GoProCamera() :
    def __init__(self, serial_name):
        self.gopro = WiredGoPro(str(serial_name))
        self.gopro.open()
        
        # Configure settings to prepare for video
        # if self.gopro.is_encoding:
        #     self.gopro.http_command.set_shutter(shutter=Params.Toggle.DISABLE)
        # self.gopro.http_setting.video_performance_mode.set(Params.PerformanceMode.MAX_PERFORMANCE)
        # self.gopro.http_setting.max_lens_mode.set(Params.MaxLensMode.DEFAULT)
        # self.gopro.http_setting.camera_ux_mode.set(Params.CameraUxMode.PRO)
        # self.gopro.http_command.set_turbo_mode(mode=Params.Toggle.DISABLE)
        # #sets the FPS to 240 fps
        # self.gopro.http_setting.fps.set(Params.FPS.FPS_240)
        # #sets the resolution to 1080p
        # self.gopro.http_setting.resolution.set(Params.Resolution.RES_1080)

        # #sets the video FOV to linear, to minimize fisheye
        # self.gopro.http_setting.video_field_of_view.set(Params.VideoFOV.LINEAR)

        # assert self.gopro.http_command.load_preset_group(group=Params.PresetGroup.VIDEO).is_ok

        self.media_set_before = set(x["n"] for x in self.gopro.http_command.get_media_list().flatten)
        self.media_set_after = set(x["n"] for x in self.gopro.http_command.get_media_list().flatten)
        # console.print("Camera initialized.")

    def startVideo(self) :
        print("starting to record...")
        self.media_set_before = set(x["n"] for x in self.gopro.http_command.get_media_list().flatten)
        assert self.gopro.http_command.set_shutter(shutter=Params.Toggle.ENABLE).is_ok

    def stopVideo(self) :
        print("stopping recording...")
        assert self.gopro.http_command.set_shutter(shutter=Params.Toggle.DISABLE).is_ok
        self.media_set_after = set(x["n"] for x in self.gopro.http_command.get_media_list().flatten)

    def downloadVideo(self, filename = "video.mp4") :
        print("downloading video file with name ", filename)
        video = self.media_set_after.difference(self.media_set_before).pop()
        # console.print("Downloading the video...")
        self.gopro.http_command.download_file(camera_file=video, local_file=filename)

    def closeCamera(self) :
        self.gopro.close()

    # add keep alive function


def interactiveCameraTest() :
    goPro = GoProCamera()
    if (input("press key to start video.")) :
        goPro.startVideo()
    
    if (input("press key to end video.")) :
        goPro.stopVideo()
    
    filename = input("enter filename to download video")
    goPro.downloadVideo(filename)
    
    goPro.closeCamera()


def autoCameraTest() :
    goPro = GoProCamera("996")
    
    goPro.startVideo()
    
    time.sleep(4)
    
    goPro.stopVideo()
    
    filename = input("enter filename to download video:")
    filename = filename+".mp4"
    goPro.downloadVideo(filename)
    
    goPro.closeCamera()


if __name__ == '__main__':
    # interactiveCameraTest()

    autoCameraTest()