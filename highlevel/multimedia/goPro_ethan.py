from open_gopro import WiredGoPro
from open_gopro import Params
from open_gopro.util import setup_logging
from pathlib import Path


def startRecord(serial_name):

    gopro = WiredGoPro(str(serial_name)) #opens gopro with serial number ending in 403
    gopro.open()


    #sets the FPS to 240 fps
    gopro.http_setting.fps.set(Params.FPS.FPS_240)
    #sets the resolution to 1080p
    gopro.http_setting.resolution.set(Params.Resolution.RES_1080)


    #sets the video mode to highest quality
    gopro.http_setting.system_video_mode.set(Params.SystemVideoMode.HIGHEST_QUALITY) 

    #sets the video FOV to linear, to minimize fisheye
    gopro.http_setting.video_field_of_view.set(Params.VideoFOV.LINEAR)

    # logger.info("LOGGING")

    #Ensures we are in video mode 
    gopro.http_command.load_preset_group(group=Params.PresetGroup.VIDEO).is_ok

    #now we want to start recording
    gopro.http_command.set_shutter(shutter=Params.Toggle.ENABLE).is_ok

    #we dont close because we wanna use the same port later 
    # #close 
    #gopro.close()
    return gopro
def endRecord(serial_name,file_path,gopro):

    #we dont open cuz we are re-using the same port as before 



    #now we want to end recording
    gopro.http_command.set_shutter(shutter=Params.Toggle.DISABLE).is_ok
    #close connection
    # gopro.close()


    # #re-opens connection so we can mainpulate other things 
    # gopro = WiredGoPro(str(serial_name)) #opens gopro with serial number ending in 403
    # gopro.open()

    #gets media after ending video
    media_set_after = set(x["n"] for x in gopro.http_command.get_media_list().flatten)
    #we now need to sort this to get the files named GX01 at the beginning then sort 
    #highest to lowest to get the most recent video 
    #now must clip file names and sort in ascending order 


    #sets our substring to test
    substring_test = "GX01"
    #get the video files 
    video_files = [i for i in media_set_after if substring_test in i]

    #removes beginning and end phrase
    renamed_files = []
    for i in range(len(video_files)):
        #gets current file
        curr_file = video_files[i]
        #removes the beginning
        begin_removed = curr_file[3:]
        #removes end 
        full_removed = begin_removed[:len(begin_removed)-4]
        #appends
        renamed_files.append(full_removed)

    #now to sort high to low 
    renamed_files.sort(reverse=True)

    #now to get full file name that we want to save

    save_name = substring_test[0:3] + renamed_files[0] + ".MP4"

    #now to get the file saved 

    gopro.http_command.download_file(camera_file=save_name,local_file=file_path)

    #closes the gopro path
    gopro.close()

    #need to reset usb for nexy run 
    #/dev/bus/usb/001/002