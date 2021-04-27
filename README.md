# **rosbag2video**

    rosbag2video.py
    rosbag to video file conversion tool
    by Maximilian Laiacker 2020
    post@mlaiacker.de

    with contributions from Abel Gabor 2019
    baquatelle@gmail.com

For use with **ROS2** bags, please refer to the [foxy](https://github.com/mlaiacker/rosbag2video/tree/foxy) branch.

For use with **ROS1** bags, please proceed with the instructions below.

## **Install**:

**ffmpeg** is needed and can be installed on **Ubuntu** with:

    sudo apt install ffmpeg

**ros** and **other stuff**

    sudo apt install python3-roslib python3-sensor-msgs python3-opencv



## **Usage**:

    rosbag2video.py [--fps 25] [--rate 1] [-o outputfile] [-v] [-s] [-t topic] bagfile1 [bagfile2] ...

    Converts image sequence(s) in ros bag file(s) to video file(s) with fixed frame rate using ffmpeg
    ffmpeg needs to be installed!

    --fps   Sets FPS value that is passed to ffmpeg
            Default is 25.
    -h      Displays this help.
    --ofile (-o) sets output file name.
            If no output file name (-o) is given the filename '<prefix><topic>.mp4' is used and default output codec is h264.
            Multiple image topics are supported only when -o option is _not_ used.
            ffmpeg  will guess the format according to given extension.
            Compressed and raw image messages are supported with mono8 and bgr8/rgb8/bggr8/rggb8 formats.
    --rate  (-r) You may slow down or speed up the video.
            Default is 1.0, that keeps the original speed.
    -s      Shows each and every image extracted from the rosbag file (cv_bride is needed).
    --topic (-t) Only the images from topic "topic" are used for the video output.
    -v      Verbose messages are displayed.
    --prefix (-p) set a output file name prefix othervise 'bagfile1' is used (if -o is not set).
    --start Optional start time in seconds.
    --end   Optional end time in seconds.

## **Example Output**:

    ./rosbag2video.py camera_and_state.bag

    rosbag2video, by Maximilian Laiacker 2020 and Abel Gabor 2019

    ############# COMPRESSED IMAGE  ######################
    /image_raw/compressed  with datatype: sensor_msgs/CompressedImage

    frame=   77 fps= 13 q=28.0 size=    1280kB time=00:00:00.96 bitrate=10922.2kbits/s speed=0.156x
