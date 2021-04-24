# rosbag2video

    rosbag2video.py
    rosbag to video file conversion tool
    by Maximilian Laiacker 2020
    post@mlaiacker.de

    with contributions from Abel Gabor 2019, Bey Hao Yun 2021
    baquatelle@gmail.com, beyhy94@gmail.com

## install:

**ffmpeg** is needed and can be installed on Ubuntu with:

```bash
sudo apt install ffmpeg
```

**ROS2** and other stuff

```bash
sudo apt install python3-sensor-msgs python3-opencv ros-foxy-rosbag2-transport
```


## usage:
``` bash
ros2bag2video.py [--fps 25] [--rate 1.0] [-o outputfile] [-v] [-s] [-t topic] bagfile1

Converts image sequence(s) in ros bag file(s) to video file(s) with fixed frame rate using ffmpeg
ffmpeg needs to be installed!

--fps   Sets FPS value that is passed to ffmpeg
            Default is 25.
-h      Displays this help.
--ofile (-o) sets output file name.
        If no output file name (-o) is given the filename 'output.mp4' is used.
--rate  (-r) You may slow down or speed up the video.
        Default is 1.0, that keeps the original speed.
-s      Shows each and every image extracted from the rosbag file.
--topic (-t) Only the images from topic "topic" are used for the video output.
-v      Verbose messages are displayed.
```
## example output:
```bash
# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Run the script
./ros2bag2video.py --topic /camera/color/image_raw rosbag2_2020_10_09-16_34_25/


[rosbag2video] - started.
FPS (int) =  25
Rate (float) =  1.0
Topic (str) =  /camera/color/image_raw
Display Images (bool) =  False
Output File (str) =  output.mp4
Verbose (bool) =  False
bag_file =  ../rosbag2video/rosbag2_2020_10_09-16_34_25/
[INFO] [1619238536.959558227] [rosbag2_storage]: Opened database 'rosbag2_2020_10_09-16_34_25/rosbag2_2020_10_09-16_34_25_0.db3' for READ_ONLY.
[INFO] [1619238537.316067802] [rosbag2videos]: Image Received [1/28]
Writing file,  001.png
[INFO] [1619238537.329882567] [rosbag2videos]: Image Received [2/28]
Writing file,  002.png
[INFO] [1619238537.558884056] [rosbag2videos]: Image Received [3/28]
Writing file,  003.png
[INFO] [1619238538.058178168] [rosbag2videos]: Image Received [4/28]
...
Writing to output file, output.mp4

```
