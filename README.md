# **rosbag2video**

    rosbag2video.py
    rosbag to video file conversion tool
    by Maximilian Laiacker 2020
    post@mlaiacker.de

    with contributions from Abel Gabor 2019, Bey Hao Yun 2021
    baquatelle@gmail.com, beyhy94@gmail.com

## **Install**:

**ffmpeg** is needed and can be installed on **Ubuntu** with:

```bash
sudo apt install ffmpeg
```

**ROS2** and **other stuff**.

```bash
sudo apt install python3-sensor-msgs python3-opencv ros-foxy-rosbag2-transport
```


## **Usage**:
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
## **Example Output**:
```bash
# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Run the script
./ros2bag2video.py --topic /camera/color/image_raw rosbag2_2020_10_09-16_34_25/


FPS (int) =  25
Rate (float) =  1.0
Topic (str) =  /camera/color/image_raw
Output File (str) =  output.mp4
Verbose (bool) =  False
bag_file =  rosbag2_2020_10_09-16_34_25/
[INFO] [1619616391.700087224] [rosbag2_storage]: Opened database './rosbag2_2020_10_09-16_34_25/rosbag2_2020_10_09-16_34_25_0.db3' for READ_ONLY.
[INFO] [1619616392.055535299] [ros2bag2videos]: Image Received [1/28]
[INFO] [1619616392.077294702] [ros2bag2videos]: Image Received [2/28]
[INFO] [1619616392.299152538] [ros2bag2videos]: Image Received [3/28]
[INFO] [1619616392.792717810] [ros2bag2videos]: Image Received [4/28]
[INFO] [1619616393.268813798] [ros2bag2videos]: Image Received [5/28]
[INFO] [1619616393.724949415] [ros2bag2videos]: Image Received [6/28]
[INFO] [1619616394.199588269] [ros2bag2videos]: Image Received [7/28]
[INFO] [1619616394.667638765] [ros2bag2videos]: Image Received [8/28]
...
Writing to output file, output.mp4

```
