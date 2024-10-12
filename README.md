# **rosbag2video**

```text
rosbag2video.py
rosbag to video file conversion tool
by Maximilian Laiacker 2020
post@mlaiacker.de

with contributions from
Abel Gabor 2019,
Bey Hao Yun 2024
baquatelle@gmail.com,
beyhy94@gmail.com
a.j.blight@leeds.ac.uk
```

## **Install**

Build docker images using the commands below:

1. Download `rosbag2video`:

```bash
cd $HOME
```

```bash
git clone https://github.com/mlaiacker/rosbag2video --depth 1 --branch sqlite3_extraction_ros2 --single-branch && cd rosbag2video
```

2. Build docker image for running ROS 1 `rosbag2video.py`:

```bash
docker build -f Dockerfile.ros1 -t ros2bagvideo:noetic .
```

3. Build docker image for running ROS 2 `ros2bag2video.py`:

```bash
docker build -f Dockerfile.ros2 -t ros2bagvideo:humble .
```

## **Usage**

For processing ROS 1 bag files, please use `rosbag2video.py`:

``` bash
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
```

For processing ROS 2 bag folders, please use `ros2bag2video.py`:

``` bash
ros2bag2video.py [--rate 1.0]  [-t topic] [-i bagfolder1] [-o outputfile] [-v] [-s]

Converts image sequence(s) in ros bag file(s) to video file(s) with fixed frame rate using ffmpeg
ffmpeg needs to be installed!

--rate  (-r) Sets FPS value that is passed to ffmpeg
            Default is 25.
-h      Displays this help.
--ofile (-o) sets output file name.
        If no output file name (-o) is given the filename 'output.mp4' is used.
--ifile (-i) sets input file name.
--rate  (-r) You may slow down or speed up the video.
        Default is 1.0, that keeps the original speed.
-s      Shows each and every image extracted from the rosbag file.
--topic (-t) Only the images from topic "topic" are used for the video output.
-v      Verbose messages are displayed.
```

## **Run**

For running ROS 1 `rosbag2video.py`:

```bash
source /opt/ros/noetic/setup.bash
```

```bash
python3 rosbag2video.py -t <topic_name> -o <output_video_file_name> <bag_file_name> 
# Eg. python3 rosbag2video.py -t /cam0/image_raw -o myvideo.mp4 ar_tracking_1.bag
```


For running ROS 2 `ros2bag2video.py`:

```bash
source /opt/ros/humble/setup.bash
```

```bash
python3 ros2bag2video.py -t <topic_name> -i <bag_folder_name> --save_images -o <output_video_file_name>
# Eg. python3 ros2bag2video.py -t /virtual_camera/image_raw -i rosbag2_2024_10_11-19_45_28 --save_images -o myvideo.mp4
```

## **Example Output**
If running `ros2bag2video.py` correctly, you should see something similar to what is shown below:

```bash
[INFO] - Processing messages: [43/193]
...
frame=  187 fps=6.3 q=29.0 size=   11264kB time=00:00:04.06 bitrate=22690.2kbits/s speed=0.187x 
frame=  193 fps=5.7 q=-1.0 Lsize=   17157kB time=00:00:06.33 bitrate=22191.6kbits/s speed=0.187x    
[INFO] - Writing video to: myvideo.mp4
```

If running `rosbag2video.py` correctly, you should see something similar to what is shown below:

```bash
############# UNCOMPRESSED IMAGE ######################
/cam0/image_raw  with datatype: sensor_msgs/Image

frame=  178 fps=0.0 q=28.0 size=     256kB time=00:00:04.64 bitrate= 452.0kbits/s speed
frame=  340 fps=338 q=28.0 size=     768kB time=00:00:11.12 bitrate= 565.8kbits/s speed
frame=  513 fps=341 q=28.0 size=    1280kB time=00:00:18.04 bitrate= 581.3kbits/s speed
finished  
```