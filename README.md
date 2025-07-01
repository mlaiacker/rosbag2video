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
git clone https://github.com/mlaiacker/rosbag2video
```

2. Build docker image for running ROS 1 `rosbag2video.py`:

```bash
docker build -f Dockerfile.ros1 -t rosbag2video:noetic .
```

3. Build docker image for running ROS 2 `ros2bag2video.py`:

```bash
docker build -f Dockerfile.ros2 -t rosbag2video:humble .
```

## **Usage**

``` bash
rosbag2video [-h] [-v] [-r RATE] -t TOPIC -i IFILE [-o OFILE] [--save_images]
                    [--frames FRAMES]

Convert ROS bag (1/2) to video using ffmpeg.

options:
  -h, --help            show this help message and exit
  -v, --verbose         Run rosbag2video script in verbose mode.
  -r RATE, --rate RATE  Video framerate
  -t TOPIC, --topic TOPIC
                        Topic Name
  -i IFILE, --ifile IFILE
                        Input File
  -o OFILE, --ofile OFILE
                        Output File
  --save_images         Boolean flag for saving extracted .png frames in frames/
  --frames FRAMES       Limit the number of frames to export
```

### **ROS 1**

```bash
docker run -it --rm \
    --name rosbag2video_c \
    -v .:/rosbag2video_workspace \
  rosbag2video:noetic bash
```

```bash
source /opt/ros/noetic/setup.bash
```

```bash
python3 rosbag2video.py -t <topic_name> -i <bag_file_name> -o <output_video_file_name>
# Eg. python3 rosbag2video.py -t /cam0/image_raw -i ar_tracking_1.bag -o myvideo.mp4
```

### **ROS 2**

```bash
docker run -it --rm \
    --name rosbag2video_c \
    -v .:/rosbag2video_workspace \
  rosbag2video:humble bash
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
python3 rosbag2video.py -t <topic_name> -i <bag_folder_name> -o <output_video_file_name>
# Eg. python3 rosbag2video.py -t /cam0/image_raw -i rosbag2_2024_10_11-19_45_28 -o myvideo.mp4
```
