# **rosbag2video**

> [!NOTE]
> A tool that converts rosbag to video files
> by [Maximilian Laiacker 2025](post@mlaiacker.de)

## **Install**

Build docker images using the commands below:

1. **Download** `rosbag2video`:

```bash
git clone https://github.com/mlaiacker/rosbag2video --depth 1 --single-branch
```

2. **Build** docker image for running ROS 1 `rosbag2video.py`:

```bash
docker build -f docker/Dockerfile.ros1 -t rosbag2video:noetic .
```

3. **Build** docker image for running ROS 2 `ros2bag2video.py`:

```bash
docker build -f docker/Dockerfile.ros2 -t rosbag2video:lyrical .
```

## **Usage**

By default it will extract all compressed image topics inside the bag directory with the name of <topic>.mp4 ('/' inside the topic name will be replaced by '_') as mjpeg encoded video with 30fps.

> [!NOTE]
> Mostly tested with bags containing:
> ```bash
> msg_type: sensor_msgs/msg/CompressedImage msg_encoding: jpeg
> ```

``` bash
usage: rosbag2video [-h] [-v] [-r RATE] [-t TOPIC] [-o OFILE] [--save_images] [--frames FRAMES] rosbag [rosbag ...]

Convert ROS bag (1/2) to video using ffmpeg.

positional arguments:
  rosbag                Input File(s)

options:
  -h, --help            show this help message and exit
  -v, --verbose         Run rosbag2video script in verbose mode.
  -r RATE, --rate RATE  Video framerate
  -t TOPIC, --topic TOPIC
                        Topic Name
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
  rosbag2video:lyrical bash -c "source /opt/ros/lyrical/setup.bash && /bin/bash"
```

Example: extract all image topics form one ore more rosbag2 directories or bags

```bash
python3 rosbag2video.py <bag_folder_name>
# Eg. python3 rosbag2video.py rosbag2_2024_10_11-19_45_28 -o myvideo.mp4
```

Example: extract a specific topic

```bash
python3 rosbag2video.py -t <topic_name> -o <output_video_file_name> <bag_folder_name>
# Eg. python3 rosbag2video.py -t /cam0/image_raw -o myvideo.mp4 rosbag2_2024_10_11-19_45_28
```
