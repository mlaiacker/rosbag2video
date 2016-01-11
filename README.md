# rosbag2video

  rosbag2video.py
  rosbag to video file conversion tool 
  by Maximilian Laiacker 2016
  post@mlaiacker.de

##uasge:
rosbag2video.py [--fps 25] [-o outputfile] [-s (show video)] bagfile1 [bagfile2] ...

  converts image sequence(s) in ros bag file(s) to video file(s) with fixed frame rate using avconv
  avconv needs to be installed!
  if no output file (-o) is given the filename '<topic>.mp4' is used and default output codec is h264
  multiple image topics are supportet only when -o option is _not_ used
  avconv will guess the format according to given extension
  compressed and raw image messages are supportet with mono8 and bgr8/rgb8

  example:
  ./rosbag2video.py -r 50 -o test.mp4 peng_2015-12-14-13-*.bag
