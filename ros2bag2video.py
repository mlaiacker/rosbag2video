 # Copyright (c) 2021 Bey Hao Yun.
 #
 # This program is free software: you can redistribute it and/or modify
 # it under the terms of the GNU General Public License as published by
 # the Free Software Foundation, version 3.
 #
 # This program is distributed in the hope that it will be useful, but
 # WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 # General Public License for more details.
 #
 # You should have received a copy of the GNU General Public License
 # along with this program. If not, see <http://www.gnu.org/licenses/>.
 #

import os
import sys
import cv2
import rclpy
import getopt
import subprocess
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rosbag2_transport import rosbag2_transport_py
from ros2bag.api import check_path_exists
from ros2cli.node import NODE_NAME_PREFIX
from argparse import FileType

VIDEO_CONVERTER_TO_USE = "ffmpeg" # or you may want to use "avconv"

def print_help():
    print('ros2bag2video.py [--fps 25] [--rate 1] [-o outputfile] [-v] [-s] [-t topic] bagfile1 [bagfile2] ...')
    print()
    print('Converts image sequence(s) in ros bag file(s) to video file(s) with fixed frame rate using',VIDEO_CONVERTER_TO_USE)
    print(VIDEO_CONVERTER_TO_USE,'needs to be installed!')
    print()
    print('--fps   Sets FPS value that is passed to',VIDEO_CONVERTER_TO_USE)
    print('        Default is 25.')
    print('-h      Displays this help.')
    print('--ofile (-o) sets output file name.')
    print('        If no output file name (-o) is given the filename \'output.mp4\' is used')
    print('        Multiple image topics are supported only when -o option is _not_ used.')
    print('        ',VIDEO_CONVERTER_TO_USE,' will guess the format according to given extension.')
    print('        Compressed and raw image messages are supported with mono8 and bgr8/rgb8/bggr8/rggb8 formats.')
    print('--rate  (-r) You may slow down or speed up the video.')
    print('        Default is 1.0, that keeps the original speed.')
    print('-s      Shows each and every image extracted from the rosbag file (cv_bride is needed).')
    print('--topic (-t) Only the images from topic "topic" are used for the video output.')
    print('-v      Verbose messages are displayed.')

class RosVideoWriter(Node):

    def __init__(self, args):
        print('[rosbag2video] - started.')
        super().__init__('rosbag2videos')

        self.fps = 25
        self.rate = 1.0
        self.opt_display_images = False
        self.opt_out_file = 'output.mp4'
        self.opt_topic = ''
        self.opt_verbose = False

        # Checks if a ROS2 bag has been specified in commandline.
        if len(args) < 2:
            print('Please specify ROS2 bag file!')
            print_help()
            sys.exit(1)
        try:
            opt_files = self.parseArgs(args[1:])
            print("FPS (int) = ", self.fps)
            print("Rate (float) = ", self.rate)
            print("Topic (str) = ", self.opt_topic)
            print("Display Images (bool) = ", self.opt_display_images)
            print("Output File (str) = ", self.opt_out_file)
            print("Verbose (bool) = ", self.opt_verbose)
        except getopt.GetoptError:
            print_help()
            sys.exit(2)

        self.subscription = self.create_subscription(
            Image,
            self.opt_topic,
            self.listener_callback,
            10)

        self.bridge = CvBridge()
        self.frame_no = 1
        self.bag_file = opt_files[0]
        print("bag_file = ", self.bag_file)

        proc = subprocess.Popen(['ros2', 'bag', 'info', self.bag_file], stdout=subprocess.PIPE)
        self.rosbag2_info = str(proc.stdout.read(), 'utf-8')
        self.rosbag2_info = self.rosbag2_info.splitlines()
        # print(self.rosbag2_info)

        self.msgtype, self.count, self.serialtype = self.get_bagtopic_info()
        # DEBUG
        # print("msgtype = ", self.msgtype)
        # print("count = ", self.count)
        # print("serialtype = ", self.serialtype)

        p1 = subprocess.Popen(['ros2', 'bag', 'play', self.bag_file, '-r' , str(self.rate)])

    def parseArgs(self, args):
        opts, opt_files = getopt.getopt(args,"hsvr:o:t:p:",["fps=","rate=","ofile=","topic=","start=","end=","prefix="])
        for opt, arg in opts:
            if opt == '-h':
                print_help()
                sys.exit(0)
            elif opt == '-s':
                self.opt_display_images = True
            elif opt == '-v':
                self.opt_verbose = True
            elif opt in ("--fps"):
                self.fps = int(arg)
            elif opt in ("--rate"):
                self.rate = float(arg)
            elif opt in ("-o", "--ofile"):
                self.opt_out_file = arg
            elif opt in ("-t", "--topic"):
                self.opt_topic = arg
            else:
                print("opz:", opt,'arg:', arg)

        if (self.fps<=0):
            print("invalid fps", self.fps)
            self.fps = 1

        if (self.rate<=0):
            print("invalid rate", self.rate)
            self.rate = 1

        if(self.opt_verbose):
            print("using ",self.fps," FPS")
        return opt_files

    def get_bagtopic_info(self):

        msgtype = ''
        count = 0
        serialtype = ''

        for line in self.rosbag2_info:
            if self.opt_topic in line:
                parse_line = line.split()
                for word_index in range(0,len(parse_line)):
                    if 'Type:' in parse_line[word_index]:
                        msgtype = parse_line[word_index+1]
                    if 'Count:' in parse_line[word_index]:
                        count = int(parse_line[word_index+1])
                    if 'Serialization' in parse_line[word_index]:
                        serialtype = parse_line[word_index+2]


        return msgtype, count, serialtype

    def listener_callback(self, msg):
        self.get_logger().info('Image Received [%i/%i]' % (self.frame_no, self.count))
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        filename = str(self.frame_no).zfill(3) + ".png"
        print("Writing file, ", filename)
        cv2.imwrite(filename, img)

        if self.frame_no is self.count:
            # ffmpeg -framerate 10 -pattern_type glob -i '*.png' -c:v libx264 -r 30 -pix_fmt yuv420p out.mp4
            p1 = subprocess.Popen(['ffmpeg', '-framerate', str(self.fps), '-pattern_type', 'glob', '-i', '*.png', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', self.opt_out_file])
            p1.communicate()
            args = ('rm', '*.png')
            p2 = subprocess.call('%s %s' % args, shell=True)
            print("Writing to output file, " + self.opt_out_file)
            sys.exit()
        else:
            self.frame_no = self.frame_no + 1

def main(args=None):

    rclpy.init(args=args)

    videowriter = RosVideoWriter(args)

    rclpy.spin(videowriter)

    videowriter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
