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
try:
    from theora_image_transport.msg import Packet
except Exception:
    pass
# from rosbag2_transport import rosbag2_transport_py
from ros2bag.api import check_path_exists
from ros2cli.node import NODE_NAME_PREFIX
from argparse import FileType

VIDEO_CONVERTER_TO_USE = 'ffmpeg'


def print_help():
    '''
    The print_help function.
    Outputs how a user can configure use of ros2bag2video to generate the video
    from ROS2 bag file.
    '''
    print('ros2bag2video.py [--fps 25] [--rate 1] [-o outputfile] [-v] ' +
          '[-s] [-t topic] bagfile1 [bagfile2] ...')
    print()
    print('Converts image sequence(s) in ros bag file(s) to video file(s)' +
          ' with fixed frame rate using', VIDEO_CONVERTER_TO_USE)
    print(VIDEO_CONVERTER_TO_USE, 'needs to be installed!')
    print()
    print('--fps   Sets FPS value that is passed to', VIDEO_CONVERTER_TO_USE)
    print('        Default is 25.')
    print('-h      Displays this help.')
    print('--ofile (-o) sets output file name.')
    print('        If no output file name (-o) is given the filename' +
          ' \'output.mp4\' is used')
    print('        Multiple image topics are supported only when -o ' +
          'option is _not_ used.')
    print('        ', VIDEO_CONVERTER_TO_USE, ' will guess the format ' +
          'according to given extension.')
    print('        Compressed and raw image messages are supported with ' +
          'mono8 and bgr8/rgb8/bggr8/rggb8 formats.')
    print('--rate  (-r) You may slow down or speed up the video.')
    print('        Default is 1.0, that keeps the original speed.')
    print('-s      Shows each and every image extracted from the rosbag file' +
          ' (cv_bride is needed).')
    print('--topic (-t) Only the images from topic "topic" are used for the' +
          'video output.')
    print('-v      Verbose messages are displayed.')


class RosVideoWriter(Node):
    '''
    The RosVideoWriter class is a ROS2 node instantiated to subscribe to a
    user-defined ROS2 topic on a user-defined ROS2 bag file.
    '''
    def __init__(self, args):
        '''
        The constructor.
        1. Initializes class attributes using user inputs and reading input
        bag file.
        2. Defines subscriber callback.
        3. Plays input bag file.
        '''
        super().__init__('ros2bag2videos')

        self.fps = 25
        self.rate = 1.0
        self.frame_no = 1
        self.opt_out_file = 'output.mp4'
        self.opt_topic = ''
        self.opt_verbose = False
        self.pix_fmt_already_set = False
        self.bridge = CvBridge()
        self.pix_fmt = 'yuv420p'
        self.msg_fmt = ''

        # Checks if a ROS2 bag has been specified in commandline.
        if len(args) < 2:
            print('Please specify ROS2 bag file!')
            print_help()
            sys.exit(1)
        try:
            opt_files = self.parse_args(args[1:])
            print('FPS (int) = ', self.fps)
            print('Rate (float) = ', self.rate)
            print('Topic (str) = ', self.opt_topic)
            print('Output File (str) = ', self.opt_out_file)
            print('Verbose (bool) = ', self.opt_verbose)
        except getopt.GetoptError:
            print_help()
            sys.exit(2)

        self.bag_file = opt_files[0]
        print('bag_file = ', self.bag_file)

        proc = subprocess.Popen(['ros2',
                                 'bag',
                                 'info',
                                 self.bag_file],
                                stdout=subprocess.PIPE)
        rosbag2_info = str(proc.stdout.read(), 'utf-8').splitlines()

        self.msgfmt_literal, self.count = self.get_topic_info(rosbag2_info)
        self.msgtype = self.filter_image_msgs(self.msgfmt_literal)

        # DEBUG
        # print(self.rosbag2_info)
        # print("msgfmt_literal = ", self.msgfmt_literal)
        # print("count = ", self.count)
        # print("serialtype = ", self.serialtype)

        self.subscription = self.create_subscription(
            self.msgtype,
            self.opt_topic,
            self.listener_callback,
            10)

        p1 = subprocess.Popen(['ros2',
                               'bag',
                               'play',
                               self.bag_file,
                               '-r',
                               str(self.rate)])

    '''
    Parses user input from commandline to get the following information.
    1. Verbose [opt_verbose]
    2. FPS [fps]
    3. Rate [rate]
    4. Output File Name [opt_out_file]
    5. Input Topic Name [opt_topic]
    6. Input Bag File Path Name [opt_files[0]]
    '''
    def parse_args(self, args):
        opts, opt_files = getopt.getopt(args, 'hsvr:o:t:p:',
                                        ['fps=',
                                         'rate=',
                                         'ofile=',
                                         'topic='])
        for opt, arg in opts:
            if opt == '-h':
                print_help()
                sys.exit(0)
            elif opt == '-v':
                self.opt_verbose = True
            elif opt in ('--fps'):
                self.fps = int(arg)
            elif opt in ('-r', '--rate'):
                self.rate = float(arg)
            elif opt in ('-o', '--ofile'):
                self.opt_out_file = arg
            elif opt in ('-t', '--topic'):
                self.opt_topic = arg
            else:
                print('opz:', opt, 'arg:', arg)

        if (self.fps <= 0):
            print('Invalid fps', self.fps)
            self.fps = 1

        if (self.rate <= 0):
            print('Invalid rate', self.rate)
            self.rate = 1

        if(self.opt_verbose):
            print('Using ', self.fps, ' FPS')
        return opt_files

    '''
    Detemines the ROS2 message format which RosVideoWriter node will expect to
    receive in our subscriber callback.
    '''
    def filter_image_msgs(self, msgfmt_literal):

        if 'sensor_msgs/msg/Image' == msgfmt_literal:
            return Image
        elif 'sensor_msgs/msg/CompressedImage' == msgfmt_literal:
            return CompressedImage
        elif 'theora_image_transport/msg/Packet' == msgfmt_literal:
            return Packet

    '''
    Parses ROS2 message encoding to derive the following information.
    1. pix_fmt (To be passed to ffmpeg process execution.)
    2. msg_fmt (To be passed to cv_bridge function to convert ROS2 messages
    to OpenCV Mat)
    '''
    def get_pix_fmt(self, msg_encoding):

        pix_fmt = 'yuv420p'
        msg_fmt = ''

        try:
            if msg_encoding.find('mono8') != -1:
                pix_fmt = 'gray'
                msg_fmt = 'bgr8'
            elif msg_encoding.find('8UC1') != -1:
                pix_fmt = 'gray'
                msg_fmt = 'bgr8'
            elif msg_encoding.find("bgra") != -1:
                pix_fmt = 'bgra'
                msg_fmt = 'bgr8'
            elif msg_encoding.find('bgr8') != -1:
                pix_fmt = 'bgr24'
                msg_fmt = 'bgr8'
            elif msg_encoding.find('bggr8') != -1:
                pix_fmt = 'bayer_bggr8'
                msg_fmt = 'bayer_bggr8'
            elif msg_encoding.find('rggb8') != -1:
                pix_fmt = 'bayer_rggb8'
                msg_fmt = 'bayer_rggb8'
            elif msg_encoding.find('rgb8') != -1:
                pix_fmt = 'rgb24'
                msg_fmt = 'bgr8'
            elif msg_encoding.find('16UC1') != -1:
                pix_fmt = 'gray16le'
                msg_fmt = 'mono16'
            else:
                print('Unsupported encoding:', msg_encoding, topic)
                exit(1)

            return pix_fmt, msg_fmt

        except AttributeError:
            # maybe theora packet
            # theora not supported
            print('Could not handle this format.' +
                  ' Maybe thoera packet? theora is not supported.')
            exit(1)

    '''
    Parses 'ros2 bag info input_bag/' terminal output to get the following
    information:
    1. ROS2 Message Type (To be used in constructor to define subscriber
    callback.)
    2. ROS2 Message Frame Count (To be used in subscriber callback to track
    progress.)
    3. ROS2 Bag Serialization Format (Not used.)

    Returns only the first two info.
    '''
    def get_topic_info(self, rosbag2_info):

        msgtype = ''
        count = 0
        serialtype = ''  # Unused

        for line in rosbag2_info:
            if self.opt_topic in line:
                parse_line = line.split()
                for word_index in range(0, len(parse_line)):
                    if 'Type:' in parse_line[word_index]:
                        msgtype = parse_line[word_index+1]
                    if 'Count:' in parse_line[word_index]:
                        count = int(parse_line[word_index+1])
                    if 'Serialization' in parse_line[word_index]:
                        serialtype = parse_line[word_index+2]

        return msgtype, count

    '''
    The Subscriber Callback.
    1. Receives images from running ROS2 bag file.
    2. Get pix_fmt info for ffmpeg process execution.
    3. Manually converts 16UC1 color encoding to mono16 to avoid cv_bridge
    conversion error.
    4. Converts ROS2 image message to OpenCV Mat object.
    5. Writes individual frame out to file.
    6. Uses ffmpeg to stitch all frames into a video at user-defined
    configuration.
    7. Removes all individual frames.
    '''
    def listener_callback(self, msg):
        self.get_logger().info('Image Received [%i/%i]' %
                               (self.frame_no, self.count))

        if self.pix_fmt_already_set is not True:
            self.pix_fmt, self.msg_fmt = self.get_pix_fmt(msg.encoding)
            self.pix_fmt_already_set = True

        if msg.encoding.find('16UC1') != -1:
            msg.encoding = 'mono16'

        img = self.bridge.imgmsg_to_cv2(msg, self.msg_fmt)

        filename = str(self.frame_no).zfill(3) + '.png'
        cv2.imwrite(filename, img)

        '''
        Once the last frame is reached, combine all individual image frames
        together to create the video. Remove all individual image frames and
        kill program once done.
        Otherwise, continue incrementing frame count.
        '''
        if self.frame_no == self.count:
            p1 = subprocess.Popen([VIDEO_CONVERTER_TO_USE,
                                  '-framerate',
                                   str(self.fps),
                                   '-pattern_type',
                                   'glob',
                                   '-i',
                                   '*.png',
                                   '-c:v',
                                   'libx264',
                                   '-pix_fmt',
                                   self.pix_fmt,
                                   self.opt_out_file,
                                   '-y'])
            p1.communicate()
            args = ('rm', '*.png')
            p2 = subprocess.call('%s %s' % args, shell=True)
            print('Writing to output file, ' + self.opt_out_file)
            sys.exit()
        else:
            self.frame_no = self.frame_no + 1


def main(args=None):
    '''
    The main function.
    Starts ros2bag2videos ROS2 node and spins it.
    '''
    rclpy.init(args=args)

    videowriter = RosVideoWriter(args)

    rclpy.spin(videowriter)

    videowriter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
