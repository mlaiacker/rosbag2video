#!/usr/bin/env python3
"""
Module to generate video output given ROS 2 bag folders via direct
sqlite3 extraction from .db3 files and metadata.yaml
"""
# -*- coding: utf-8 -*-
#
# Copyright (c) 2024 Bey Hao Yun.
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

import os
import sys
import subprocess
import sqlite3
import argparse
import shutil
import cv2
import yaml
from cv_bridge import CvBridge
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from rclpy.time import Time

IS_VERBOSE = False
MSG_ENCODING = ''

def get_pix_fmt(msg_encoding):
    """
    Determine pixel format based on message encoding.

    Args:
        msg_encoding (str): The encoding of the message.

    Returns:
        str: The determined pixel format.

    Notes:
        This function attempts to infer the correct pixel format from a given 
        message encoding. It supports various formats, including gray, bgra, 
        and RGB variants. If an unsupported encoding is provided, it defaults 
        to yuv420p or prints warnings accordingly.

    Raises:
        AttributeError: If msg_encoding cannot be handled.
    """
    pix_fmt = "yuv420p"

    if IS_VERBOSE:
        print("[INFO] - AJB: Encoding:", msg_encoding)

    try:
        if msg_encoding.find("mono8") != -1:
            pix_fmt = "gray"
        elif msg_encoding.find("8UC1") != -1:
            pix_fmt = "gray"
        elif msg_encoding.find("bgra") != -1:
            pix_fmt = "bgra"
        elif msg_encoding.find("bgr8") != -1:
            pix_fmt = "bgr24"
        elif msg_encoding.find("bggr8") != -1:
            pix_fmt = "bayer_bggr8"
        elif msg_encoding.find("rggb8") != -1:
            pix_fmt = "bayer_rggb8"
        elif msg_encoding.find("rgb8") != -1:
            pix_fmt = "rgb24"
        elif msg_encoding.find("16UC1") != -1:
            pix_fmt = "gray16le"
        else:
            print(f"[WARN] - Unsupported encoding: {msg_encoding}. "
                  "Defaulting pix_fmt to {pix_fmt}...")
    except AttributeError:
        # Maybe this is a theora packet which is unsupported.
        print(
            "[ERROR] - Could not handle this format."
            + " Maybe thoera packet? theora is not supported."
        )
        sys.exit(1)
    if IS_VERBOSE:
        print("[INFO] - pix_fmt:", pix_fmt)
    return pix_fmt

def save_image_from_rosbag(
        cvbridge,
        cursor,
        topic_name,
        input_msg_type,
        message_index=0):
    """
    Save an image from a ROS bag.

    Args:
        cvbridge: CvBridge instance for converting between OpenCV and ROS images.
        cursor: Database cursor to query the ROS 2 database.
        topic_name (str): The name of the ROS 2 topic containing the image messages.
        input_msg_type (str): The type of message in the topic, e.g. "sensor_msgs/msg/Image".
        message_index (int, optional): The index of the message to save. Defaults to 0.

    Returns:
        None

    Raises:
        Exception: If an error occurs during image conversion or saving.

    Notes:
        This function queries a ROS 2 database for messages in a specified topic,
        deserializes them into OpenCV images, and saves them as PNG files.
    """
    # Query for the messages in the specified ROS 2 topic
    query = """
    SELECT data
    FROM messages
    WHERE topic_id = (SELECT id FROM topics WHERE name = ?)
    LIMIT 1 OFFSET ?;
    """
    cursor.execute(query, (topic_name, message_index))
    message_data = cursor.fetchone()

    if not message_data:
        print(f"[ERROR] - No message found at index {message_index} for topic {topic_name}")
        return

    # Deserialize the sensor_msgs/msg/Image message
    msg_type = get_message(input_msg_type)
    msg = deserialize_message(message_data[0], msg_type)
    image_file_type = ".png"

    if msg.format == "jpeg":
        image_file_type = '.jpg'

    try:
        global MSG_ENCODING
        # AttributeError: 'CompressedImage' object has no attribute 'encoding'
        # format='jpeg'
        MSG_ENCODING = msg.encoding
    except AttributeError:
        pass

    # Use CvBridge to convert the ROS Image message to an OpenCV image
    try:
        if input_msg_type == "sensor_msgs/msg/CompressedImage":
            cv_image = cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        else:
            cv_image = cvbridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        print(f"[ERROR] - Error converting image: {e}")
        return

    # Save the image using OpenCV
    padded_number = f"{message_index:07d}"
    output_filename = "frames/" + padded_number + image_file_type
    cv2.imwrite(output_filename, cv_image)

def check_and_create_folder(folder_path):
    """
    Check if a directory exists and create it if not.

    Args:
        folder_path (str): The path of the directory to be checked or created.

    Returns:
        None

    Notes:
        This function attempts to ensure that the specified directory is present.
        If it does not exist, an attempt is made to create it. Any errors during
        creation are logged and reported.

    Raises:
        OSError: If there's a problem creating the folder.
    """
    if not os.path.exists(folder_path):
        try:
            os.makedirs(folder_path)
            if IS_VERBOSE:
                print(f"[INFO] - Folder '{folder_path}' created successfully.")
        except OSError as e:
            print(f"[ERROR] - Failed to create folder '{folder_path}'. {e}")

def clear_folder_if_non_empty(folder_path):
    """
    Check if a folder is non-empty. If it is, remove all its contents.

    Parameters:
        folder_path (str): The path of the folder to check and clear.

    Returns:
        bool: True if the folder was cleared, False if it was already empty.
    """
    # Check if the folder exists
    if not os.path.exists(folder_path):
        if IS_VERBOSE:
            print(f"[WARN] - The folder '{folder_path}' does not exist.")
        return False

    # List all files and directories in the folder
    contents = os.listdir(folder_path)

    # If the folder is not empty, remove all its contents
    if contents:
        for item in contents:
            item_path = os.path.join(folder_path, item)
            if os.path.isfile(item_path):
                os.remove(item_path)  # Remove files
            elif os.path.isdir(item_path):
                shutil.rmtree(item_path)  # Remove directories
        if IS_VERBOSE:
            print(f"[INFO] - Cleared all contents from '{folder_path}'...")
        return True
    if IS_VERBOSE:
        print(f"[INFO] - The folder '{folder_path}' is already empty.")
    return False

def get_info_from_yaml(yaml_file, topic_name):
    """
    Extract message count and type from a ROS 2 bag's YAML metadata.

    Args:
        yaml_file (str): The path of the YAML file containing the bag information.
        topic_name (str): The name of the topic for which to retrieve info.

    Returns:
        tuple: A pair containing the number of messages in the specified topic
            and its message type. If either value is None, it means an error occurred.

    Raises:
        FileNotFoundError: If the YAML file does not exist.
        yaml.YAMLError: If there's a problem parsing the YAML data.

    Notes:
        This function attempts to extract relevant information from a ROS 2 bag's
        metadata. It looks for specific keys in the provided YAML file and returns
        the requested values if found, or logs an error otherwise.
    """

    message_count = None
    msg_type = None

    try:
        # Open and read the YAML file
        with open(yaml_file, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)

        # Access the messages_count under rosbag2_bagfile_information
        path = data.get(
            'rosbag2_bagfile_information', {}).get('files')[0].get('path')
        topics = data.get(
            'rosbag2_bagfile_information', {}).get('topics_with_message_count')

        for topic in topics:
            if topic['topic_metadata']['name'] == topic_name:
                message_count = topic['message_count']
                print(f"topic = {topic}")
                msg_type = topic['topic_metadata']['type']

        if message_count is None:
            print(f"[ERROR] - No matching topic for {topic_name} in {path}. "
                  "Please ensure you have provided the correct topic name. Exiting...")
            sys.exit(1)

        if message_count is not None and msg_type is not None:
            if IS_VERBOSE:
                print(f"[INFO] - {path} has {message_count} {msg_type} messages...")
            return message_count, msg_type

        print("[ERROR] - messages_count not found in the YAML file.")
        sys.exit()

    except FileNotFoundError:
        print(f"[ERROR] - The file {yaml_file} does not exist.")
    except yaml.YAMLError as e:
        print(f"[ERROR] - Failed to parse YAML file. {e}")

def create_video_from_images(image_folder, output_video, framerate=30):
    """
    Creates a video from a list of images in the specified folder.

    Args:
        image_folder (str): The path to the folder containing the images.
        output_video (str): The desired file name for the generated video.
        framerate (int, optional): The frame rate of the resulting video. Defaults to 30.

    Returns:
        bool: True if the operation was successful, False otherwise.
    """
    images = sorted(
        [img for img in os.listdir(image_folder) if img.endswith(('.png', '.jpg', '.jpeg'))],
        key=lambda x: int(os.path.splitext(x)[0])  # Sort by the numeric part of the filename
    )

    if not images:
        print("[WARN] - No images found in the specified folder.")
        return

    IMAGE_TXT_FILE = 'images.txt'
    # Create a temporary text file listing all images
    with open(IMAGE_TXT_FILE, 'w', encoding='utf-8') as f:
        for image in images:
            f.write(f"file '{os.path.join(image_folder, image)}'\n")

    # Determine pix_fmt from ROS msg encoding.
    pix_fmt = get_pix_fmt(MSG_ENCODING)

    command = []
    # Build the ffmpeg command
    if IS_VERBOSE:
        command = [
            'ffmpeg',
            '-r', str(framerate),  # Set frame rate
            '-f', 'concat',
            '-safe', '0',
            '-i', IMAGE_TXT_FILE,  # Input list of images
            '-c:v', 'libx264',
            '-pix_fmt', pix_fmt,
            output_video,
            "-y"
        ]
    else:
        command = [
            'ffmpeg',
            '-r', str(framerate),  # Set frame rate
            '-f', 'concat',
            '-safe', '0',
            '-i', IMAGE_TXT_FILE,  # Input list of images
            '-c:v', 'libx264',
            '-pix_fmt', pix_fmt,
            '-loglevel', 'error', '-stats',
            output_video,
            "-y"
        ]

    # Run the ffmpeg command
    try:
        subprocess.run(command, check=True)
        print(f"[INFO] - Writing video to: {output_video}")
        os.remove(IMAGE_TXT_FILE)
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] - Error occurred: {e}")
        os.remove(IMAGE_TXT_FILE)
        return False

def create_video_from_jpg(cursor, output_video:str, topic_name:str, fps:float, input_msg_type,message_count, max_frames:int = -1):
    """
    Save an video from a ROS bag with jpg compressed images.

    Args:

    Returns:
        None

    Raises:


    Notes:

    """
    
    cmd = [
                        'ffmpeg',
                        '-v',
                        '10',
                        '-stats',
                        '-r',str(fps),
                        '-c',
                        'mjpeg',
                        '-f',
                        'mjpeg',
                        '-i',
                        '-',
                        '-an',
                        output_video]

    # Query for the messages in the specified ROS 2 topic
    query = """
    SELECT data
    FROM messages
    WHERE topic_id = (SELECT id FROM topics WHERE name = ?);
    """
    cursor.execute(query, (topic_name,))
    message_data = cursor.fetchone()

    if IS_VERBOSE:
        print(cmd)
    ffmpeg_pipe = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    msg_type = get_message(input_msg_type)
    i = 0
    t_video = 0.0
    t_0 = 0.0
    t_bag = 0.0
    while message_data:
        i=i+1
        # Deserialize the sensor_msgs/msg/Image message
        msg = deserialize_message(message_data[0], msg_type)
        if t_0 == 0:
            if IS_VERBOSE:
                print(msg)
            # get timestamp of first frame
            t_0 = msg.header.stamp
        t_bag = (Time.from_msg(msg.header.stamp).nanoseconds - Time.from_msg(t_0).nanoseconds)*1e-9
        ffmpeg_pipe.stdin.write(msg.data)
        if IS_VERBOSE:
            print(f"[INFO] - Processing messages: [{i+1}/{message_count}]... bag time:{t_bag:.2f}s, video time:{t_video:.2f}s", end='\r')        
        t_video += 1.0/fps
        message_data = cursor.fetchone()
        if max_frames>0  and i>=max_frames:
            break
    if IS_VERBOSE:
        print("finished")
    ffmpeg_pipe.stdin.close()




def get_db3_filepath(folder_path):
    """
    Get the filenames of .db3 files in a specified folder.

    Parameters:
        folder_path (str): The path of the folder to search for .db3 files.

    Returns:
        list: A list of .db3 filenames in the folder. 
              Returns an empty list if no .db3 files are found.
    """
    # Check if the folder exists
    if not os.path.exists(folder_path):
        print(f"The folder '{folder_path}' does not exist.")
        return []

    # List to store .db3 filenames
    db3_files = []
    yaml_files = []

    # Iterate through the files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith('.db3'):
            db3_files.append(filename)  # Add .db3 file to the list

    # Iterate through the files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith('.yaml'):
            yaml_files.append(filename)  # Add .db3 file to the list

    assert len(db3_files)==1
    assert len(yaml_files)==1

    return folder_path + "/" + db3_files[0], folder_path + "/" + yaml_files[0]

if __name__ == "__main__":

    # Parse commandline input arguments.
    parser = argparse.ArgumentParser(
        prog="ros2bag2video",
        description="Convert ros2 bag file into a mp4 video")
    parser.add_argument("-v", "--verbose", action="store_true", required=False, default=False,
                        help="Run ros2bag2video script in verbose mode.")
    parser.add_argument("-r", "--rate", type=int, required=False, default=30,
                        help="Rate")
    parser.add_argument("-t", "--topic", type=str, required=True,
                        help="ROS 2 Topic Name")
    parser.add_argument("-i", "--ifile", type=str, required=True,
                        help="Input File")
    parser.add_argument("-o", "--ofile", type=str, required=False, default="output_video.mp4",
                        help="Output File")
    parser.add_argument("--save_images", action="store_true", required=False, default=False,
                        help="Boolean flag for saving extracted .png frames in frames/")
    parser.add_argument("--frames", type=int, required=False, default=-1,
                        help="number of frames to export")
    args = parser.parse_args(sys.argv[1:])

    db_path, yaml_path = get_db3_filepath(args.ifile)
    topic_name = args.topic

    IS_VERBOSE = args.verbose

    # Check if input fps is valid.
    if args.rate <= 0:
        print(f"Invalid rate: {args.rate}. Setting to default 30...")
        args.rate = 30

    # Get total number of messages from metadata.yaml
    message_count, msg_type = get_info_from_yaml(yaml_path, topic_name)

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    if msg_type == "sensor_msgs/msg/CompressedImage":
        create_video_from_jpg(cursor, args.ofile, topic_name, args.rate, msg_type, message_count, args.frames)
        conn.close()
        exit(0)
    

    FRAMES_FOLDER = "frames"

    check_and_create_folder(FRAMES_FOLDER)
    clear_folder_if_non_empty(FRAMES_FOLDER)

    # Connect to the database

    # Initialize the CvBridge
    bridge = CvBridge()

    message_index = 0
    # for testing
    for i in range(message_count):
        save_image_from_rosbag(bridge, cursor, topic_name, msg_type, message_index)
        message_index = message_index + 1
        print(f"[INFO] - Processing messages: [{i+1}/{message_count}]...", end='\r')
        sys.stdout.flush()

    # Close the database connection
    conn.close()

    # Construct video from image sequence
    output_video = args.ofile
    if not create_video_from_images(FRAMES_FOLDER, output_video, framerate=args.rate):
        print("[ERROR] - Unable to generate video...")

    # Keep or remove frames folder content based on --save-images flag.
    if not args.save_images:
        clear_folder_if_non_empty(FRAMES_FOLDER)
