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
import argparse
import shutil
from pathlib import Path
from typing import Tuple

import cv2
from cv_bridge import CvBridge
from rosbags.highlevel import AnyReader
from rosbags.interfaces import Connection


IS_VERBOSE = False


def get_pix_fmt(msg_encoding: str) -> str:
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


def get_topic_info(reader: AnyReader, topic_name: str) -> Tuple[int, str, int]:
    """Return (message_count, msg_type, connection) for *topic_name*."""
    conn = next((c for c in reader.connections if c.topic == topic_name), None)
    if conn is None:
        sys.exit(f"[ERROR] - Topic '{topic_name}' not found in bag.")
    return conn.msgcount, conn.msgtype, conn


def get_msg_format_from_rosbag(reader: AnyReader, connection: Connection) -> str:
    """Peek at first message to derive ``msg.format``/``msg.encoding``."""
    try:
        _, _, raw = next(reader.messages(connections=[connection]))
    except StopIteration:
        return ""
    msg = reader.deserialize(raw, connection.msgtype)
    return getattr(msg, "format", getattr(msg, "encoding", ""))


def save_image_from_rosbag(
    cvbridge: CvBridge,
    reader: AnyReader,
    connection: Connection,
    input_msg_type: str,
    message_index: int = 0,
) -> None:
    """
    Save an image from a ROS bag.

    Args:
        cvbridge: CvBridge instance for converting between OpenCV and ROS images.
        reader: Rosbag reader.
        connection: connection containing the image messages.
        input_msg_type: The type of message in the topic, e.g. "sensor_msgs/msg/Image".
        message_index (optional): The index of the message to save. Defaults to 0.

    Returns:
        None

    Raises:
        Exception: If an error occurs during image conversion or saving.

    Notes:
        This function queries a ROS 2 database for messages in a specified topic,
        deserializes them into OpenCV images, and saves them as PNG files.
    """
    for i, (conn, ts, raw) in enumerate(reader.messages(connections=[connection])):

        print(f"[INFO] - Extracting [{i+1}/{message_count}] …", end="\r")
        sys.stdout.flush()

        msg = reader.deserialize(raw, connection.msgtype)
        image_file_type = ".jpg" if getattr(msg, "format", "").lower() == "jpeg" else ".png"

        if input_msg_type.endswith("CompressedImage"):
            cv_image = cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        else:
            cv_image = cvbridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        padded_number = f"{i:07d}"
        output_filename = f"frames/{padded_number}{image_file_type}"
        cv2.imwrite(output_filename, cv_image)
        
    else:
        print(f"[ERROR] - No message at index {message_index} for topic {conn.topic}")


def check_and_create_folder(folder_path: str) -> None:
    """
    Check if a directory exists and create it if not.

    Args:
        folder_path: The path of the directory to be checked or created.

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


def clear_folder_if_non_empty(folder_path: str) -> bool:
    """
    Check if a folder is non-empty. If it is, remove all its contents.

    Parameters:
        folder_path: The path of the folder to check and clear.

    Returns:
        True if the folder was cleared, False if it was already empty.
    """
    # Check if the folder exists
    if not os.path.exists(folder_path):
        if IS_VERBOSE:
            print(f"[WARN] - The folder '{folder_path}' does not exist.")
        return False

    # List all files and directories in the folder
    contents = os.listdir(folder_path)
    if contents:
        for item in contents:
            item_path = os.path.join(folder_path, item)
            if os.path.isfile(item_path):
                os.remove(item_path)  # Remove files
            else:
                shutil.rmtree(item_path)  # Remove directories
        if IS_VERBOSE:
            print(f"[INFO] - Cleared all contents from '{folder_path}'...")
        return True
    if IS_VERBOSE:
        print(f"[INFO] - The folder '{folder_path}' is already empty.")
    return False


def create_video_from_images(image_folder: str, output_video: str, pix_fmt: str, framerate: int = 30):
    """
    Creates a video from a list of images in the specified folder.

    Args:
        image_folder: The path to the folder containing the images.
        output_video: The desired file name for the generated video.
        pix_fmt: ffmpeg pixel format.
        framerate (optional): The frame rate of the resulting video. Defaults to 30.

    Returns:
        True if the operation was successful, False otherwise.
    """
    images = sorted(
        [img for img in os.listdir(image_folder) if img.endswith((".png", ".jpg", ".jpeg"))],
        key=lambda x: int(os.path.splitext(x)[0]),  # Sort by the numeric part of the filename
    )
    if not images:
        print("[WARN] - No images found in the specified folder.")
        return False

    # Create a temporary text file listing all images
    image_list_file = os.path.join(image_folder, "_images.txt")
    with open(image_list_file, "w", encoding="utf-8") as f:
        for path in images:
            f.write(f"file '{path}'\n")

    # Build the ffmpeg command
    command = [
        "ffmpeg",
        "-loglevel",
        "error" if not IS_VERBOSE else "info",
        "-stats",
        "-r",
        str(framerate),  # Set frame rate
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        image_list_file,  # Input list of images
        "-c:v",
        "libx264",
        "-pix_fmt",
        pix_fmt,
        output_video,
        "-y",
    ]

    if IS_VERBOSE:
        print("[INFO] -", " ".join(command))
    try:
        subprocess.run(command, check=True)
        print(f"[INFO] - Video written to {output_video}.")
        os.remove(image_list_file)
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] - Error occurred: {e}")
        os.remove(image_list_file)
        return False


def create_video_from_jpg(
    reader: AnyReader,
    connection: Connection,
    output_video: str,
    fps: float,
    max_frames: int = -1,
):
    """
    Save an video from a ROS bag with jpg compressed images into a mjpeg video file.

    Args:
        reader: Rosbag reader.
        connection: Connection containing the image messages.
        output_video: Output video filepath.
        fps: The desired video framerate.
        max_frames (int, optional): stops export after this number of frames.

    Returns:
        None

    Raises:


    Notes:

    """
    cmd = [
        "ffmpeg",
        "-loglevel",
        "error" if not IS_VERBOSE else "info",
        "-stats",
        "-r",
        str(fps),
        "-f",
        "mjpeg",
        "-i",
        "-",  # stdin
        "-c:v",
        "copy",
        "-an",
        output_video,
        "-y",
    ]
    if IS_VERBOSE:
        print("[INFO] -", " ".join(cmd))

    ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    for i, (conn, ts, raw) in enumerate(reader.messages(connections=[connection])):
        if 0 < max_frames <= i:
            break
        ffmpeg.stdin.write(raw)  # raw is already JPEG bytes
        if IS_VERBOSE:
            print(f"[INFO] - Streaming frame {i+1}", end="\r")
    ffmpeg.stdin.close()
    ffmpeg.wait()
    print(f"[INFO] - Video written to {output_video}.")


if __name__ == "__main__":
    # Parse commandline input arguments.
    parser = argparse.ArgumentParser(
        prog="rosbag2video",
        description="Convert ROS bag (1/2) to video using ffmpeg.",
    )
    parser.add_argument("-v", "--verbose", action="store_true", required=False, default=False,
                        help="Run rosbag2video script in verbose mode.")
    parser.add_argument("-r", "--rate", type=int, required=False, default=30,
                        help="Video framerate")
    parser.add_argument("-t", "--topic", type=str, required=True,
                        help="Topic Name")
    parser.add_argument("-i", "--ifile", type=str, required=True,
                        help="Input File")
    parser.add_argument("-o", "--ofile", type=str, required=False, default="output_video.mp4",
                        help="Output File")
    parser.add_argument("--save_images", action="store_true", required=False, default=False,
                        help="Boolean flag for saving extracted .png frames in frames/")
    parser.add_argument("--frames", type=int, required=False, default=-1,
                        help="Limit the number of frames to export")
    args = parser.parse_args(sys.argv[1:])

    IS_VERBOSE = args.verbose

    # Check if input fps is valid.
    if args.rate <= 0:
        print(f"[WARN] - Invalid rate {args.rate}; using 30 FPS.")
        args.rate = 30

    # Check if bag exists
    bag_path = Path(args.ifile).expanduser().resolve()
    if not bag_path.exists():
        sys.exit(f"[ERROR] - Path '{bag_path}' does not exist.")

    # Process the bag
    with AnyReader([bag_path]) as reader:
        message_count, msg_type, conn = get_topic_info(reader, args.topic)

        msg_encoding = get_msg_format_from_rosbag(reader, conn)
        if (
            msg_type.endswith("CompressedImage")
            and not args.save_images
            and msg_encoding in ("jpeg", "jpg")
        ):
            # we can directly feed the jpg data to ffmpeg to create the video
            create_video_from_jpg(reader, conn, args.ofile, args.rate, args.frames)
            exit(0)

        # else do the image export stuff - extract frames, then ffmpeg concat
        FRAMES_FOLDER = "frames"
        check_and_create_folder(FRAMES_FOLDER)
        clear_folder_if_non_empty(FRAMES_FOLDER)

        bridge = CvBridge()
        save_image_from_rosbag(bridge, reader, conn, msg_type)

    # Construct video from image sequence
    pix_fmt = get_pix_fmt(msg_encoding)
    if not create_video_from_images(FRAMES_FOLDER, args.ofile, pix_fmt, framerate=args.rate):
        print("[ERROR] - Could not generate video.")

    # Keep or remove frames folder content based on --save-images flag.
    if not args.save_images:
        clear_folder_if_non_empty(FRAMES_FOLDER)
