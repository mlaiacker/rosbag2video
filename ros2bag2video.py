import os
import cv2
import sys
import yaml
import shutil
import sqlite3
import subprocess
from cv_bridge import CvBridge
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

def save_image_from_rosbag(cursor, topic_name, message_index=0):
    # Query for the messages in the specified topic
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
    msg_type = get_message('sensor_msgs/msg/Image')
    msg = deserialize_message(message_data[0], msg_type)

    # Use CvBridge to convert the ROS Image message to an OpenCV image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        print(f"[ERROR] - Error converting image: {e}")
        return

    # Save the image using Pillow
    padded_number = f"{message_index:03d}"
    output_filename = "frames/" + padded_number + '.png'
    cv2.imwrite(output_filename, cv_image)

# Function to check if folder exists, and create it if it doesn't
def check_and_create_folder(folder_path):
    if not os.path.exists(folder_path):
        try:
            os.makedirs(folder_path)
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
        print(f"[INFO] - Cleared all contents from '{folder_path}'.")
        return True
    else:
        print(f"[INFO] - The folder '{folder_path}' is already empty.")
        return False

# Function to load the YAML file and extract messages_count
def get_messages_count_from_yaml(yaml_file):
    try:
        # Open and read the YAML file
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
            
        # Access the messages_count under rosbag2_bagfile_information
        message_count = data.get('rosbag2_bagfile_information', {}).get('message_count')
        
        if message_count is not None:
            print(f"[INFO] - Messages count: {message_count}")
            return message_count
        else:
            print("[ERROR] - messages_count not found in the YAML file.")
            sys.exit()
            
    except FileNotFoundError:
        print(f"Error: The file {yaml_file} does not exist.")
    except yaml.YAMLError as e:
        print(f"Error: Failed to parse YAML file. {e}")

def create_video_from_images(image_folder, output_video, framerate=30):
    # Get a sorted list of image files in the folder
    images = sorted(
        [img for img in os.listdir(image_folder) if img.endswith(('.png', '.jpg', '.jpeg'))],
        key=lambda x: int(os.path.splitext(x)[0])  # Sort by the numeric part of the filename
    )

    if not images:
        print("[WARN] - No images found in the specified folder.")
        return

    # Prepare image input pattern
    # Example: if images are named 001.png, 002.png, etc.
    image_pattern = os.path.join(image_folder, "%03d.png")  # Adjust padding as needed

    # Create a temporary text file listing all images
    with open('images.txt', 'w') as f:
        for image in images:
            f.write(f"file '{os.path.join(image_folder, image)}'\n")

    # TODO(cardboardcode): Remove hardcoded pix_fmt.

    # Build the ffmpeg command
    command = [
        'ffmpeg',
        '-r', str(framerate),  # Set frame rate
        '-f', 'concat',
        '-safe', '0',
        '-i', 'images.txt',  # Input list of images
        '-c:v', 'libx264',
        '-pix_fmt', 'yuv420p',
        output_video,
        "-y"
    ]

    # Run the ffmpeg command
    try:
        subprocess.run(command, check=True)
        print(f"[INFO] - Video created successfully: {output_video}")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] - Error occurred: {e}")

    # TODO(cardboardcode): Remove images.txt.

if __name__ == "__main__":

    # TODO(cardboardcode): Allow for more configurable parameters via argparse.

    db_path = "rosbag2_2024_10_11-19_45_28/rosbag2_2024_10_11-19_45_28_0.db3"
    topic_name = "/virtual_camera/image_raw"

    # Get total number of messages from metadata.yaml
    yaml_path = "rosbag2_2024_10_11-19_45_28/metadata.yaml"
    message_count = get_messages_count_from_yaml(yaml_path)

    frames_folder = "frames"

    check_and_create_folder(frames_folder)
    clear_folder_if_non_empty(frames_folder)

    # Connect to the database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Initialize the CvBridge
    bridge = CvBridge()

    message_index = 0
    for i in range(message_count):
        save_image_from_rosbag(cursor, topic_name, message_index)
        message_index = message_index + 1
        print(f"[INFO] - Processing messages: [{i+1}/{message_count}]", end='\r')
        sys.stdout.flush()
    
    # Close the database connection
    conn.close()

    # Construct video from image sequence
    output_video = "output_video.mp4"
    create_video_from_images(frames_folder, output_video)

    # TODO(cardboardcode): Keep or remove frames folder content based on --save-images flag.

