import sqlite3
import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from PIL import Image

def save_image_from_rosbag(db_path, topic_name, message_index=0):
    print(f"PROCESS STARTED")
    # Connect to the database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    print(f"CONNECTED TO DATABASE")
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
        print(f"No message found at index {message_index} for topic {topic_name}")
        return
    
    # Deserialize the sensor_msgs/msg/Image message
    msg_type = get_message('sensor_msgs/msg/Image')
    msg = deserialize_message(message_data[0], msg_type)
    # msg = msg_type.deserialize(message_data[0])
    
    # Convert the message to a dict for easier manipulation
    msg_dict = message_to_ordereddict(msg)
    
    # Extract image information
    height = msg_dict['height']
    width = msg_dict['width']
    encoding = msg_dict['encoding']
    is_bigendian = msg_dict['is_bigendian']
    step = msg_dict['step']
    data = msg_dict['data']

    data_bytes = bytes(data)

    
    # Convert to a numpy array
    image_array = np.frombuffer(data_bytes, dtype=np.uint8)
    image_array = image_array.reshape((height, width, -1))  # Assumes 3 channels (RGB or BGR)
    
    # Check encoding and convert if necessary
    if encoding == 'bgr8':
        image_array = image_array[:, :, ::-1]  # Convert BGR to RGB
    
    # Save the image using Pillow
    img = Image.fromarray(image_array)
    img.save("output_image.png")
    
    print(f"Image saved as output_image.png")
    
    # Close the database connection
    conn.close()

if __name__ == "__main__":
    db_path = "rosbag2_2024_10_11-19_45_28/rosbag2_2024_10_11-19_45_28_0.db3"
    topic_name = "/virtual_camera/image_raw"
    message_index = 0

    save_image_from_rosbag(db_path, topic_name, message_index)

