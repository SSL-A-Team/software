import rclpy
import os
import shutil
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py import message_to_ordereddict

from rclpy.parameter import Parameter

import flatdict

import rosbag2_py

I 
# TODO add a catch statement on the get param to tell the user (and remind me) how to give a param override because they are annoying in ros2
# ros2 run bag_to_csv_converter bag_to_csv_converter --ros-args -p bag_path:="..."
class BagToCSVConverter(Node):
    def __init__(self):
        super().__init__("RosBagToCSVConverter", automatically_declare_parameters_from_overrides=True)
        self.reader = rosbag2_py.SequentialReader()

        param_bag_path = self.get_parameter('bag_path').value
        print(f"Path for bag {param_bag_path}")
        bag_dir = os.path.dirname(param_bag_path)

        # should I always wipe it?
        output_dir = os.path.join(bag_dir, "csvs")
        os.makedirs(os.path.join(bag_dir, "csvs"), exist_ok=True)

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=param_bag_path,
            storage_id='')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)

        topics_meta = self.reader.get_all_topics_and_types()
        topics_table = {topic_meta.name : get_message(topic_meta.type) for topic_meta in topics_meta} 
        # get_message takes the string for message type and finds the MsgType object (message definition) for it from your ros env
        # so really just make ourselves a lookup table of topic name to MsgType/defs not message type strings ahead of time for deserialization
        # IMPORTANT this does require you have built the messages for any topic in the bag file
        # Rolling ros2 has updates which store the defs in the rosbag files but we are on humble so...


        all_topics_count = 0 
        # for every topic we pulled out of the meta data we start a csv file for it. If 
        for topic_name, msg_type in topics_table.items():
            output_csv_path = os.path.join(output_dir, topic_name.replace("/", "") + ".csv")
            print(output_csv_path)

            filter = rosbag2_py.StorageFilter([topic_name])
            self.reader.set_filter(filter)

            # Only if there is one message of that type do we want to even make a file for it. 
            # Unlikely as its in the meta data table but something I saw when I was looking at this stuff because apparently this can happen if a bag is filtered
            # Also we use the first message for metadata
            i = 0
            if self.reader.has_next():
                with open(output_csv_path, 'w') as output_csv:
                    # Use the first messages ordered dict then flattened to write out the header for this topics csv 
                    msg_topic, s_msg, timestamp = self.reader.read_next()

                    msg = deserialize_message(s_msg, msg_type)
                    flat_fields = flatdict.FlatDict(message_to_ordereddict(msg), delimiter='.').keys()
                    header = ','.join(list(flat_fields))
                    output_csv.write(header + "\n")

                    msg_csv = message_to_csv(msg)
                    output_csv.write(msg_csv + "\n")
                    i += 1

                    # read and convert the remaining messages 
                    while self.reader.has_next():
                        msg_topic, s_msg, timestamp = self.reader.read_next()
                        msg = deserialize_message(s_msg, msg_type)
                        msg_csv = message_to_csv(msg)
                        output_csv.write(msg_csv + "\n")
                        i += 1

            print(f"number of messages for topic {topic_name}: {i}")
            all_topics_count += i

            self.reader.reset_filter()
            self.reader.seek(0)
            print()

        # TODO check the metadata.yaml to make sure these are right
        print(f"Total number of messages in the bagfile: {all_topics_count}")


def main(args=None):
    rclpy.init(args=args)
    node = BagToCSVConverter()
    rclpy.shutdown()


if __name__ == '__main__':
    main()