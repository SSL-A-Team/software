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


class BagToCSVConverter(Node):
    # TODO add a catch statement on the get param to tell the user (and remind me) how to give a param override because they are annoying in ros2
    def __init__(self):
        super().__init__("RosBagToCSVConverter", automatically_declare_parameters_from_overrides=True)
        self.reader = rosbag2_py.SequentialReader()

        # self.declare_parameter('bag_path', "")
        # param_bag_path = "/home/collin/documents/ateam/software_ws/rosbag2_2023_04_10-19_24_55/rosbag2_2023_04_10-19_24_55_0.db3"

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
        # get_message takes the string and finds the MsgType object (message definition) for it from your ros env
        # so really just make ourselves a lookup table of topic name to MsgType/defs not message type strings ahead of time for deserialization


        all_topics_count = 0 
        for topic_name, msg_type in topics_table.items():
            output_csv_path = os.path.join(output_dir, topic_name.replace("/", "") + ".csv")
            print(output_csv_path)

            filter = rosbag2_py.StorageFilter([topic_name])
            self.reader.set_filter(filter)

            with open(output_csv_path, 'w') as output_csv:
                i = 0
                # use the first message to flatten and write out the header for the message because it was easier
                if self.reader.has_next():
                    msg_topic, s_msg, timestamp = self.reader.read_next()

                    msg = deserialize_message(s_msg, msg_type)
                    flat_fields = flatdict.FlatDict(message_to_ordereddict(msg), delimiter='.').keys()
                    header = ','.join(list(flat_fields))
                    output_csv.write(header + "\n")

                    msg_csv = message_to_csv(msg)
                    output_csv.write(msg_csv + "\n")
                    i += 1


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