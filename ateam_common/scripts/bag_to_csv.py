#! /usr/bin/env python3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_csv
import rosbag2_py
import argparse
import csv


primitive_field_types = ['boolean', 'octet', 'char', 'float', 'double',
                         'short', 'unsigned short', 'long', 'unsigned long',
                         'long long', 'unsigned long long', 'string',
                         'wstring', 'int8', 'uint8', 'int16', 'uint16',
                         'int32', 'uint32', 'int64', 'uint64']


def open_bag(path, storage_id):
    storage_options = rosbag2_py.StorageOptions(uri=path, 
                                                storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def parse_array_type(array_type_name):
    open_bracket_pos = array_type_name.index('[')
    close_bracket_pos = array_type_name.index(']')
    array_size = int(array_type_name[open_bracket_pos+1:close_bracket_pos])
    type_name = array_type_name[0:open_bracket_pos]
    return (type_name, array_size)


def get_column_headers_for_type(namespace, type_name):
    # print(f"get_column_headers_for_type({namespace},{type_name})")
    if type_name in primitive_field_types:
        return [namespace]

    if type_name.startswith('sequence<'):
        print("Warning: Omitting dynamic length sequence: " + namespace)
        return []
    
    headers = []

    if type_name[-1] == ']':
        non_array_type_name, array_size = parse_array_type(type_name)
        for index in range(array_size):
            headers.extend(get_column_headers_for_type(namespace + '[' + str(index) + ']', non_array_type_name))
        return headers

    msg_type = get_message(type_name)
    field_details = msg_type.get_fields_and_field_types()

    for field_name, field_type in field_details.items():
        headers.extend(get_column_headers_for_type(namespace+'/'+field_name, field_type))
    return headers


def get_column_headers(topic_details):
    headers = ['time']
    for topic in topic_details:
        headers.extend(get_column_headers_for_type(topic.name, topic.type))
    return headers


def set_row_field(row_data, column_headers, namespace, type_name, value):
    if type_name in primitive_field_types:
        row_data[column_headers.index(namespace)] = value
        return

    if type_name.startswith('sequence<'):
        return

    if type_name[-1] == ']':
        non_array_type_name, array_size = parse_array_type(type_name)
        for index in range(array_size):
            new_namespace = namespace+'['+str(index)+']'
            value_at_index = value[index]
            set_row_field(row_data, column_headers, new_namespace, non_array_type_name, value_at_index)
        return

    for (field_name, field_type) in value.get_fields_and_field_types().items():
        set_row_field(row_data, column_headers, namespace+'/'+field_name, field_type, getattr(value, field_name))


def write_message(csv_writer, column_headers, topic, msg_data, time, msg_type_name):
    row_data = ['' for _ in column_headers]
    row_data[0] = time
    msg = deserialize_message(msg_data, get_message(msg_type_name))
    set_row_field(row_data, column_headers, topic, msg_type_name, msg)
    csv_writer.writerow(row_data)


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('bag_path', help='Path to bag folder')
    arg_parser.add_argument('-s', '--storage', default='sqlite3', type=str, help='ID for the storage plugin to be used. Defaults to sqlite3.')
    arg_parser.add_argument('-t', '--topics', default='', type=str, nargs='+', help='Filters which topics are saved to the CSV. By default all topics are included')
    arg_parser.add_argument('-o', '--output', default='output.csv', type=str, help="Name of the output file")
    args = arg_parser.parse_args()
    bag_reader = open_bag(args.bag_path, args.storage)
    if len(args.topics) > 0:
        storage_filter = rosbag2_py.StorageFilter(topics=args.topics)
        bag_reader.set_filter(storage_filter)
    topic_details = bag_reader.get_all_topics_and_types()
    if len(args.topics) > 0:
        topic_details = [topic_details[i] for i in range(len(topic_details)) if topic_details[i].name in args.topics]
    topic_type_map = {topic_details[i].name: topic_details[i].type for i in range(len(topic_details))}
    column_headers = get_column_headers(topic_details)
    with open(args.output, 'w') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(column_headers)
        while bag_reader.has_next():
            (topic, msg_data, time) = bag_reader.read_next()
            write_message(csv_writer, column_headers, topic, msg_data, time, topic_type_map[topic])
