#!/usr/bin/env python3

import argparse

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def parse_mapping(mappings):
    mapping_dict = {}

    for mapping in mappings:
        if "=" not in mapping:
            raise ValueError(
                f"Invalid mapping '{mapping}'. Expected format: <key>=<value>"
            )

        key, value = mapping.split("=", 1)
        mapping_dict[key] = value

    return mapping_dict


def add_namespace(namespace, topic):
    if not namespace:
        return topic

    namespace = namespace.rstrip("/")

    if topic.startswith("/"):
        return f"{namespace}{topic}"

    return f"{namespace}/{topic}"


def remap_frame(frame_id, frame_map):
    return frame_map.get(frame_id, frame_id)


def main():
    parser = argparse.ArgumentParser(
        description="Copy a ROS 2 bag while renaming topics and remapping frame IDs."
    )

    parser.add_argument(
        "--input",
        required=True,
        help="Input bag directory"
    )

    parser.add_argument(
        "--output",
        required=True,
        help="Output bag directory"
    )

    parser.add_argument(
        "--namespace",
        default="",
        help="Namespace to prepend to every topic (e.g. /uav33)"
    )

    parser.add_argument(
        "--map",
        nargs="*",
        default=[],
        metavar="TOPIC=FRAME_ID",
        help="Set header.frame_id for specific topics."
    )

    parser.add_argument(
        "--frame-map",
        nargs="*",
        default=[],
        metavar="OLD=NEW",
        help="Remap frame IDs."
    )

    parser.add_argument(
        "--topic-map",
        nargs="*",
        default=[],
        metavar="OLD=NEW",
        help="Rename topics before applying the namespace."
    )

    args = parser.parse_args()

    topic_to_frame = parse_mapping(args.map)
    frame_map = parse_mapping(args.frame_map)
    topic_map = parse_mapping(args.topic_map)

    storage_options = rosbag2_py.StorageOptions(
        uri=args.input,
        storage_id="sqlite3"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=args.output,
            storage_id="sqlite3"
        ),
        converter_options
    )

    topics = reader.get_all_topics_and_types()

    type_map = {}
    topic_name_map = {}

    for topic in topics:

        mapped_topic = topic_map.get(topic.name, topic.name)
        new_topic_name = add_namespace(args.namespace, mapped_topic)

        topic_name_map[topic.name] = new_topic_name

        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=new_topic_name,
                type=topic.type,
                serialization_format=topic.serialization_format,
                offered_qos_profiles=topic.offered_qos_profiles
            )
        )

        type_map[topic.name] = get_message(topic.type)

    while reader.has_next():

        topic_name, data, timestamp = reader.read_next()

        msg_type = type_map[topic_name]
        msg = deserialize_message(data, msg_type)

        #
        # Update header.frame_id
        #
        if hasattr(msg, "header"):

            if topic_name in topic_to_frame:
                msg.header.frame_id = topic_to_frame[topic_name]
            else:
                msg.header.frame_id = remap_frame(
                    msg.header.frame_id,
                    frame_map
                )

        #
        # Update child_frame_id
        #
        if hasattr(msg, "child_frame_id"):
            msg.child_frame_id = remap_frame(
                msg.child_frame_id,
                frame_map
            )

        data = serialize_message(msg)

        writer.write(
            topic_name_map[topic_name],
            data,
            timestamp
        )


if __name__ == "__main__":
    main()
