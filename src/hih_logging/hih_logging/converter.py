"""Stream a finalized sqlite3 rosbag into one MATLAB-friendly wide CSV."""

import argparse
import csv
import os
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .topics import CSV_COLUMNS, flatten_message


def convert_bag(bag_dir, csv_path):
    """Atomically replace CSV only after the entire bag was read successfully."""
    bag_dir, csv_path = Path(bag_dir), Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = csv_path.with_name(csv_path.name + '.tmp')
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    classes = {name: get_message(type_name) for name, type_name in types.items()}
    count = 0
    try:
        with temporary.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            while reader.has_next():
                topic, serialized, timestamp = reader.read_next()
                message = deserialize_message(serialized, classes[topic])
                writer.writerow({
                    't_sec': f'{timestamp / 1e9:.9f}',
                    't_ns': timestamp,
                    'topic_name': topic,
                    'type_name': types[topic],
                    **flatten_message(message),
                })
                count += 1
        os.replace(temporary, csv_path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return count


def main():
    parser = argparse.ArgumentParser(description='Export a HandinHand rosbag to CSV')
    parser.add_argument('bag', type=Path, help='Session rosbag directory')
    parser.add_argument('--csv', type=Path, help='Output path (default: bag sibling)')
    args = parser.parse_args()
    csv_path = args.csv or args.bag.parent / 'all_topics.csv'
    count = convert_bag(args.bag, csv_path)
    print(f'Converted {count} messages to {csv_path}')


if __name__ == '__main__':
    main()
