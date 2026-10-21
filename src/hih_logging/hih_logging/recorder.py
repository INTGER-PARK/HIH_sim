"""Record independent drone topics and export a CSV for each launch session."""

import argparse
import csv
from datetime import datetime
import gc
from pathlib import Path
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.utilities import remove_ros_args
import rosbag2_py

from .converter import convert_bag
from .topics import CSV_COLUMNS, TOPICS, flatten_message


class CooperationRecorder(Node):
    """Subscribe only; write the same received sample to rosbag and live CSV."""

    def __init__(self, output_dir):
        super().__init__('hih_cooperation_recorder')
        run_name = 'run_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.run_dir = Path(output_dir).expanduser().resolve() / run_name
        self.run_dir.mkdir(parents=True)
        self.bag_dir = self.run_dir / 'rosbag'
        self.csv_path = self.run_dir / 'all_topics.csv'
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_dir), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                        output_serialization_format='cdr'))
        self.csv_stream = self.csv_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_stream, fieldnames=CSV_COLUMNS)
        self.csv_writer.writeheader()
        self.count = 0
        self._record_subscriptions = []

        for name, (message_class, type_name) in TOPICS.items():
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=name, type=type_name, serialization_format='cdr'))
            self._record_subscriptions.append(self.create_subscription(
                message_class, name,
                lambda message, topic=name, ros_type=type_name:
                self.record(topic, ros_type, message), 10))
        self.get_logger().info(f'Recording {len(TOPICS)} topics to {self.run_dir}')

    def record(self, topic, type_name, message):
        # One acquisition timestamp drives both output formats. This plant has
        # headerless messages; t_sec is wall reception time, not MuJoCo time.
        timestamp = time.time_ns()
        self.writer.write(topic, serialize_message(message), timestamp)
        self.csv_writer.writerow({
            't_sec': f'{timestamp / 1e9:.9f}',
            't_ns': timestamp,
            'topic_name': topic,
            'type_name': type_name,
            **flatten_message(message),
        })
        self.count += 1
        if self.count % 100 == 0:
            self.csv_stream.flush()

    def finish_recording(self):
        self.csv_stream.flush()
        self.csv_stream.close()
        # ROS 2 Humble's rosbag2_py writer has no portable close() binding.
        # Dropping the last reference invokes its C++ destructor, which writes
        # metadata.yaml and closes SQLite before the bag reader opens it.
        self.writer = None
        gc.collect()


def main():
    parser = argparse.ArgumentParser(description='Record cooperation bag and CSV')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    # Custom --output-dir is parsed above, so pass only ROS defaults to rclpy.
    rclpy.init(args=[])
    node = None
    try:
        node = CooperationRecorder(args.output_dir)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        if node is not None:
            bag_dir, csv_path = node.bag_dir, node.csv_path
            node.destroy_node()
            node.finish_recording()
            print(f'Bag finalized: {bag_dir}', flush=True)
            # The live CSV already has complete rows. Re-read the finalized bag
            # into a temporary CSV and replace it only if conversion succeeds.
            try:
                count = convert_bag(bag_dir, csv_path)
                print(f'CSV converted from bag: {csv_path} ({count} rows)', flush=True)
            except Exception as exc:
                print(f'Bag conversion failed ({exc}); live CSV kept: {csv_path}',
                      file=sys.stderr, flush=True)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
