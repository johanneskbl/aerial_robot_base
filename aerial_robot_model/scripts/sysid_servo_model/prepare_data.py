# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""
created by Jinjie LI, 2024/02/06
"""

import argparse
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from send_cmd import kondo_pos_2_rad
from spinal.msg import ServoControlCmd, ServoStates


class DataStore:
    def __init__(self, number):
        self.timestamp_list: list[float] = []
        self.data_0_list: list[float] = []
        self.data_1_list: list[float] = []
        self.data_2_list: list[float] = []
        self.data_3_list: list[float] = []

        self.inter_timestamp_list: np.ndarray = np.array([])
        self.inter_data_0_list: np.ndarray = np.array([])
        self.inter_data_1_list: np.ndarray = np.array([])
        self.inter_data_2_list: np.ndarray = np.array([])
        self.inter_data_3_list: np.ndarray = np.array([])


MESSAGE_TYPE_MAP = {
    "sensor_msgs/msg/JointState": JointState,
    "spinal_msgs/msg/ServoStates": ServoStates,
    "spinal_msgs/msg/ServoControlCmd": ServoControlCmd,
}


def read_bag_messages(bag_file, topic_names):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing_topics = [topic_name for topic_name in topic_names if topic_name not in topic_types]
    if missing_topics:
        raise ValueError(f"Missing expected topics in bag: {', '.join(missing_topics)}")

    messages = []
    while reader.has_next():
        topic_name, serialized_msg, timestamp = reader.read_next()
        if topic_name not in topic_names:
            continue

        msg_type_name = topic_types.get(topic_name)
        if msg_type_name is None:
            raise ValueError(f"Missing message type information for topic '{topic_name}'")

        msg_type = MESSAGE_TYPE_MAP.get(msg_type_name)
        if msg_type is None:
            raise ValueError(f"Unsupported ROS 2 message type '{msg_type_name}' for topic '{topic_name}'")

        messages.append((topic_name, deserialize_message(serialized_msg, msg_type), timestamp * 1e-9))

    return messages


def process_data(file_name, is_kondo_pos, if_save_csv, time_start, time_span, freq_inter):
    time_end = time_start + time_span

    # Initialize a list to store the data
    cmd_data = DataStore(4)
    real_data = DataStore(4)

    # Specify the ROS bag file and topic name
    bag_file = file_name
    if is_kondo_pos:
        joint_states_topic_name = "/kondo_servo/states_real"
        gimbals_ctrl_topic_name = "/kondo_servo/states_cmd"
    else:
        joint_states_topic_name = "/beetle1/joint_states"
        gimbals_ctrl_topic_name = "/beetle1/gimbals_ctrl"

    # Iterate through messages in the ROS 2 bag
    for topic, msg, timestamp in read_bag_messages(bag_file, [joint_states_topic_name, gimbals_ctrl_topic_name]):
        if topic == gimbals_ctrl_topic_name:
            cmd_data.timestamp_list.append(timestamp)
            for i in range(4):
                if is_kondo_pos:
                    getattr(cmd_data, f"data_{i}_list").append(kondo_pos_2_rad(msg.angles[i]))
                else:
                    getattr(cmd_data, f"data_{i}_list").append(msg.position[i])

        if topic == joint_states_topic_name:
            real_data.timestamp_list.append(timestamp)
            for i in range(4):
                if is_kondo_pos:
                    getattr(real_data, f"data_{i}_list").append(kondo_pos_2_rad(msg.servos[i].angle))
                else:
                    getattr(real_data, f"data_{i}_list").append(msg.position[i])

    if not cmd_data.timestamp_list or not real_data.timestamp_list:
        raise ValueError("No data found for one or both expected topics in the bag")

    # Find the max first timestamp
    timestamp_start = max(cmd_data.timestamp_list[0], real_data.timestamp_list[0])

    # Normalize the timestamp
    cmd_data.timestamp_list = [t - timestamp_start for t in cmd_data.timestamp_list]
    real_data.timestamp_list = [t - timestamp_start for t in real_data.timestamp_list]

    # Interpolate the data
    cmd_data.inter_timestamp_list = np.linspace(time_start, time_end, freq_inter * (time_end - time_start))
    cmd_data.inter_data_0_list = np.interp(cmd_data.inter_timestamp_list, cmd_data.timestamp_list, cmd_data.data_0_list)
    cmd_data.inter_data_1_list = np.interp(cmd_data.inter_timestamp_list, cmd_data.timestamp_list, cmd_data.data_1_list)
    cmd_data.inter_data_2_list = np.interp(cmd_data.inter_timestamp_list, cmd_data.timestamp_list, cmd_data.data_2_list)
    cmd_data.inter_data_3_list = np.interp(cmd_data.inter_timestamp_list, cmd_data.timestamp_list, cmd_data.data_3_list)

    real_data.inter_timestamp_list = np.linspace(time_start, time_end, freq_inter * (time_end - time_start))
    real_data.inter_data_0_list = np.interp(
        real_data.inter_timestamp_list, real_data.timestamp_list, real_data.data_0_list
    )
    real_data.inter_data_1_list = np.interp(
        real_data.inter_timestamp_list, real_data.timestamp_list, real_data.data_1_list
    )
    real_data.inter_data_2_list = np.interp(
        real_data.inter_timestamp_list, real_data.timestamp_list, real_data.data_2_list
    )
    real_data.inter_data_3_list = np.interp(
        real_data.inter_timestamp_list, real_data.timestamp_list, real_data.data_3_list
    )

    if if_save_csv:
        time_now = time.strftime("%Y%m%d-%H%M%S")
        with open(f"{time_now}_inter_gimbals_ctrl.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            for i in range(len(cmd_data.inter_timestamp_list)):
                writer.writerow(
                    [
                        cmd_data.inter_timestamp_list[i],
                        cmd_data.inter_data_0_list[i],
                        cmd_data.inter_data_1_list[i],
                        cmd_data.inter_data_2_list[i],
                        cmd_data.inter_data_3_list[i],
                    ]
                )
                print(
                    f"gimbals_ctrl: {cmd_data.inter_timestamp_list[i]}, {cmd_data.inter_data_0_list[i]}, {cmd_data.inter_data_1_list[i]}, {cmd_data.inter_data_2_list[i]}, {cmd_data.inter_data_3_list[i]}"
                )

        with open(f"{time_now}_inter_joint_states.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            for i in range(len(real_data.inter_timestamp_list)):
                writer.writerow(
                    [
                        real_data.inter_timestamp_list[i],
                        real_data.inter_data_0_list[i],
                        real_data.inter_data_1_list[i],
                        real_data.inter_data_2_list[i],
                        real_data.inter_data_3_list[i],
                    ]
                )
                print(
                    f"joint_states: {real_data.inter_timestamp_list[i]}, {real_data.inter_data_0_list[i]}, {real_data.inter_data_1_list[i]}, {real_data.inter_data_2_list[i]}, {real_data.inter_data_3_list[i]}"
                )

    # Now you can work with the data as needed
    # plot the curves
    # draw a figure with four subplots, each subplot is two curves with cmd and real data
    fig, axs = plt.subplots(2, 2, figsize=(25, 10))
    plt.tight_layout(pad=3.0)
    for i in range(4):
        axs[i // 2, i % 2].plot(real_data.timestamp_list, getattr(real_data, f"data_{i}_list"), label="real")
        axs[i // 2, i % 2].plot(cmd_data.timestamp_list, getattr(cmd_data, f"data_{i}_list"), label="cmd")
        axs[i // 2, i % 2].set_title(f"Joint {i} data -- original")
        axs[i // 2, i % 2].set_xlabel("Time (s)")
        axs[i // 2, i % 2].set_ylabel("Position")
        axs[i // 2, i % 2].legend()
        axs[i // 2, i % 2].grid(True)
    plt.show()

    # Draw a figure with four subplots, each subplot is two curves with interpolated cmd and real data
    fig, axs = plt.subplots(2, 2, figsize=(25, 10))
    plt.tight_layout(pad=3.0)
    for i in range(4):
        axs[i // 2, i % 2].plot(cmd_data.inter_timestamp_list, getattr(cmd_data, f"inter_data_{i}_list"), label="cmd")
        axs[i // 2, i % 2].plot(
            real_data.inter_timestamp_list, getattr(real_data, f"inter_data_{i}_list"), label="real"
        )
        axs[i // 2, i % 2].set_title(f"Joint {i} data -- interpolated")
        axs[i // 2, i % 2].set_xlabel("Time (s)")
        axs[i // 2, i % 2].set_ylabel("Position")
        axs[i // 2, i % 2].legend()
        axs[i // 2, i % 2].grid(True)
    plt.show()

    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process and visualize data from a rosbag file.")
    parser.add_argument("file_name", help="Name of the text file containing the data")
    parser.add_argument("is_kondo_pos", help="Whether the data is in kondo position", type=int)
    parser.add_argument("-c", "--if_csv", help="Whether to save the data to csv", default=0, type=int)
    parser.add_argument("-t", "--t_start", help="The start time of the data", default=0, type=int)
    parser.add_argument("-p", "--t_span", help="The time span of the data", default=3, type=int)
    parser.add_argument("-f", "--freq_inter", help="The frequency of interpolation", default=100, type=int)
    args = parser.parse_args()

    process_data(
        args.file_name,
        bool(args.is_kondo_pos),
        bool(args.if_csv),
        int(args.t_start),
        int(args.t_span),
        int(args.freq_inter),
    )
