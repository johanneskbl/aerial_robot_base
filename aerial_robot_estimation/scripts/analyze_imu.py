# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""
Created by li-jinjie on 24-7-28.
"""

import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse

legend_alpha = 0.5


def main(file_path, type):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    # ======= Accelerometer =========
    data_acc_b = data[["__time", "/beetle1/imu/acc_data[0]", "/beetle1/imu/acc_data[1]", "/beetle1/imu/acc_data[2]"]]
    data_acc_b = data_acc_b.dropna()

    # ======= Gyroscope =========
    data_gyro_b = data[
        ["__time", "/beetle1/imu/gyro_data[0]", "/beetle1/imu/gyro_data[1]", "/beetle1/imu/gyro_data[2]"]
    ]
    data_gyro_b = data_gyro_b.dropna()

    # ======= Plotting =========
    if type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # Default is 10
        label_size = 14

        fig = plt.figure(figsize=(14, 10))

        t_bias = max(data_acc_b["__time"].iloc[0], data_acc_b["__time"].iloc[0])
        color_ref = "#0C5DA5"
        color_real = "#0C5DA5"  # '#FF2C00'

        # --------------------------------
        plt.subplot(3, 2, 1)
        t = np.array(data_acc_b["__time"]) - t_bias
        acc_b_x = np.array(data_acc_b["/beetle1/imu/acc_data[0]"])
        plt.plot(t, acc_b_x, label="acc_b_x", color=color_real)

        plt.legend(framealpha=legend_alpha)
        # Plt.ylabel('X (m)', fontsize=label_size)

        # --------------------------------
        plt.subplot(3, 2, 2)
        # Fft of acc_b_x
        acc_b_x = acc_b_x - np.mean(acc_b_x)  # Remove the bias
        N = len(acc_b_x)
        T = t[-1] / N
        f = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        yf = np.fft.fft(acc_b_x)
        xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        plt.plot(xf, 2.0 / N * np.abs(yf[0 : N // 2]), label="fft of acc_b_x (mean removed)", color=color_real)

        plt.ylabel("Magnitude", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.subplot(3, 2, 3)
        t = np.array(data_acc_b["__time"]) - t_bias
        acc_b_y = np.array(data_acc_b["/beetle1/imu/acc_data[1]"])
        plt.plot(t, acc_b_y, label="acc_b_y", color=color_real)

        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.subplot(3, 2, 4)
        # Fft of acc_b_y
        acc_b_y = acc_b_y - np.mean(acc_b_y)  # Remove the bias
        N = len(acc_b_y)
        T = t[-1] / N
        f = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        yf = np.fft.fft(acc_b_y)
        xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        plt.plot(xf, 2.0 / N * np.abs(yf[0 : N // 2]), label="fft of acc_b_y (mean removed)", color=color_real)

        plt.ylabel("Magnitude", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.subplot(3, 2, 5)
        t = np.array(data_acc_b["__time"]) - t_bias
        acc_b_z = np.array(data_acc_b["/beetle1/imu/acc_data[2]"])
        plt.plot(t, acc_b_z, label="acc_b_z", color=color_real)

        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.subplot(3, 2, 6)
        # Fft of acc_b_z
        acc_b_z = acc_b_z - np.mean(acc_b_z)  # Remove the bias
        N = len(acc_b_z)
        T = t[-1] / N
        f = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        yf = np.fft.fft(acc_b_z)
        xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        plt.plot(xf, 2.0 / N * np.abs(yf[0 : N // 2]), label="fft of acc_b_z (mean removed)", color=color_real)

        plt.ylabel("Magnitude", fontsize=label_size)
        plt.xlabel("Frequency (Hz)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.tight_layout()
        # Make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    if type == 1:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # Default is 10
        label_size = 14

        fig = plt.figure(figsize=(14, 10))

        t_bias = max(data_gyro_b["__time"].iloc[0], data_gyro_b["__time"].iloc[0])
        color_ref = "#0C5DA5"
        color_real = "#0C5DA5"  # '#FF2C00'

        # --------------------------------
        plt.subplot(3, 2, 1)
        t = np.array(data_gyro_b["__time"]) - t_bias
        gyro_b_x = np.array(data_gyro_b["/beetle1/imu/gyro_data[0]"])
        plt.plot(t, gyro_b_x, label="gyro_b_x", color=color_real)

        plt.legend(framealpha=legend_alpha)
        # Plt.ylabel('X (m)', fontsize=label_size)

        # --------------------------------
        plt.subplot(3, 2, 2)
        # Fft of gyro_b_x
        gyro_b_x = gyro_b_x - np.mean(gyro_b_x)  # Remove the bias
        N = len(gyro_b_x)
        T = t[-1] / N
        f = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        yf = np.fft.fft(gyro_b_x)
        xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        plt.plot(xf, 2.0 / N * np.abs(yf[0 : N // 2]), label="fft of gyro_b_x (mean removed)", color=color_real)

        plt.ylabel("Magnitude", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.subplot(3, 2, 3)
        t = np.array(data_gyro_b["__time"]) - t_bias
        gyro_b_y = np.array(data_gyro_b["/beetle1/imu/gyro_data[1]"])
        plt.plot(t, gyro_b_y, label="gyro_b_y", color=color_real)

        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.subplot(3, 2, 4)
        # Fft of gyro_b_y
        gyro_b_y = gyro_b_y - np.mean(gyro_b_y)  # Remove the bias
        N = len(gyro_b_y)
        T = t[-1] / N
        f = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        yf = np.fft.fft(gyro_b_y)
        xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        plt.plot(xf, 2.0 / N * np.abs(yf[0 : N // 2]), label="fft of gyro_b_y (mean removed)", color=color_real)

        plt.ylabel("Magnitude", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.subplot(3, 2, 5)
        t = np.array(data_gyro_b["__time"]) - t_bias
        gyro_b_z = np.array(data_gyro_b["/beetle1/imu/gyro_data[2]"])
        plt.plot(t, gyro_b_z, label="gyro_b_z", color=color_real)

        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.subplot(3, 2, 6)
        # Fft of gyro_b_z
        gyro_b_z = gyro_b_z - np.mean(gyro_b_z)  # Remove the bias
        N = len(gyro_b_z)
        T = t[-1] / N
        f = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        yf = np.fft.fft(gyro_b_z)
        xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        plt.plot(xf, 2.0 / N * np.abs(yf[0 : N // 2]), label="fft of gyro_b_z (mean removed)", color=color_real)

        plt.ylabel("Magnitude", fontsize=label_size)
        plt.xlabel("Frequency (Hz)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)

        # --------------------------------
        plt.tight_layout()
        # Make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    else:
        print("Invalid type")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Analyze the IMU data. Please use plotjuggler to generate the csv file."
    )
    parser.add_argument("file_path", type=str, help="The file name of the trajectory")
    parser.add_argument("--type", type=int, help="The type of the plot", default=0)

    args = parser.parse_args()

    main(args.file_path, args.type)
