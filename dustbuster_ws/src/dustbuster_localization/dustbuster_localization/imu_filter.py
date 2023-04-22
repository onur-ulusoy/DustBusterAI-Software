import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.signal import butter, filtfilt


def butter_highpass(cutoff, fs, order=1):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a


def apply_highpass_filter(data, cutoff, fs, order=1):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

class ImuFilter(Node):

    def __init__(self):
        super().__init__('imu_filter')
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.listener_callback, 10)
        self.imu_filtered_pub = self.create_publisher(
            Imu, 'imu/data_filtered', 10)
        self.accel_history = np.zeros((3, 10))  # Create a buffer for the last 10 IMU readings
        self.fs = 1000.0  # Adjust this value according to the IMU data rate
        self.cutoff = 0.3  # Adjust this value based on desired cutoff frequency

    def listener_callback(self, msg):
        current_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Shift the buffer and add the new acceleration values
        self.accel_history[:, :-1] = self.accel_history[:, 1:]
        self.accel_history[:, -1] = current_accel

        # Apply the high-pass filter
        filtered_accel = np.zeros(3)
        for i in range(3):
            filtered_accel[i] = apply_highpass_filter(
                self.accel_history[i], self.cutoff, self.fs)

        msg.linear_acceleration.x = filtered_accel[0]
        msg.linear_acceleration.y = filtered_accel[1]
        msg.linear_acceleration.z = filtered_accel[2]

        self.imu_filtered_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    imu_filter = ImuFilter()
    rclpy.spin(imu_filter)
    imu_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
