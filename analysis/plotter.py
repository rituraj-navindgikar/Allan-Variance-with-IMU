import rclpy
from rclpy.node import Node
from imu_msgs.msg import IMUmsg  # Import your custom message
import math
import numpy as np
import matplotlib.pyplot as plt

class IMUDataAnalyzer(Node):
    def __init__(self):
        super().__init__('imu_data_analyzer')
        self.subscription = self.create_subscription(
            IMUmsg,
            '/imu',
            self.listener_callback,
            10)
        
        self.imu_data = []
        self.received_messages = 0
        self.target_messages = 37000

    def listener_callback(self, msg):
        if self.received_messages < self.target_messages:
            self.received_messages += 1
            self.get_logger().info(f'Received {self.received_messages}/{self.target_messages} messages.')

            # Extract IMU data with a correct timestamp
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # Convert nanoseconds to seconds
            if self.received_messages == 1:
                self.start_time = timestamp  # Record the start time for normalization
            timestamp -= self.start_time  # Normalize the timestamp to start from zero

            
            yaw, pitch, roll = self.quaternion_to_euler(msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w)
            self.imu_data.append({
                'timestamp': timestamp,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll,
                'acc_x': msg.imu.linear_acceleration.x,
                'acc_y': msg.imu.linear_acceleration.y,
                'acc_z': msg.imu.linear_acceleration.z,
                'gyro_x': msg.imu.angular_velocity.x,
                'gyro_y': msg.imu.angular_velocity.y,
                'gyro_z': msg.imu.angular_velocity.z,
                'mag_x': msg.mag_field.magnetic_field.x,
                'mag_y': msg.mag_field.magnetic_field.y,
                'mag_z': msg.mag_field.magnetic_field.z
            })

            if self.received_messages >= self.target_messages:
                self.analyze_data()

    def quaternion_to_euler(self, x, y, z, w):
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw, pitch, roll

    def plot_time_series(self, field, ylabel, unit):
        """ Plot time series data for a given field with time from the header """
        timestamps = [entry['timestamp'] for entry in self.imu_data]
        values = [entry[field] for entry in self.imu_data]

        plt.figure()
        plt.plot(timestamps, values, color='blue', label='f{ylabel}')
        plt.xlabel('Time (seconds)', fontsize=25)
        
        plt.ylabel(f'{ylabel} ({unit})', fontsize=25)

        plt.xticks(np.arange(0, 1000, 125), fontsize=20)
        plt.yticks(fontsize=20)
        plt.xlim(0, 900)


        # plt.title(f'Time v/s {ylabel}')
        
        # graph Y axis min max adjusting factors for each

        if ylabel == "Yaw":
            # Yaw
            plt.ylim(-0.3500, -0.3325)  # prefect
            plt.title(f'Time v/s {ylabel}', fontsize=40)
        elif ylabel == "Pitch":
            # Pitch
            plt.ylim(-0.00200, 0.0008)  
            plt.title(f'Time v/s {ylabel}', fontsize=40)
        elif ylabel == "Roll":
            # Roll
            plt.ylim(-0.0150, -0.0100)  # perfect 
            plt.title(f'Time v/s {ylabel}', fontsize=40)
        elif ylabel == "Acc X":
            # Acc X
            plt.ylim(-0.15, 0.150) # perfect
            plt.title(f'Time v/s Acceleration in X', fontsize=40)
        elif ylabel == "Acc Y":
            # Acc Y
            plt.ylim(0, 0.25)     # perfect
            plt.title(f'Time v/s Acceleration in Y', fontsize=40)
        elif ylabel == "Acc Z":
            # Acc Z
            plt.ylim(-10.2, -9.7)  #perfect
            plt.title(f'Time v/s Acceleration in Z', fontsize=40)
        elif ylabel == "Gyro X":
            # Gyro X
            plt.ylim(-0.012, 0.012)  # perfect
            plt.title(f'Time v/s Gyro in X', fontsize=40)
        elif ylabel == "Gyro Y":
            # Gyro Y
            plt.ylim(-0.012, 0.012)  # perfect
            plt.title(f'Time v/s Gyro in Y', fontsize=40)
        elif ylabel == "Gyro Z":
            # Gyro Z
            plt.ylim(-0.012, 0.012)  # prefect
            plt.title(f'Time v/s Gyro in Z', fontsize=40)
        elif ylabel == "Mag X":
            # Mag X
            plt.ylim(-3.800*1e-5, -3.300*1e-5)  # perfect
            plt.title(f'Time v/s Magnetic Field in X', fontsize=40)
        elif ylabel == "Mag Y":
            # Mag Y
            plt.ylim(-3.4*1e-5, -2.2*1e-5)  # prefect
            plt.title(f'Time v/s Magnetic Field in Y', fontsize=40)
        elif ylabel == "Mag Z":
            # Mag Z
            plt.ylim(0.45*1e-5, 2.00*1e-5)  # prefect
            plt.title(f'Time v/s Magnetic Field in Z', fontsize=40)

        
        plt.grid()
        plt.show()


    def plot_histogram(self, field, ylabel, unit):
        """ Plot histogram of a given field with outlines on bars """
        values = [entry[field] for entry in self.imu_data]
        
        plt.figure()
        plt.hist(values, bins=20, alpha=0.6, color='g', edgecolor='black', linewidth=1)
        plt.xlabel(f'{ylabel} ({unit})')
        plt.ylabel("Frequency")
        plt.title(f'Histogram of {ylabel} values')
        plt.grid()
        plt.show()

        # Calculate and print the mean and standard deviation
        mean = np.mean(values)
        std_dev = np.std(values)
        print(f'Mean of {ylabel}: {mean}')
        print(f'Standard Deviation of {ylabel}: {std_dev}')



    def analyze_data(self):
        """ Analyze the collected IMU data """
        self.get_logger().info('Data collection complete. Analyzing data...')

        # Plot yaw, pitch, roll
        self.plot_time_series('yaw', 'Yaw', 'radian')
        self.plot_time_series('pitch', 'Pitch', 'radian')
        self.plot_time_series('roll', 'Roll', 'radian')

        # Plot accelerometer, gyroscope, and magnetometer data
        for field, label, unit in [('acc_x', 'Acc X', 'meters/sec²'), ('acc_y', 'Acc Y', 'meters/sec²'), ('acc_z', 'Acc Z', 'meters/sec²'),
                                   ('gyro_x', 'Gyro X', 'radians/sec'), ('gyro_y', 'Gyro Y', 'radians/sec'), ('gyro_z', 'Gyro Z', 'radians/sec'),
                                   ('mag_x', 'Mag X', 'Tesla'), ('mag_y', 'Mag Y', 'Tesla'), ('mag_z', 'Mag Z', 'Tesla')]:
            self.plot_time_series(field, label, unit)

        # Histograms
        for field, label, unit in [('yaw', 'Yaw', 'radian'), ('pitch', 'Pitch', 'radian'), ('roll', 'Roll', 'radian'),
                                   ('acc_x', 'Acc X', 'meters/sec²'), ('acc_y', 'Acc Y', 'meters/sec²'), ('acc_z', 'Acc Z', 'meters/sec²'),
                                   ('gyro_x', 'Gyro X', 'radians/sec'), ('gyro_y', 'Gyro Y', 'radians/sec'), ('gyro_z', 'Gyro Z', 'radians/s'),
                                   ('mag_x', 'Mag X', 'T'), ('mag_y', 'Mag Y', 'T'), ('mag_z', 'Mag Z', 'T')]:
            self.plot_histogram(field, label, unit)

def main(args=None):
    rclpy.init(args=args)
    imu_data_analyzer = IMUDataAnalyzer()
    rclpy.spin(imu_data_analyzer)
    imu_data_analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# scale values time vs value graph

# Yaw      -0.3385, -0.3410, -0.3510, -0.3290
# Pitch    -0.0003, -0.0009, -0.00005, -0.0001
# Roll     -0.0118, -0.0132, -0.0150, -0.0100
# Acc X    -0.07,   0.07,   -0.10,   0.10
# Acc Y    0.08,    0.18,    0.05,    0.20
# Acc Z    -10.00,  -9.90,  -11.00,  -9.80
# Gyro X   -0.025,  0.0025, -0.030,   0.005
# Gyro Y   -0.025,  0.0025, -0.030,   0.005
# Gyro Z   -0.025,  0.0025, -0.030,   0.005
# Mag X    -3.550,  -3.425, -3.600,  -3.400
# Mag Y    -3.00,   -2.85,  -3.10,   -2.80
# Mag Z    0.50,    1.45,   0.45,    1.50





# scale values histogram

# Yaw      0, 1400
# Pitch    0, 4000
# Roll     0, 3000
# Acc X    0, 15.89
# Acc Y    0, 28
# Acc Z    0, 22.5
# Gyro X   0, 650
# Gyro Y   0, 550
# Gyro Z   0, 620
# Mag X    0, 3
# Mag Y    0, 1.6
# Mag Z    0, 700000

