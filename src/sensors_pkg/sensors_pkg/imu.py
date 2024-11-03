import rclpy
from rclpy.node import Node
from vnpy import VnSensor
from sensor_msgs.msg import Imu

class IMU(Node):

    def __init__(self):
        super().__init__('imu_node')

        # Create VnSensor Object
        self.s = VnSensor()

        # Connect imu from usb bus connection
        self.s.connect('/dev/ttyUSB0', 115200)

        # Publish imu data every 0.5 second
        self.publisher_= self.create_publisher(Imu, 'imu', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Callback funciton that publishes imu data constantly"""
        imu_msg = Imu()

        # Get Quaternion Data
        wxyz = self.s.read_attitude_quaternion()
        imu_msg.orientation.x = wxyz.x
        imu_msg.orientation.y = wxyz.y
        imu_msg.orientation.w = wxyz.w
        imu_msg.orientation.z = wxyz.z

        # Get Angular Rate Data
        angrate = self.s.read_angular_rate_measurements()
        imu_msg.angular_velocity.x = angrate.x
        imu_msg.angular_velocity.y = angrate.y
        imu_msg.angular_velocity.z = angrate.z

        # Get acceleration Data
        accel = self.s.read_acceleration_measurements()
        imu_msg.linear_acceleration.x = accel.x
        imu_msg.linear_acceleration.y = accel.y
        imu_msg.linear_acceleration.z = accel.z

        # Get timestamp of the imu data moment
        # This could be helpful for path planning
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish all this data as an imu message type
        self.publisher_.publish(imu_msg)
        self.get_logger().info('Publishing imu data')

    def __del__(self):
        """Method for properly shutting down the imu on program stop"""
        self.get_logger().info("Shutting down imu")

        self.s.disconnect()

def main(args=None):
    rclpy.init(args=args)

    imu = IMU()
    rclpy.spin(imu)

    imu.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        