import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler

class IMU_Interpreter(Node):
    def __init__(self):
        super().__init__('imu_interpreter')

        # Create IMU subscriber
        self.subscription = self.create_subscription(Imu, 'imu', self.interpret_imu_information, 10)

        # Create IMU interpreter publisher
        self.publisher_= self.create_publisher(Imu, 'interpreted_imu', 10)
        
        # Init previous values
        self.previous_gyro_angle_x = 0.0
        self.previous_gyro_angle_y = 0.0
        self.previous_gyro_angle_z = 0.0
        self.previous_time = self.get_clock().now()

        self.initialized = False # So I can init with the actual gyro values

    # What runs when we get info from the IMU publisher
    def interpret_imu_information(self, msg: Imu):
        # Initialize with actual gyro values
        if (not self.initialized):
            self.previous_gyro_angle_x = msg.orientation.x
            self.previous_gyro_angle_y = msg.orientation.y
            self.previous_gyro_angle_z = msg.orientation.z
            self.previous_time = self.get_clock().now()
            self.initialized = True

            # Treat first cycle as starting position
            return

        # Raw IMU data
        # Accelerometer
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        # Normalize acceleration
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if accel_magnitude > 0:
            accel_x /= accel_magnitude
            accel_y /= accel_magnitude
            accel_z /= accel_magnitude

        # Angular rate (gyro data)
        angrate_x = msg.angular_velocity.x
        angrate_y = msg.angular_velocity.y
        angrate_z = msg.angular_velocity.z

        # Inclination angles (from accelerometer)
        accel_angle_x = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        accel_angle_y = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # Time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds * 1e-9
        self.previous_time = current_time

        # Estimate angle by integrating gyro
        gyro_angle_x = self.previous_gyro_angle_x + angrate_x * dt
        gyro_angle_y = self.previous_gyro_angle_y + angrate_y * dt
        gyro_angle_z = self.previous_gyro_angle_z + angrate_z * dt

        # Apply complementary filter
        alpha = 0.98  # Weight factor (adjust if needed)
        filtered_angle_x = alpha * gyro_angle_x + (1 - alpha) * accel_angle_x
        filtered_angle_y = alpha * gyro_angle_y + (1 - alpha) * accel_angle_y

        # Quaternion from filtered angles
        q = quaternion_from_euler(filtered_angle_x, filtered_angle_y, gyro_angle_z)
        
        interpreted_imu_msg = Imu()
        interpreted_imu_msg.orientation.x = q[0]
        interpreted_imu_msg.orientation.y = q[1]
        interpreted_imu_msg.orientation.z = q[2]
        interpreted_imu_msg.orientation.w = q[3]
        interpreted_imu_msg.angular_velocity = msg.angular_velocity
        interpreted_imu_msg.linear_acceleration = msg.linear_acceleration
        interpreted_imu_msg.header.stamp = current_time.to_msg()

        # Publish interpreted IMU message
        self.publisher_.publish(interpreted_imu_msg)

        # Update previous gyro angles for next iteration
        self.previous_gyro_angle_x = gyro_angle_x
        self.previous_gyro_angle_y = gyro_angle_y
        self.previous_gyro_angle_z = gyro_angle_z

def main(args=None):
    rclpy.init(args=args)
    node = IMU_Interpreter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
