import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUInterpreter(Node):
    def __init__(self):
        super().__init__('imu_interpreter')

        # Create IMU subscriber
        self.subscription = self.create_subscription(Imu, 'imu', self.interpret_imu_information, 10)

        # Create IMU interpreter publisher
        self.publisher_ = self.create_publisher(Imu, 'interpreted_imu', 10)

        self.Rx_est = 0.0
        self.Ry_est = 0.0
        self.Rz_est = 0.0

        # init time
        self.previous_time = self.get_clock().now()

        # weight for w gyro
        self.w_gyro = 10.0 #probably needs to be tuned

        # init with real imu data not just 0,0,0,0
        self.initialized = False

    def interpret_imu_information(self, msg: Imu):
        # get time distance
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds * 1e-9
        if dt <= 0.0:
            # skip if time difference isnt positive
            return
        self.previous_time = current_time

        # accelerometer, m/s^2
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        # gyro, rad/s
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # normalize
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if accel_magnitude == 0:
            self.get_logger().warning('Accelerometer magnitude is zero. Skipping this iteration.')
            return

        # normalize by gravity
        Rx_acc = accel_x / accel_magnitude
        Ry_acc = accel_y / accel_magnitude
        Rz_acc = accel_z / accel_magnitude

        # init with base values
        if not self.initialized:
            self.Rx_est = Rx_acc
            self.Ry_est = Ry_acc
            self.Rz_est = Rz_acc
            self.initialized = True
            return

        # previous estimate rest
        Rx_est_prev = self.Rx_est
        Ry_est_prev = self.Ry_est
        Rz_est_prev = self.Rz_est

        # calculate
        Axz_prev = math.atan2(Rx_est_prev, Rz_est_prev)
        Ayz_prev = math.atan2(Ry_est_prev, Rz_est_prev)

        # update angles based off gyro
        # gyro x corresponds to rotation around x axis affecting ayz (y component)
        # gyro y corresponds to rotation around y axis affecting axz (x component)
        Axz = Axz_prev + gyro_y * dt  # Rotation around y axis affects xz plane
        Ayz = Ayz_prev + gyro_x * dt  # rotation around x axis affects yz plane

        # reconstruct gyro
        sin_Axz = math.sin(Axz)
        cos_Axz = math.cos(Axz)
        tan_Ayz = math.tan(Ayz)

        denominator_x = math.sqrt(1 + (cos_Axz**2) * (tan_Ayz**2))
        Rx_gyro = sin_Axz / denominator_x

        sin_Ayz = math.sin(Ayz)
        cos_Ayz = math.cos(Ayz)
        tan_Axz = math.tan(Axz)

        denominator_y = math.sqrt(1 + (cos_Ayz**2) * (tan_Axz**2))
        Ry_gyro = sin_Ayz / denominator_y

        # get sign of rz gyro
        Rz_sign = 1.0 if Rz_est_prev >= 0 else -1.0
        Rz_gyro_sq = 1.0 - Rx_gyro**2 - Ry_gyro**2
        if Rz_gyro_sq < 0:
            # make rz not negative
            Rz_gyro_sq = max(Rz_gyro_sq, 0.0)
        Rz_gyro = Rz_sign * math.sqrt(Rz_gyro_sq)

        # normalize
        Rgyro_magnitude = math.sqrt(Rx_gyro**2 + Ry_gyro**2 + Rz_gyro**2)
        Rx_gyro /= Rgyro_magnitude
        Ry_gyro /= Rgyro_magnitude
        Rz_gyro /= Rgyro_magnitude

        # combine accelerometer and gyro, weight it
        w_gyro = self.w_gyro
        Rx_est = (Rx_acc + Rx_gyro * w_gyro) / (1.0 + w_gyro)
        Ry_est = (Ry_acc + Ry_gyro * w_gyro) / (1.0 + w_gyro)
        Rz_est = (Rz_acc + Rz_gyro * w_gyro) / (1.0 + w_gyro)

        # normalize
        Rest_magnitude = math.sqrt(Rx_est**2 + Ry_est**2 + Rz_est**2)
        Rx_est /= Rest_magnitude
        Ry_est /= Rest_magnitude
        Rz_est /= Rest_magnitude

        # update previous
        self.Rx_est = Rx_est
        self.Ry_est = Ry_est
        self.Rz_est = Rz_est

        # convert estimated vector to euler
        roll = math.atan2(Ry_est, Rz_est)
        pitch = math.atan2(-Rx_est, math.sqrt(Ry_est**2 + Rz_est**2))
        yaw = 0.0  # Cannot determine yaw without magnetometer data

        # euler to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy

        # Publishing stuff
        interpreted_imu_msg = Imu()
        interpreted_imu_msg.header.stamp = current_time.to_msg()
        interpreted_imu_msg.orientation.x = q_x
        interpreted_imu_msg.orientation.y = q_y
        interpreted_imu_msg.orientation.z = q_z
        interpreted_imu_msg.orientation.w = q_w
        interpreted_imu_msg.angular_velocity = msg.angular_velocity
        interpreted_imu_msg.linear_acceleration = msg.linear_acceleration

        # Publish
        self.publisher_.publish(interpreted_imu_msg)

    def main(args=None):
        rclpy.init(args=args)
        node = IMUInterpreter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
