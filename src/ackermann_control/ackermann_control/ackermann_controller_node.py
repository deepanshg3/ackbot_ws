import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import math
import time


class AckermannController(Node):

    def __init__(self):
        super().__init__('ackermann_controller')

        # ================= Robot Parameters =================
        self.L = 0.213                     # wheelbase (m)
        self.max_speed = 0.20              # m/s (Matched to Arduino limit)
        self.max_steer = math.radians(25)  # radians

        # Servo calibration (microseconds)
        self.servo_center = 1300
        self.servo_left = 650
        self.servo_right = 1790

        # ================= Serial Setup =================
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            time.sleep(2.0)  # wait for Arduino reset
            self.get_logger().info("Serial connected.")
        except serial.SerialException:
            self.get_logger().error("Could not open serial port.")
            raise

        # ================= Safety =================
        self.last_cmd_time = time.time()
        self.timeout = 0.5  # seconds

        self.v = 0.0
        self.w = 0.0

        # ================= ROS =================
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(1.0 / 30.0, self.update)

        self.get_logger().info("Ackermann Controller Started 🚗")

    # =========================================================
    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd_time = time.time()

    # =========================================================
    def update(self):

        # ===== Safety Timeout =====
        if time.time() - self.last_cmd_time > self.timeout:
            self.v = 0.0
            self.w = 0.0

        # ===== Clamp Linear Speed =====
        v = max(min(self.v, self.max_speed), -self.max_speed)

        # ===== Compute Steering Angle (Ackermann) =====
        if abs(v) > 1e-3:
            # ✅ THE FIX: Removed abs(v). 
            # True Ackermann kinematics require the real sign of v to handle reversing properly!
            steer = math.atan((self.L * self.w) / v)
        else:
            # When stationary, keep wheels straight
            steer = 0.0

        steer = max(min(steer, self.max_steer), -self.max_steer)

        # ===== Convert Speed to signed int8 (-127 to 127) =====
        motor_pwm = int((v / self.max_speed) * 127)
        motor_pwm = max(min(motor_pwm, 127), -127)

        # ===== Convert Steering to Servo PWM =====
        steer_ratio = steer / self.max_steer

        if steer_ratio >= 0.0:
            servo_pwm = int(
                self.servo_center +
                steer_ratio * (self.servo_left - self.servo_center)
            )
        else:
            servo_pwm = int(
                self.servo_center +
                steer_ratio * (self.servo_center - self.servo_right)
            )

        servo_pwm = max(650, min(servo_pwm, 1790))

        # ===== Build Serial Packet =====
        # Format:
        # [0xAA][0x55][LEN=3][speed(int8)][servo(uint16)][checksum]

        header = b'\xAA\x55'
        length = b'\x03'

        speed_byte = struct.pack('b', motor_pwm)
        servo_bytes = struct.pack('<H', servo_pwm)

        low_byte = servo_pwm & 0xFF
        high_byte = (servo_pwm >> 8) & 0xFF

        checksum = (motor_pwm & 0xFF) ^ low_byte ^ high_byte
        checksum_byte = struct.pack('B', checksum)

        packet = header + length + speed_byte + servo_bytes + checksum_byte

        try:
            self.ser.write(packet)
        except serial.SerialException:
            self.get_logger().error("Serial write failed.")

    # =========================================================
    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
