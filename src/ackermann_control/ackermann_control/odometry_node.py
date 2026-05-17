import rclpy
from rclpy.node import Node
import serial
import struct
import math
import time

from nav_msgs.msg import Odometry

# The EKF will handle the TF tree now!

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # ✅ CHANGED: True robot geometry based on your physical tests
        self.wheel_radius = 0.0325       # 6.5 cm diameter
        self.wheelbase = 0.213
        self.ticks_per_rev = 330         # 1X decoding reality

        self.meters_per_tick = (2*math.pi*self.wheel_radius)/self.ticks_per_rev

        # servo calibration
        self.servo_center = 1300
        self.servo_left = 650
        self.servo_right = 1790
        
        # ✅ CHANGED: Removed max_steer variable since it is asymmetric now

        # pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()

        self.ser = serial.Serial('/dev/ttyACM0',115200,timeout=0.01)
        time.sleep(2)

        self.odom_pub = self.create_publisher(Odometry,'/odom_raw',20)

        self.timer = self.create_timer(0.02,self.update)

    def update(self):

        while self.ser.in_waiting >= 14:

            header = self.ser.read(2)
            if header != b'\x55\xAA':
                continue

            length = self.ser.read(1)[0]
            if length != 10:
                self.ser.read(length+1)
                continue

            payload = self.ser.read(10)
            checksum = self.ser.read(1)[0]

            if self.calc_checksum(payload) != checksum:
                continue

            dL,dR,servo = struct.unpack('<iiH',payload)

            self.integrate(dL,dR,servo)

    def integrate(self,dL_ticks,dR_ticks,servo_pwm):

        current_time = self.get_clock().now()
        dt = (current_time-self.last_time).nanoseconds*1e-9
        self.last_time = current_time

        if dt<=0:
            return

        dL = dL_ticks*self.meters_per_tick
        dR = dR_ticks*self.meters_per_tick

        d_center = (dL+dR)/2
        v = d_center / dt

        # The Deadband Filter
        if abs(v) < 0.05:
            d_center = 0.0
            v = 0.0

        # ✅ CHANGED: Corrected asymmetric steering logic for the reversed servo rack
        # Center = 1300, Left = 650, Right = 1790
        if servo_pwm < self.servo_center:
            # Turning LEFT (PWM drops from 1300 towards 650) -> Positive angle
            ratio = (self.servo_center - servo_pwm) / (self.servo_center - self.servo_left)
            steer = ratio * math.radians(25)
        else:
            # Turning RIGHT (PWM rises from 1300 towards 1790) -> Negative angle
            ratio = (servo_pwm - self.servo_center) / (self.servo_right - self.servo_center)
            steer = -ratio * math.radians(25) # ⚠️ Notice the negative sign to signal a right turn!

        yaw_rate = 0.0
        # Only calculate turning if we are actually moving forward and steering
        if abs(steer) > 1e-4 and abs(v) > 0.0:
            yaw_rate = v / self.wheelbase * math.tan(steer)

        d_theta = yaw_rate*dt

        self.x += d_center*math.cos(self.yaw)
        self.y += d_center*math.sin(self.yaw)
        self.yaw += d_theta

        self.publish(v, yaw_rate, current_time)

    def publish(self,v,w,stamp):

        qz = math.sin(self.yaw/2)
        qw = math.cos(self.yaw/2)

        odom = Odometry()

        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

    def calc_checksum(self,data):

        c = 0
        for b in data:
            c ^= b
        return c

def main():

    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
