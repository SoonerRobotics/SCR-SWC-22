import rclpy
import math
from rclpy.node import Node
from swc_msgs.msg import Control
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

class PID:
    def __init__(self, dt, kP, kI, kD):
        self.dt = dt
        self.kP = kP
        self.kI = kI
        self.kD = kD

        self.previous_error = 0
        
        self.proportional = 0
        self.integral = 0
        self.derivate = 0

    def process(self, current, target):
        error = target - current

        self.proportional  = error
        self.integral = self.integral + error * self.dt
        self.derivative = (error - self.previous_error) / self.dt
        
        self.previous_error = error

        return self.kP * self.proportional + self.kI * self.integral + self.kD * self.derivate

# https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
class DemoNode(Node):

    def __init__(self):
        super().__init__('demo_node')
        
        self.control_publisher = self.create_publisher(Control, '/sim/control', 10)

        self.velocity_subscriber = self.create_subscription(Float32, '/sim/velocity', self.velocity_callback, 1)

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.headingPID = PID(0.05, 0.1, 0.001, 0)
        self.speedPID = PID(0.05, 1000, 1, 0)

        self.curSpeed = 0
        self.curHeading = 0

    def timer_callback(self):
        msg = Control()

        msg.rpm = self.speedPID.process(self.curSpeed, 1)
        msg.turn_angle = self.headingPID.process(self.curHeading, 2)

        self.control_publisher.publish(msg)

    def velocity_callback(self, msg: Float32):
        self.curSpeed = msg.data

    def imu_callback(self, msg: Imu):
        roll, pitch, yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.curHeading = pitch

def main(args=None):
    rclpy.init(args=args)

    demo_node = DemoNode()

    rclpy.spin(demo_node)

    # Destroy the node explicitly
    demo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()