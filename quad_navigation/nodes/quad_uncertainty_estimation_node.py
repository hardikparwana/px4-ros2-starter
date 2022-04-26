import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAngularVelocity

class Estimator(Node):
    
    def __init__(self):
        super().__init__('minimal_publisher')
        
        
        # Publishers
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # Subscribers
        self.sub_position = self.create_subscription(VehicleLocalPosition, '/fmu/vehicle_local_position/out', self.position_callback, 10)
        self.sub_attitude = self.create_subscription(VehicleAttitude, '/fmu/vehicle_attitude/out', self.attitude_callback, 10)
        self.sub_attitude = self.create_subscription(VehicleAngularVelocity, '/fmu/vehicle_angular_velocity/out', self.attitude_callback, 10)
        
        # Run Estimation at fixed frequency
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
def main(args=None):
    rclpy.init(args=args)

    estimator = Estimator()

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()