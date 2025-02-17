import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyFilterNode(Node):
    def __init__(self):
        super().__init__('joy_filter_node')
        self.cmd_subscriber = self.create_subscription(Twist, '/cmd_vel_joy', self.cmd_callback, 10)
        self.filter_publisher = self.create_publisher(Twist, '/filtered_cmd_vel_joy', 10)
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.axes_enabled = True
        self.global_cmd_vel = Twist()

    def cmd_callback(self, msg):
        self.global_cmd_vel = msg

    def joy_callback(self, msg):
        # Check if the X button (assuming index 0) is pressed
        if msg.buttons[0] == 1:
            if self.axes_enabled:
                self.axes_enabled = False
                self.get_logger().info("X button pressed. Disabling joystick axes.")

        # Check if the Circle button (assuming index 1) is pressed
        if msg.buttons[1] == 1:
            if not self.axes_enabled:
                self.axes_enabled = True
                self.get_logger().info("Circle button pressed. Enabling joystick axes.")
        
        # Check if the Square button (assuming index 2) is pressed
        if msg.buttons[2] == 1:
            if self.axes_enabled:
                self.axes_enabled = False
                self.get_logger().info("Square button pressed. Disabling joystick axes.")

        # Check if the Up button (assuming index 11) is pressed
        if msg.buttons[11] == 1:
            if self.axes_enabled:
                self.axes_enabled = False
                self.get_logger().info("Up button pressed. Disabling joystick axes.")

        # Check if the Down button (assuming index 12) is pressed
        if msg.buttons[12] == 1:
            if self.axes_enabled:
                self.axes_enabled = False
                self.get_logger().info("Down button pressed. Disabling joystick axes.")
        # Check if the Left button (assuming index 13) is pressed
        if msg.buttons[4] == 1:
            if self.axes_enabled:
                self.axes_enabled = False
                self.get_logger().info("Left button pressed. Disabling joystick axes.")

        # Check if the Right button (assuming index 14) is pressed
        if msg.buttons[5] == 1:
            if self.axes_enabled:
                self.axes_enabled = False
                self.get_logger().info("Right button pressed. Disabling joystick axes.")

        # Check if the Triangle button (assuming index 3) is pressed
        if msg.buttons[3] == 1:
            if self.axes_enabled:
                self.axes_enabled = False
                self.get_logger().info("Triangle button pressed. Disabling joystick axes.")

        # Publish the joystick commands only if axes are enabled
        if self.axes_enabled:
            self.filter_publisher.publish(self.global_cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = JoyFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
