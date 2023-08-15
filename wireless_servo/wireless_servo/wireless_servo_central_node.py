from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor
from .wireless_servo_base_node import WirelessServoBaseNode
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class WirelessServoCentralNode(WirelessServoBaseNode):

    def __init__(self):

        super().__init__(node_name = 'wireless_servo_central')

        self.declare_parameter('arduino_local_name', 'wireless_servo_central')

        self._arduino_local_name = self.get_parameter('arduino_local_name').get_parameter_value().string_value
        self._arduino_peripherals_connected = False
       
        # publishers
        self._arduino_peripherals_connected_pub = self.create_publisher(Bool, self.get_name() + '/arduino_peripherals_connected', 1)
        # subscriptions
        self._serial_connected_sub = self.create_subscription(Bool, '/serial_manager/connected', self.serial_connected_sub_callback, 1)
        
    def pub_timer_callback(self):

        super().pub_timer_callback()

        msg = Bool()
        msg.data = self._arduino_peripherals_connected

        self._arduino_peripherals_connected_pub.publish(msg)

    def clear_arduino_srv_callback(self, request: Trigger.Request, response: Trigger.Response):

        return self.trigger_arduino_srv_callback(request, response, 'clear')
    
    def reset_arduino_srv_callback(self, request: Trigger.Request, response: Trigger.Response):

        return self.trigger_arduino_srv_callback(request, response, 'reset')
    
    def serial_connected_sub_callback(self, msg: Bool):
        
        # central is connected, when serial is connected
        self._arduino_connected = msg.data

        # when disconnected ensure that arduino_peripherals_connected is false
        if not self._arduino_connected:
            self._arduino_peripherals_connected = False

    def parse(self, msg: str) -> dict:
        
        content =  super().parse(msg)

        if content:
            try:
                assert all(key in content for key in ('connected',)) 

                # only when central/serial is connected parse 'connected'
                if self._arduino_connected:
                    self._arduino_peripherals_connected = content['connected'] == '1'
            except:
                self.get_logger().warn('Unable to parse serial data')
                content = {}

        return content

def main(args = None):

    rclpy.init(args = args)
    try:
        node = WirelessServoCentralNode()
        executor = MultiThreadedExecutor()
        try:
            rclpy.spin(node, executor = executor)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()