from __future__ import annotations

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool, Int8
from std_srvs.srv import Trigger
from custom_srvs.srv import SerialSend

class WirelessServoBaseNode(Node):

    def __init__(self, **kwargs):
        
        super().__init__(**kwargs)

        self._arduino_local_name = ''
        self._arduino_error_code = 0
        self._arduino_connected = False

        # groups
        self._client_group = ReentrantCallbackGroup()
        # subscriptions
        self._serial_message_sub = self.create_subscription(String, 'serial_manager/message', self.serial_message_sub_callback, 20)
        # publishers
        self._arduino_local_name_pub = self.create_publisher(String, self.get_name() + '/arduino_local_name', 1)
        self._arduino_connected_pub = self.create_publisher(Bool, self.get_name() + '/arduino_connected', 1)
        self._arduino_error_code_pub = self.create_publisher(Int8, self.get_name() + '/arduino_error_code', 1)
        # services
        self._clear_arduino_srv = self.create_service(Trigger, self.get_name() + '/clear_arduino', self.clear_arduino_srv_callback)
        self._reset_arduino_srv = self.create_service(Trigger, self.get_name() + '/reset_arduino', self.reset_arduino_srv_callback)
        # clients
        self._serial_send_cli = self.create_client(SerialSend, '/serial_manager/send', callback_group=self._client_group)
        # reates
        self._rate = self.create_rate(5, self.get_clock())
        # timers
        self._pub_timer = self.create_timer(0.25, self.pub_timer_callback)

    def pub_timer_callback(self):

        msg = Bool()
        msg.data = bool(self._arduino_connected)

        self._arduino_connected_pub.publish(msg)

        msg = Int8()
        msg.data = int(self._arduino_error_code)

        self._arduino_error_code_pub.publish(msg)

        msg = String()
        msg.data = self._arduino_local_name

        self._arduino_local_name_pub.publish(msg)

    def serial_message_sub_callback(self, msg: String) -> dict:

        if self._arduino_local_name in msg.data:

            self.parse(msg.data)

    def clear_arduino_srv_callback(self, request: Trigger.Request, response: Trigger.Response):

        pass
    
    def reset_arduino_srv_callback(self, request: Trigger.Request, response: Trigger.Response):

        pass
  
    def trigger_arduino_srv_callback(self, request: Trigger.Request, response: Trigger.Response, msg: str):

        timeout = 3

        state = 0

        while True:
            self._rate.sleep()

            if state == 0:
                start_time = self.time_in_seconds()
                response = Trigger.Response()
                state = 1

            # wait for server
            elif state == 1:
                if self._serial_send_cli.service_is_ready():
                    start_time = self.time_in_seconds()
                    state = 2
                else:
                    state = 1
            
            # send request
            elif state == 2:
                req = SerialSend.Request()
                req.msg = msg
                future = self._serial_send_cli.call_async(req)
                state = 3

            # wait for future
            elif state == 3:
                if future.done():
                    response.success = future.result().success
                    state = 4
                else:
                    state = 3

            # exit
            elif state == 4:
                break

            # exit conditions
            if (self.time_in_seconds() - start_time) > timeout:
                self.get_logger().error('Arduino service: Timeout')
                response.success = False
                state = 4
            elif self._arduino_connected == False:
                self.get_logger().error('Arduino service: Arduino not disconnected')
                response.success = False
                state = 4

            self._rate.sleep()

        return response  
        
    def parse(self, msg: str) -> dict:

        try:
            # remove start/end markers
            msg = msg[1:-1]
            
            # split message into lines
            msg = msg.split('\n')

            # split each line into variable name + value
            for i in range(len(msg)):
                msg[i] = msg[i].split(':')
            
            # turn into dictionary
            content = {item[0]: item[1] for item in msg}

            # assert correctness of the contents
            assert all(key in content for key in ('error',))

            # extract data
            arduino_error_code = int(content['error'])

        except:
            self.get_logger().warn('Unable to parse serial data')
            content = {}
        else:
            # check for error codes in the Arduino
            if arduino_error_code > 0 and arduino_error_code != self._arduino_error_code:
                    self.get_logger().error('Arduino in error state (error code ' + str(arduino_error_code) + ')')
            self._arduino_error_code = arduino_error_code

        return content
    
    def time_in_seconds(self) -> float:

        return self.get_clock().now().nanoseconds/(10**9)




