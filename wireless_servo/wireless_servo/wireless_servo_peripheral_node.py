from __future__ import annotations

import rclpy

from .wireless_servo_base_node import WirelessServoBaseNode
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Trigger
from custom_srvs.srv import SerialSend
from custom_actions.action import Empty
from threading import Lock

class WirelessServoPeripheralNode(WirelessServoBaseNode):

    def __init__(self):
        
        super().__init__(node_name = 'wireless_servo_peripheral')

        self.declare_parameter('arduino_local_name', 'wireless_servo0')

        self._arduino_local_name = self.get_parameter('arduino_local_name').get_parameter_value().string_value
        self._arduino_connected = False
        self._servo_running = False
        self._servo_closed = 2 # 0: open, 1: close, 2: unknown
        self._servo_timeout = 4 # in seconds
        self._servo_action_id = int(-1)
        self._limitswitch0 = False
        self._limitswitch1 = False
        self._invert = True # invert polarity of limit switch
        self._lock = Lock()

        # subscriptions
        self._arduino_peripherals_connected_sub = self.create_subscription(Bool, '/wireless_servo_central/arduino_peripherals_connected', self.arduino_peripherals_connected_sub_callback , 1)
        # publishers
        self._servo_running_pub = self.create_publisher(Bool, self.get_name() + '/servo_running', 1)
        self._servo_closed_pub = self.create_publisher(Int8, self.get_name() + '/servo_closed', 1)
        self._limitswitch0_pub = self.create_publisher(Bool, self.get_name() + '/limitswitch0', 1)
        self._limitswitch1_pub = self.create_publisher(Bool, self.get_name() + '/limitswitch1', 1)
        # actions
        self._servo_open_action = ActionServer(self, Empty, self.get_name() + '/servo_open', 
                                               execute_callback=self.execute_servo_open_action_callback, 
                                               goal_callback=self.goal_servo_action_callback,
                                               cancel_callback=self.cancel_servo_action_callback)
        self._servo_close_action = ActionServer(self, Empty, self.get_name() + '/servo_close', 
                                                execute_callback=self.execute_servo_close_action_callback, 
                                                goal_callback=self.goal_servo_action_callback,
                                                cancel_callback=self.cancel_servo_action_callback)
        
    def pub_timer_callback(self):

        super().pub_timer_callback()

        msg = Bool()
        msg.data = bool(self._servo_running)

        self._servo_running_pub.publish(msg)

        msg = Int8()
        msg.data = int(self._servo_closed)

        self._servo_closed_pub.publish(msg)

        msg = Bool()
        msg.data = bool(self._limitswitch0)

        self._limitswitch0_pub.publish(msg)

        msg = Bool()
        msg.data = bool(self._limitswitch1)

        self._limitswitch1_pub.publish(msg)

    def clear_arduino_srv_callback(self, request: Trigger.Request, response: Trigger.Response):

        return self.trigger_arduino_srv_callback(request, response, 'send/' + self._arduino_local_name + '/clear')
    
    def reset_arduino_srv_callback(self, request: Trigger.Request, response: Trigger.Response):

        return self.trigger_arduino_srv_callback(request, response, 'send/' + self._arduino_local_name + '/reset')
    
    def execute_servo_open_action_callback(self, goal_handle: ServerGoalHandle):   
        
        return self.execute_servo_action(goal_handle, 0)
     
    def execute_servo_close_action_callback(self, goal_handle: ServerGoalHandle):      
        
        return self.execute_servo_action(goal_handle, 1)

    def execute_servo_action(self, goal_handle: ServerGoalHandle, close: bool):
        
        servo_action_id = self.get_servo_action_id()

        with self._lock:

            state = 0

            while True:
                self._rate.sleep()

                if state == 0:
                    start_time = self.time_in_seconds()
                    if self._servo_closed == close:
                        self.get_logger().info('Open/close servo [%s]: Completed' %servo_action_id)
                        goal_handle.succeed()
                        state = 5
                    else:
                        send_future: rclpy.Future = None  
                        state = 1

                # wait for service to become available
                elif state == 1:
                    if self._serial_send_cli.service_is_ready():
                        start_time = self.time_in_seconds() # reset timeout
                        state = 2
                    else:
                        state = 1

                # send request
                elif state == 2:
                    req = SerialSend.Request()
                    if close:
                        req.msg = 'send/' + self._arduino_local_name + '/start/close'
                    else:
                        req.msg = 'send/' + self._arduino_local_name + '/start/open'
                    send_future = self._serial_send_cli.call_async(req)
                    state = 3

                # wait for send future
                elif state == 3:
                    if send_future.done():
                        if send_future.result().success == 0:
                            self.get_logger().error('Open/close servo [%s]: Aborted, "serial_send" failed' %servo_action_id)
                            goal_handle.abort()
                            state = 5
                        else:
                            state = 4
                
                # wait for servo to close
                elif state == 4:
                    if self._servo_closed == close:
                        self.get_logger().info('Open/close servo [%s]: Completed' %servo_action_id)
                        goal_handle.succeed()
                        state = 5
                    else:
                        state = 4

                # exit
                elif state == 5:
                    break
            
                # exit conditions
                if (self.time_in_seconds() - start_time) > self._servo_timeout:
                    self.get_logger().error('Open/close servo [%s]: Timed out' %servo_action_id)
                    goal_handle.abort() 
                    state = 5 
                elif self._arduino_connected == False:
                    self.get_logger().error('Open/close servo [%s]: Aborted, Arduino disconnected' %servo_action_id)
                    goal_handle.abort()
                    state = 5
                elif servo_action_id != self._servo_action_id:
                    self.get_logger().info('Open/close servo [%s]: Aborted, overwritten by new action' %servo_action_id)
                    goal_handle.abort() 
                    state = 5
                elif goal_handle.is_cancel_requested:
                    self.get_logger().error('Open/close servo [%s]: Canceled' %servo_action_id)
                    goal_handle.canceled()
                    state = 5

            return Empty.Result()

    def cancel_servo_action_callback(self, _):

        self.trigger_arduino_srv_callback(SerialSend.Request(), SerialSend.Response(), 'send/' + self._arduino_local_name + '/stop')

        return CancelResponse.ACCEPT
    
    def goal_servo_action_callback(self, _):

        return GoalResponse.ACCEPT

    def arduino_peripherals_connected_sub_callback(self, msg: Bool):

        self._arduino_connected = msg.data

    def parse(self, msg: str) -> dict:
        
        content = super().parse(msg)

        if content:
            try:
                assert all(key in content for key in ('running', 'closed', 'limitswitch0', 'limitswitch1'))

                self._servo_running = content['running'] == '1'
                self._servo_closed = int(content['closed'])
                self._limitswitch0 = (content['limitswitch0'] == '1') ^ self._invert
                self._limitswitch1 = (content['limitswitch1'] == '1') ^ self._invert
            except:
                self.get_logger().warn('Unable to parse serial data')
    
    def get_servo_action_id(self) -> int:
        
        self._servo_action_id = self._servo_action_id + 1
        
        return self._servo_action_id

def main(args = None):

    rclpy.init(args = args)
    try:
        node = WirelessServoPeripheralNode()
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
