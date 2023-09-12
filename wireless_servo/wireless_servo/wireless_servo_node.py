from __future__ import annotations

import rclpy
import bleak
import asyncio
import threading
import queue
from rclpy_wrapper.node import Node2
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger
from custom_actions.action import Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from typing import Any

#https://github.com/Ladvien/arduino_ble_sense/blob/master/app.py


class SendFeedback():

    UNKNOWN = 0
    SUCCEEDED = 1
    FAILED = 2

    def __init__(self):

        self._status = self.UNKNOWN
        self._message = ''

    def succeeded(self, message: str = ''):

        self._status = self.SUCCEEDED
        self._message = message

    def failed(self, message: str = ''):

        self._status = self.FAILED
        self._message = message

    def status(self) -> tuple[int, str]:

        return self._status, self._message


class SendQueue(queue.Queue):

    def __init__(self):

        super().__init__(1)

        self.lock = threading.Lock()

    def enqueue(self, item) -> tuple[bool, SendFeedback]:
        
        with self.lock:
            if self.empty():
                feedback = SendFeedback()
                self.put((item, feedback))
                return False, feedback
            else:
                return True, None
            
    def dequeue(self) -> tuple[Any, SendFeedback, bool]:

        with self.lock:
            if self.qsize() > 0:
                item, message = self.get()
                return item, message, False
            else:
                return None, None, True


class WirelessServo():

    def __init__(self):

        self.node: WirelessServoNode = None

        # servo parameters
        self.connected = False
        self.mode = None
        self.limitswitch0 = False
        self.limitswitch1 = False
        self.polarity = True
        self.running = False
        self.error = False
        self.closed = int(0)
        self.error = int(0)

        # bluetooth parameters
        self.name = 'wireless_servo0'
        self.client: bleak.BleakClient = None
        self.device = None
        self.service = None
        self.rxcharacteristic = None
        self.txcharacteristic = None

        # threading parameters
        self.thread: threading.Thread = None
        self.send_queue = SendQueue()
        self.stop_event = threading.Event()

    def disconnect_callback(self, client: bleak.BleakClient):

        self.connected = False
        self.get_logger().info(f"Disconnected from {self.name}")

    def receive_callback(self, _, data: bytearray):

        self.mode = int(data[0])
        self.limitswitch0 = bool(data[1]) ^ self.polarity
        self.limitswitch1 = bool(data[2]) ^ self.polarity
        self.running = bool(data[4])
        self.error = int(data[5])
        self.closed = int(data[6])

    def get_logger(self):

        return self.node.get_logger()

    async def send(self):

        message, feedback, timedout = self.send_queue.dequeue()

        if timedout:
            return
        else:
            try:
                if not self.connected:
                    raise Exception('Not connected')
                else:
                    await self.client.write_gatt_char(self.txcharacteristic, message.encode('utf-8'), response=True)
                    feedback.succeeded(f'Succeded to send "{message}"')
            except Exception as exc:
                feedback.failed(f'Failed to send "{message}": {exc}')

    async def discover(self):

        try:
            self.device = await bleak.BleakScanner.find_device_by_name(self.name, timeout=0.5)

            if self.device is not None:
                self.client = bleak.BleakClient(self.device.address, disconnected_callback = self.disconnect_callback)
                self.get_logger().info(f"Discovered {self.name}")

            else:
                self.get_logger().warn(f"Failed to discover {self.name}")

        except Exception as exc:
            self.get_logger().error(f"Failed to discover {self.name}: {exc}")

    async def connect(self):

        try:
            await self.client.connect()
            self.connected = bool(self.client.is_connected)         
            if self.connected:
                for service in self.client.services:
                    if service.uuid == "7def8317-7300-4ee6-8849-46face74ca2a":
                        self.service = service
                for characteristic in self.service.characteristics:
                    if characteristic.uuid == "7def8317-7301-4ee6-8849-46face74ca2a":
                        self.rxcharacteristic = characteristic
                    elif characteristic.uuid == "7def8317-7302-4ee6-8849-46face74ca2a":
                        self.txcharacteristic = characteristic
                await self.client.start_notify(self.rxcharacteristic, self.receive_callback)
                self.get_logger().info(f"Connected to {self.name}")
            else:
                self.get_logger().warn(f"Failed to connect to {self.name}")
        except Exception as exc:
            self.get_logger().error(f"Failed to connect to {self.name}: {exc}")

    async def destroy(self):

        if self.client:
            try:
                await self.client.stop_notify(self.rxcharacteristic)
            except Exception as exc:
                self.get_logger().error(str(exc))
            finally:
                await self.client.disconnect()

    async def execute(self):

        try:
            state = 0
            while not self.stop_event.is_set():
                # discover servo
                if state == 0:
                    await self.discover()
                    if self.device is None:
                        state = 0
                    else: 
                        state = 1
                # connect servo
                elif state == 1:
                    await self.connect()
                    if not self.connected:
                        state = 1
                    else:
                        state = 2
                # send messages to servo:
                if state == 2:
                    await self.send()
                    if not self.connected:
                        state = 1
                    else:
                        state = 2
                await asyncio.sleep(0.25)
        finally:
            await self.destroy()

    def start_async_execution(self):

        self.thread = threading.Thread(target=asyncio.run, args = (self.execute(),))
        self.thread.start()

    def stop_async_execution(self):

        self.stop_event.set()
        self.thread.join(10)


class WirelessServoNode(Node2):

    def __init__(self, servo: WirelessServo):

        super().__init__('wireless_servo')

        self.declare_parameter('servo_name', 'wireless_servo0')

        # servo
        self.servo = servo
        self.servo.node = self
        self.servo.name = self.get_parameter('servo_name').get_parameter_value().string_value

        # publishers
        self.servo_name_pub = self.create_publisher(String, self.get_name() + '/name', 1)
        self.servo_connected_pub = self.create_publisher(Bool, self.get_name() + '/connected', 1)
        self.servo_running_pub = self.create_publisher(Bool, self.get_name() + '/running', 1)
        self.servo_closed_pub = self.create_publisher(Int8, self.get_name() + '/closed', 1)
        self.servo_limitswitch0_pub = self.create_publisher(Bool, self.get_name() + '/limitswitch0', 1)
        self.servo_limitswitch1_pub = self.create_publisher(Bool, self.get_name() + '/limitswitch1', 1)
        self.servo_error_pub = self.create_publisher(Int8, self.get_name() + '/error', 1)

        # services
        self.create_service(Trigger, self.get_name() + '/clear', self.clear_srv_callback)
        self.create_service(Trigger, self.get_name() + '/reset', self.reset_srv_callback)

        # timing
        self.create_timer(0.5, self.timer_callback)
        self.rate = self.create_rate(10)
        self.lock = threading.Lock()
        self.timeout = 3

        # actions
        self.create_action_server(Empty, self.get_name() + '/open', 
                                  execute_callback=self.execute_servo_open_action_callback, 
                                  goal_callback=self.goal_servo_action_callback,
                                  cancel_callback=self.cancel_servo_action_callback)
        self.create_action_server(Empty, self.get_name() + '/close', 
                                  execute_callback=self.execute_servo_close_action_callback, 
                                  goal_callback=self.goal_servo_action_callback,
                                  cancel_callback=self.cancel_servo_action_callback)
        self.servo_action_id = int(-1)

    def timer_callback(self):

        msg = String()
        msg.data = self.servo.name
        self.servo_name_pub.publish(msg)

        msg = Bool()
        msg.data = self.servo.connected
        self.servo_connected_pub.publish(msg)

        msg = Bool()
        msg.data = self.servo.running
        self.servo_running_pub.publish(msg)

        msg = Int8()
        msg.data = self.servo.closed
        self.servo_closed_pub.publish(msg)

        msg = Bool()
        msg.data = self.servo.limitswitch0
        self.servo_limitswitch0_pub.publish(msg)

        msg = Bool()
        msg.data = self.servo.limitswitch1
        self.servo_limitswitch1_pub.publish(msg)

        msg = Int8()
        msg.data = self.servo.error
        self.servo_error_pub.publish(msg)

    def clear_srv_callback(self, resquest: Trigger.Request, _) -> Trigger.Response:
        
        response = self.send('clear')

        return response

    def reset_srv_callback(self, resquest: Trigger.Request, _) -> Trigger.Response:

        response = self.send('reset')

        return response
    
    def execute_servo_open_action_callback(self, goal_handle: ServerGoalHandle):   
        
        return self.execute_servo_action(goal_handle, 0)
     
    def execute_servo_close_action_callback(self, goal_handle: ServerGoalHandle):      
        
        return self.execute_servo_action(goal_handle, 1)
    
    def cancel_servo_action_callback(self, _):

        self.send('open')

        return CancelResponse.ACCEPT
    
    def goal_servo_action_callback(self, _):

        return GoalResponse.ACCEPT

    def execute_servo_action(self, goal_handle: ServerGoalHandle, close: bool):
        
        servo_action_id = self.get_servo_action_id()

        with self.lock:

            state = 0

            while True:
                # initialize
                if state == 0:
                    self.get_logger().info(f'Open/close servo [{servo_action_id}]: Start')
                    if self.servo.closed == close:
                        self.get_logger().info(f'Open/close servo [{servo_action_id}]: Succeeded')
                        goal_handle.succeed()
                        break
                    else:
                        start_time = self.time_in_seconds()
                        state = 1
                # send message
                elif state == 1:
                    if close:
                        timedout, feedback = self.servo.send_queue.enqueue('start/close')
                    else:
                        timedout, feedback = self.servo.send_queue.enqueue('start/open')
                    if timedout:
                        state = 1
                    else: 
                        state = 2
                # wait for sending of the message
                elif state == 2:
                    status, message = feedback.status()
                    if status == SendFeedback.SUCCEEDED:
                        start_time = self.time_in_seconds()
                        state = 3
                    elif status == SendFeedback.FAILED:
                        self.get_logger().error(f'Open/close servo [{servo_action_id}]: Aborted: {message}')
                        goal_handle.abort()
                        break
                    else:
                        state = 2
                # wait for servo to close
                elif state == 3:
                    if self.servo.closed == close:
                        self.get_logger().info(f'Open/close servo [{servo_action_id}]: Succeeded')
                        goal_handle.succeed()
                        break
                print(state)
                if self.time_in_seconds() - start_time > self.timeout:
                    self.get_logger().error(f'Open/close servo [{servo_action_id}]: Aborted, timed out')
                    goal_handle.abort() 
                    break
                elif goal_handle.is_cancel_requested:
                    self.get_logger().error(f'Open/close servo [{servo_action_id}]: Canceled')
                    goal_handle.canceled() 
                    break
                elif servo_action_id != self.servo_action_id:
                    self.get_logger().info(f'Open/close servo [{servo_action_id}]: Aborted, overwritten by new action' )
                    goal_handle.abort() 
                    break
                elif self.servo.error > 0:
                    self.get_logger().error(f'Open/close servo [{servo_action_id}]: Aborted, servo in error state {self.servo.error}')
                    goal_handle.abort()
                    break
                self.rate.sleep()
            return Empty.Result()

    def send(self, message: str) -> Trigger.Response:

        state = 0

        while True:
            if state == 0:
                response = Trigger.Response()
                start_time = self.time_in_seconds()
                state = 1
            elif state == 1:
                timedout, feedback = self.servo.send_queue.enqueue(message)
                if timedout:
                    if self.time_in_seconds() - start_time >= self.timeout:
                        response.success = False
                        response.message = f'Timed out while sending message "{message}"'
                        self.get_logger().error(response.message)
                        break
                    else:
                        state = 1
                else:
                    state = 2
            elif state == 2:
                status, message = feedback.status()
                if status == SendFeedback.SUCCEEDED:
                    response.success = True
                    response.message = message
                    self.get_logger().info(response.message)
                    break
                elif status == SendFeedback.FAILED:
                    response.success = False
                    response.message = message
                    self.get_logger().error(response.message)
                    break
                elif self.time_in_seconds() - start_time >= self.timeout:
                    response.success = False
                    response.message = f'Timed out while sending message "{message}"'
                    self.get_logger().error(response.message)
                    break
                else:
                    state = 2
            self.rate.sleep()
        return response
    
    def get_servo_action_id(self) -> int:
        
        self.servo_action_id = self.servo_action_id + 1
        
        return self.servo_action_id
    
    def time_in_seconds(self) -> float:

        return self.get_clock().now().seconds_nanoseconds()[0]

def main(args = None):

    rclpy.init(args = args)
    try:
        servo = WirelessServo()
        node = WirelessServoNode(servo=servo)
        try:
            servo.start_async_execution()
            rclpy.spin(node, executor = MultiThreadedExecutor())
        finally:
            node.destroy_node()
            servo.stop_async_execution()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
