from __future__ import annotations

import rclpy
import bleak
import asyncio
import threading
import queue
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Trigger
from rclpy.executors import SingleThreadedExecutor
from time import

#https://github.com/Ladvien/arduino_ble_sense/blob/master/app.py

class Message():

    def __init__(self, data: str):
        
        self.data = data
        self.send_success = False

    def wait_for_send(self, timeout=1):


class WirelessServo():

    def __init__(self):

        self.node: WirelessServoNode = None

        # servo parameters
        self.connected = False
        self.mode = None
        self.limitswitch0 = False
        self.limitswitch1 = False
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

        # asyncio
        self.queue = queue.Queue(1)

    def disconnect_callback(self, client: bleak.BleakClient):

        self.connected = False
        self.get_logger().info(f"Disconnected from {self.name}!")

    def receive_callback(self, _, data: bytearray):

        self.mode = int(data[0])
        self.limitswitch0 = bool(data[1])
        self.limitswitch1 = bool(data[2])
        self.running = bool(data[4])
        self.error = int(data[5])
        self.closed = int(data[6])

    async def send(self):

        try:
            message = self.queue.get(block=False)

            if not self.connected:
                raise Exception('Not connected')
            else:
                self.get_logger().info(f"Sending message {message}")
                await self.client.write_gatt_char(self.txcharacteristic, message.encode('utf-8'), response=True)
        except queue.Empty:
            pass
        except Exception as exc:
            self.get_logger().error(f"Failed to send {message}: {exc}")

    async def discover(self):

        if self.device:
            return

        try:
            self.device = await bleak.BleakScanner.find_device_by_name(self.name, timeout=1)

            if self.device is not None:
                self.client = bleak.BleakClient(self.device.address, disconnected_callback = self.disconnect_callback)
                self.get_logger().info(f"Discovered {self.name}")

            else:
                self.get_logger().warn(f"Failed to discover {self.name}")

        except Exception as exc:
            self.get_logger().error(f"Failed to discover {self.name}: {exc}")

    async def connect(self):

        if self.connected or not self.client:
            return
       
        try:
            await self.client.connect()
            self.connected = self.client.is_connected
            
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

    def get_logger(self):

        return self.node.get_logger()


class WirelessServoNode(Node):

    def __init__(self, servo: WirelessServo):

        super().__init__('wireless_servo')

        # servo
        self.servo = servo
        self.servo.node = self

        # publishers
        self.servo_running_pub = self.create_publisher(Bool, self.get_name() + '/running', 1)
        self.servo_closed_pub = self.create_publisher(Int8, self.get_name() + '/closed', 1)
        self.servo_limitswitch0_pub = self.create_publisher(Bool, self.get_name() + '/limitswitch0', 1)
        self.servo_limitswitch1_pub = self.create_publisher(Bool, self.get_name() + '/limitswitch1', 1)

        # services
        self.create_service(Trigger, self.get_name() + '/clear', self.clear_srv_callback)
        self.create_service(Trigger, self.get_name() + '/reset', self.reset_srv_callback)

        # timer
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):

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

    def clear_srv_callback(self, resquest: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        
        self.send('<clear>')

        return response

    def reset_srv_callback(self, resquest: Trigger.Request, response: Trigger.Response) -> Trigger.Response:

        self.send('<reset>')

        return response

    def send(self, message: str) -> bool:

        self.servo.queue.put(message)
           
async def async_servo(servo: WirelessServo, event: threading.Event):
    try:
        while not event.is_set():
            await servo.discover()
            await servo.connect()
            await servo.send()
            await asyncio.sleep(0.5)
    finally:
        await servo.destroy()


def main(args = None):

    rclpy.init(args = args)
    try:
        servo = WirelessServo()
        node = WirelessServoNode(servo=servo)
        try:
            event = threading.Event()
            thread = threading.Thread(target=asyncio.run, args = (async_servo(servo, event),))
            thread.start()
            rclpy.spin(node, executor = SingleThreadedExecutor())
        finally:
            node.destroy_node()
            event.set()
            thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
