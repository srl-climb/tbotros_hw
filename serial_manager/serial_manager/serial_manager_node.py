from __future__ import annotations

import serial
import serial.tools.list_ports
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String, Bool
from custom_srvs.srv import SerialSend


class SerialManagerNode(Node):

    def __init__(self):
        super().__init__('serial_manager')

        # serial paramters
        self._serial_port = '/dev/ttyTHS0'
        self._serial_baudrate = 115200
        self._serial_connected = False
        self._serial_msg_buffersize = 256
        self._serial_msg_startmarker = b'<'
        self._serial_msg_endmarker = b'>'
        self._serial_msg_buffer = bytearray(self._serial_msg_buffersize)
        self._serial_msg_residual = b''
        self._serial_msg_counter = 0
        self._serial_watchdog_timeout = 2
        self._serial_watchdog_msg_counter = 0

        # timers
        self._serial_read_timer = self.create_timer(0.25, self.serial_read_timer_callback)
        self._serial_watchdog_timer = self.create_timer(self._serial_watchdog_timeout, self.serial_watchdog_timer_callback)
        # publishers
        self._serial_message_pub = self.create_publisher(String, self.get_name() + '/message', 20)
        self._serial_connected_pub = self.create_publisher(Bool, self.get_name() + '/connected', 1) #qos_profile=QoSProfile(depth=1,durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL))
        # services
        self._serial_send_srv = self.create_service(SerialSend, self.get_name() + '/send', self.serial_send_callback)
        
        self._serial = serial.Serial(self._serial_port, self._serial_baudrate, timeout=0)
        self._serial.close()
        self._serial.open()
        
        self.get_logger().info('Serial port opened')
        # if tty is blocked: sudo chmod 666 /dev/ttyTHS0
        # if tty is busy: sudo fuser -k /dev/ttyTHS0
        # if permission denied: sudo chmod a+rw /dev/ttyTHS0

    def serial_read_timer_callback(self):

        # read from serial interface
        msgs = self.serial_read()

        for msg in msgs:
            try:
                serial_msg = String()
                serial_msg.data = msg.decode("utf-8")
                self._serial_message_pub.publish(serial_msg)
            except:
                self.get_logger().warn('Unable to decode serial message')

    def serial_watchdog_timer_callback(self):

        if self._serial_watchdog_msg_counter == self._serial_msg_counter and self._serial_connected:
            self.get_logger().error('Serial disconnected')
            
            self._serial_connected = False

        elif self._serial_watchdog_msg_counter != self._serial_msg_counter and not self._serial_connected:
            self.get_logger().info('Serial connected')

            self._serial_connected = True

        self._serial_watchdog_msg_counter = self._serial_msg_counter

        connected_msg = Bool()
        connected_msg.data = self._serial_connected
        self._serial_connected_pub.publish(connected_msg)

    def serial_send_callback(self, request: SerialSend.Request, response: SerialSend.Response):

        if self._serial_connected:
            self.serial_write(self._serial_msg_startmarker + request.msg.encode() + self._serial_msg_endmarker)

            self.get_logger().info('Sending message "' + request.msg + '"')
            response.success = True
        else:
            self.get_logger().error('Sending message "' + request.msg + '" failed')
            response.success = False
        
        return response

    def serial_read(self) -> list[bytes]:  
        msgs = []

        # if serial contains enough to fill the buffer
        n = self._serial.in_waiting // self._serial_msg_buffersize

        for _ in range(n):
            # read the buffer       
            self._serial_msg_buffer = self._serial.read(self._serial_msg_buffersize)
            
            # search index
            search_index = 0

            while search_index < self._serial_msg_buffersize:
                
                # find markers
                start_index = self._serial_msg_buffer.find(self._serial_msg_startmarker, search_index)
                end_index = self._serial_msg_buffer.find(self._serial_msg_endmarker, search_index)
                
                # start marker only
                if end_index < 0 and start_index >= 0:
                    self._serial_msg_residual = self._serial_msg_buffer[start_index:]
                    break
                # end marker only/end marker before start marker
                elif start_index > end_index:
                    if self._serial_msg_residual != b'':
                        msgs.append(self._serial_msg_residual + self._serial_msg_buffer[0:end_index+1])
                        self._serial_msg_counter += 1
                # end marker after start marker
                elif end_index > start_index:
                    msgs.append(self._serial_msg_buffer[start_index:end_index+1])   
                    self._serial_msg_counter += 1
                # no marker
                elif start_index < 0 and end_index < 0:
                    break

                search_index = end_index + 1
        return msgs
    
    def serial_write(self, msg: bytes):

        self._serial.write(msg)

    def __del__(self):

        if self._serial is not None:
            self._serial.close()

def main(args = None):

    rclpy.init(args = args)
    try:
        node = SerialManagerNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
