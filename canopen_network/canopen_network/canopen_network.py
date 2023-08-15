from __future__ import annotations

import rclpy
import canopen
import yaml
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.executors import MultiThreadedExecutor
from .canopen_network_faulhaber_motor_node import FaulhaberMotorNode
from .canopen_network_canopen_master_node import CanopenMasterNode

def setup() -> list[Node]:
    
    config_file_param = rclpy.Parameter(name = 'config_file', value = '/home/srl-orin/ros2_ws/src/tbotros_config/config/can.yaml')
    config_file = config_file_param.get_parameter_value().string_value

    # load config
    with open(config_file, 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except Exception as exc:
            get_logger('setup').info('Failed loading config file: ' + str(exc))
    
    # initialize can open network
    network = canopen.Network()
    network.connect(channel = config['channel'], 
                    bustype = config['bustype'], 
                    bitrate = config['bitrate'])

    # initialize nodes
    nodes = []

    for node_config in config['nodes']:

        if node_config['type'] == 'canopen_master':
            node = CanopenMasterNode(network = network, 
                                     namespace = node_config['namespace'], 
                                     name = node_config['name'], 
                                     params = node_config['params'])
        elif node_config['type'] == 'faulhaber_motor':
            node = FaulhaberMotorNode(network = network, 
                                      namespace = node_config['namespace'], 
                                      name = node_config['name'], 
                                      params = node_config['params'])
        else:
            node = None

        nodes.append(node)

    return nodes

def main(args = None):

    rclpy.init(args = args)
    try:
        nodes = setup()
        executor = MultiThreadedExecutor(num_threads = len(nodes) + 4)
        for node in nodes:
            executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()         
            for node in nodes:
                node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':

    main()