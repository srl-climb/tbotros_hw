from __future__ import annotations

from rclpy_wrapper.node import Node2
import canopen

class BaseNode(Node2):

    def __init__(self, network: canopen.Network = None, name: str = '', namespace: str = '', params: dict = {}):
        
        super().__init__(node_name = name, namespace = namespace)

        self._network = network
        self._params = params