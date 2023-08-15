from __future__ import annotations

from rclpy.node import Node
import canopen

class BaseNode(Node):

    def __init__(self, network: canopen.Network = None, name: str = '', namespace: str = '', params: dict = {}):
        
        super().__init__(node_name = name, namespace = namespace)

        self._network = network
        self._params = params