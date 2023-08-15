from __future__ import annotations

from .canopen_network_base_node import BaseNode

class CanopenMasterNode(BaseNode):

    def __init__(self, **args) -> None:
        
        super().__init__(**args)

        self._syncrate = self._params.get('syncrate', 0.02)

        # timer
        self._start_sync_timer = self.create_timer(1, self.start_sync_callback)
        self._network.bus.channel_info
        
        self.get_logger().info('Initialization completed')

    def start_sync_callback(self):

        # start periodic transmission of SYNC message in background thread
        self._network.sync.start(self._syncrate) # in seconds
        # NOTE: The SYNC message transmission is started after the initialization of the node using a one time timer
        #       Starting the transmission during the initialiazation leads to SDO errors when other ros-nodes configure can-nodes in the network       

        # destroy the timer as we only want to run it once
        self.destroy_timer(self._start_sync_timer)

    def destroy_node(self) -> bool:

        self._network.sync.stop()

        super().destroy_node()