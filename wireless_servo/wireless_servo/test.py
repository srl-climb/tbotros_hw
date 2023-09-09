from __future__ import annotations
import asyncio
import bleak

#https://github.com/Ladvien/arduino_ble_sense/blob/master/app.py

class WirelessServoConnection:

    client: bleak.BleakClient = None

    def __init__(self, loop: asyncio.AbstractEventLoop):

        self.loop = loop
        
        # device state
        self.device = None
        self.device_connected = False
        self.device_name = 'wireless_servo0'
        self.device_mode = None
        self.device_limitswitch0 = False
        self.device_limitswitch1 = False
        self.device_running = False
        self.device_error = False
        self.device_closed = int(0)
        self.device_error = int(0)

        # characteristics and services:
        self.service = None
        self.rxcharacteristic = None
        self.txcharacteristic = None

    def disconnect_callback(self, client: bleak.BleakClient):

        self.device_connected = False
        print(f"Disconnected from {self.device.name}!")

    def receive_callback(self, _, data: str):

        self.mode = int(data[0])
        self.device_limitswitch0 = bool(data[1])
        self.device_limitswitch1 = bool(data[2])
        self.device_running = bool(data[4])
        self.device_error = int(data[5])
        self.device_closed = int(data[6])

    async def discover(self):

        if self.device:
            return

        try:
            self.device = await bleak.BleakScanner.find_device_by_name(self.device_name)
            if self.device is not None:
                self.client = bleak.BleakClient(self.device.address, disconnected_callback = self.disconnect_callback, loop=self.loop)
            else:
                print(f"Failed to discover {self.device_name}")
        except Exception as exc:
            print('Discover:', exc)

    async def connect(self):

        if self.device_connected or not self.client:
            return
       
        try:
            await self.client.connect()
            self.device_connected = self.client.is_connected
            
            if self.device_connected:
                for service in self.client.services:
                    if service.uuid == "7def8317-7300-4ee6-8849-46face74ca2a":
                        self.service = service
                for characteristic in self.service.characteristics:
                    if characteristic.uuid == "7def8317-7301-4ee6-8849-46face74ca2a":
                        self.rxcharacteristic = characteristic
                    elif characteristic.uuid == "7def8317-7302-4ee6-8849-46face74ca2a":
                        self.txcharacteristic = characteristic
                await self.client.start_notify(self.rxcharacteristic, self.receive_callback)

                print(F"Connected to {self.device.name}")
            else:
                print(f"Failed to connect to {self.connected_device.name}")
        except Exception as exc:
            print('Connect:', exc)

    async def destroy(self):
        if self.client:
            try:
                await self.client.stop_notify(self.rxcharacteristic)
            except Exception as exc:
                print(exc)
            finally:
                await self.client.disconnect()

    async def manager(self):
        
        print('starting connection manager')

        while True:
            await self.discover()
            await self.connect()
            await asyncio.sleep(1)




if __name__ == "__main__":
    # Create the event loop.
    loop = asyncio.get_event_loop()

    connection = WirelessServoConnection(loop)
    try:
        asyncio.ensure_future(connection.manager())
        loop.run_forever()
    except KeyboardInterrupt:
        print()
        print("User stopped program.")
    finally:
        print("Disconnecting...")
        loop.run_until_complete(connection.destroy())
