import asyncio
import bleak

from bleak import BleakScanner, BleakClient

 

async def notification_handler(sender, data):

    """Handles incoming BLE notifications."""

    print(f"Data received: {data.decode('utf-8')}")  # Decode from Arduino sending format

 

async def read_ble_data(device_name, characteristic_uuid):

    """Scans for the BLE device"""

    devices = await BleakScanner.discover()

    target_device = None

 

    for device in devices:

        if device.name == device_name:

            target_device = device

            break

 

    if target_device is None:

        print(f"Device '{device_name}' not found.")

        return

 

    async with BleakClient(target_device.address) as client:

        print(f"Connected to {device_name}") # Victory

 

        # Start receiving notifications

        await client.start_notify(characteristic_uuid, notification_handler)

 

        try:

            while True:  # Keep it streaming

                await asyncio.sleep(1) 

        except KeyboardInterrupt:

            print("\nDisconnecting...") # Control C on keyboard to kill it

        finally:

            await client.stop_notify(characteristic_uuid)

 

if __name__ == "__main__":

    device_name = "GyroSensor"  # Gyro name

    characteristic_uuid = "87654321-4321-4321-4321-ba0987654321" # ID in the Arduino script

 

    asyncio.run(read_ble_data(device_name, characteristic_uuid))