import msgpack
import serial

"""
    Data Packet Structure
    ---------------------

    packet id: int
    packets sent: int
    error: True | False
    payload_id: int
    payload: {struct}
"""

"""
    Vehicle Data Struct (Payload)
    payload_id: 0
    -------------------
    Arrow: True | False # Do not use data if false
    Arrow Head: int
    Arrow Tail: int
    
    Speed: int
    Servo Value: int

    Battery Voltage: float
    Total Current: float

    Steering PID: float[3]
"""

"""
    Steering PID Update Command (Payload)
    id = 1
    ------------------
    New P: float
    New I: float
    New D: float
"""

"""
    Start Race Command (Payload)
    id = 2
    ------------------
    race_duration (ms): int
"""

"""
    Stop Command (Payload)
    id = 3
    ----------------------
    (intentionally empty)
"""

class VehicleLink():
    """
        Manages communications between the Car and the UI
        Polling from the UI will get the messages.
    """
    unpacker: msgpack.Unpacker = msgpack.Unpacker()
    port: serial.Serial

    baud_rate = 115200
    stop_bits = 0
    parity = serial.PARITY_NONE

    async def connect(self):
        pass

    async def poll_serial():
        pass