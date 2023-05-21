
"""
    Data Packet Structure !! OUT OF DATE !!
    ---------------------

    packet id: int # Number id of packet
    rest of packet is a map.
    {struct}
"""

"""
    Error Message Struct (Payload)
    payload_id: -1
    --------------------
    Message: string
"""
ERROR = -1
"""
    Vehicle Data Struct (Payload)
    payload_id: 0
    -------------------
    millis: int
    race_time: int

    Arrow: True | False # Do not use the following arrow data if false
    Arrow Head: int
    Arrow Tail: int

    
    Speed: int
    Servo Value: int

    Battery Voltage: float
    Total Current: float
    power
    energy

    P: float
    I: float
    D: float
"""
VEHICLE_STATUS = 32

"""
    Steering PID Update Command (Payload)
    id = 1
    ------------------
    p: float # p, i, d
    i: float
    d: float
"""
UPDATE_PID = 69

"""
    Start Race Command (Payload)
    id = 2
    ------------------
    race_duration (ms): int
"""
START_RACE = 2

"""
    Stop Command (Payload)
    id = 3
    ----------------------
    (intentionally empty)
"""
STOP_RACE = 3

"""
    Request Telemetry Command (Payload)
    id = 4
    ----------------------
    (intentionally empty)
"""
REQUEST_TELEMETRY = 52