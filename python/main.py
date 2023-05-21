
from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal, Vertical
from textual.widgets import *
from textual.worker import Worker, get_current_worker, WorkerState
from textual.message import Message
from textual.binding import Binding

from textual import work, log
from textual.reactive import reactive
from textual.timer import Timer

import comm_link

from time_display import TimeDisplay
from cobs import cobs

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from typing import TextIO

import serial
import serial.tools.list_ports
import msgpack
import time
from datetime import datetime
import threading
import crc8

class ConnectionSettings(Static):
    """Lets user select the port to use."""

     #### MESSAGES
    class VehicleStateChanged(Message):
        def __init__(self, id:int, data:dict) -> None:
            self.data:dict = data
            self.id:int = id
            super().__init__()

    class ConnectedStateChange(Message):
        def __init__(self, connected:bool) -> None:
            self.connected:bool = connected
            super().__init__()

    selected_port: str|None = None
    try_connecting: bool = False

    connection_handle: Worker = None
    connection_handle_lock: threading.Lock = threading.Lock()

    bt_handle: serial.Serial
    bt_handle_lock: threading.Lock = threading.Lock()

    packets_sent = reactive(0)
    packets_received = reactive(0)

    telemetry_timer: Timer

    def recurring_telemetry(self):
        self.transmission_worker(comm_link.REQUEST_TELEMETRY, None)

    def set_connected_state(self, connected_state:int) -> None:
        """Sets the connect button based on state: 
            0: Connect!
            1: Cancel (warn)
            2: Disconnect (warn)
        """
        button:Button = self.query_one('#connect_button')
        log("Setting connect button state to "+str(connected_state))
        if connected_state == 2:
            button.label = "Disconnect"
            self.app.query_one(ControlPanel).disabled = False  # TODO Change to messsage post to parent, but oh well.
            self.packets_received = 0
            self.packets_sent = 0
            log("Starting telemetry timer.")
            self.recurring_telemetry()
            self.telemetry_timer.reset()
            self.telemetry_timer.resume()
        elif connected_state == 1:
            button.label = "Cancel"
            button.remove_class("succ")
            button.add_class("warning")
        elif connected_state == 0:
            button.label = "Connect!"
            self.app.query_one(ControlPanel).disabled = True
            button.add_class("succ")            
            button.remove_class("warning")
            log("Stopping telemetry timer.")
            self.telemetry_timer.pause()

    def set_error_text_state(self, text:str|None) -> None:
        error_text_container: Horizontal = self.query_one("#error_text_container")
        error_text: Label = self.query_one("#error_text_label")

        if text is None:
            error_text_container.visible = False
        else:
            error_text.update(text)
            error_text_container.visible = True
            
    def compose(self) -> ComposeResult: 
        with Container(classes="grid_item"): # ports
            yield Label("Port:", classes="highlight")
            yield OptionList(id="port-select")

        with Vertical(classes="grid_item"): #Connection Settings
            yield Label("Connection Settings:", classes="highlight")
            with Horizontal():
                yield Label("Baudrate:")
                yield Input(placeholder="115200", id="baud_input", classes="form_data")
            
            with Horizontal():
                yield Label("Stop Bits: ")
                yield OptionList("1", "1.5", "2", id="stopb_choice")

            with Horizontal():
                yield Label("Parity Enable: ")
                yield Switch(id="parity_en_sw", classes="form_data")
                yield Label("Odd Parity:", classes="parity_dep")
                yield Switch(id="parity_sw", classes="parity_dep form_data", disabled=True)

        with Vertical(classes="grid_item"): # Connection Status
            yield Label("Connection Stats:", classes="highlight")
            yield Label("Packets Sent: 0", id="packets_sent")
            yield Label("Packets received: 0", id="packets_received")

        with Vertical(classes="grid_item", id="connect_button_box"):
            yield Button("Connect!", id="connect_button", classes="succ")
            with Horizontal(id="error_text_container"):
                yield Label("Error: ")
                yield Label("", classes="error", id="error_text_label")

        return

    ### EVENTS

    def on_mount(self) -> None:
        ports = self.query_one("#port-select")
        for port in serial.tools.list_ports.comports():
            ports.add_option(port.name)

    def on_switch_changed(self, event: Switch.Changed):
        if event.switch.id == "parity_en_sw":
            options = self.query(Switch).filter('.parity_dep')
            for sw in options.results():
                sw.disabled = not sw.disabled
            event.stop()

    def on_option_list_option_selected(self, event:OptionList.OptionSelected) -> None:
        self.selected_port = str(event.option.prompt)
        log("Selected Port: ", self.selected_port)
        event.stop()

    def on_worker_state_changed(self, event: Worker.StateChanged) -> None:
        if event.worker.name == 'connection_worker':
            if event.state != WorkerState.SUCCESS:
                self.set_connected_state(0)
            
            if event.state == WorkerState.ERROR:
                self.set_error_text_state(str(event.worker._error))
         
    async def on_button_pressed(self, event: Button.Pressed) -> None:
        if(event.button.id == "connect_button"):
            self.handle_connection()
            event.stop()
        return
    
    # TODO if on_vehicle_state_changed doesnt work, un-comment out.
    # async def on_vehicle_state_changed(self, message: VehicleStateChanged) -> None:
        # log("State Changed!")
        # control_panel:ControlPanel = self.app.query_one("#control_panel")
        # control_panel.post_message(message)

    def handle_connection(self):
        """Handles when the connection button is pressed."""
        self.try_connecting = not self.try_connecting
        if self.try_connecting or self.connection_handle.is_finished:
            self.set_error_text_state(None)
            self.connection_worker()
            self.telemetry_timer = self.set_interval(0.1, self.recurring_telemetry, pause=True) # telemetry interval timer
        else:
            with self.connection_handle_lock:
                log("Cancelling connection handle.")
                self.connection_handle.cancel()
                self.connection_handle = None

    def receive_telemetry(self, port:serial.Serial, unpacker: msgpack.Unpacker, worker: Worker):
        incoming:bytes = bytes()

        log("Waiting for incoming byte.")       
        while not worker.is_cancelled: # Receive until null byte & not canceled
            b = port.read(1)
            if b == b'\x00':
                break
            incoming += b

        if worker.is_cancelled:
            return
        
        hash = crc8.crc8()
        incoming = cobs.decode(incoming)

        log("Incoming packet: "+ str(incoming))

        hash.update(incoming[1:-1]) # cut out the id and crc8
        if hash.digest() != int.to_bytes(incoming[-1]):
            log("Packet does not match digest. -> "+ str(incoming[-1]) +" =/=" + str(hash.digest()))
            return
        
        log("Successfully found a packet!")
        self.packets_received += 1

        unpacker.feed(incoming[1:-1]) # Skip id & crc
        for o in unpacker:
            log("Object: "+str(o))
            state = self.VehicleStateChanged(incoming[0], o)
            self.app.post_message(state)

    ### REACTIVE

    def watch_packets_sent(self, packets):
        self.query_one("#packets_sent").update(f"Packets Sent: {packets}")
        pass

    def watch_packets_received(self, packets):
        self.query_one("#packets_received").update(f"Packets received: {packets}")
        pass

    ### WORKER #################### WORKER ###################

    @work(exit_on_error=False)
    def connection_worker(self):
        # Set the current worker in case needs canceling.
        self.app.call_from_thread(self.set_connected_state, 1)
        worker = get_current_worker()
        with self.connection_handle_lock:
            self.connection_handle = worker

        ### Check that all info is ready for connection
        if self.selected_port == None:
            raise ValueError("Please select a port.")
        
        rate:Input = self.query_one("#baud_input")
        stopb:OptionList = self.query_one("#stopb_choice")
        
        log("Evaluating Baudrate Input: " + str(rate.value))
        if not rate.value.strip().isnumeric():
            raise ValueError("The baudrate is not numeric.")
        rate_val = rate.value.strip()

        stopb:OptionList = self.query_one("#stopb_choice")
        if stopb.highlighted is None:
            raise ValueError("Please select a stopbit value.")
        stopb_val = int(stopb.get_option_at_index(stopb.highlighted).prompt)

        parity_en = self.query_one("#parity_en_sw")
        parity_odd = self.query_one("#parity_sw")

        parity = serial.PARITY_NONE

        if parity_en.value:
            if parity_odd.value:
                parity = serial.PARITY_ODD
            else:
                parity = serial.PARITY_EVEN

        log("New Serial Port Opening")
        with serial.Serial(self.selected_port, baudrate=rate_val, stopbits=stopb_val, parity=parity, timeout=0.1) as bt_port:
            self.app.call_from_thread(self.set_connected_state, 2)
            # Write Handle to Widget so that we can send data through it.
            with self.bt_handle_lock:
                self.bt_handle = bt_port
            
            self.app.post_message(self.ConnectedStateChange(True))
            unpacker = msgpack.Unpacker()
            while not worker.is_cancelled:
                log("Receive Loop")
                self.receive_telemetry(bt_port, unpacker, worker)
                time.sleep(0.01)
        
        log("Successfully disconnected.")
        self.app.post_message(self.ConnectedStateChange(False))

    @work(exit_on_error=False)
    def transmission_worker(self, id:int, payload:dict):
        self.packets_sent += 1
        if self.bt_handle.is_open:
            hash = crc8.crc8()
            if payload == None:
                log("None Payload Gotten")
                payload = {}

            payload.update({"packet_id": self.packets_sent})
            bites:bytes = msgpack.packb(payload) # msgpack bytes of packet
            log(f"Packet: ID {id} "+ str(bites))
            # Create bytes for msgpack |id| msgpack | crc8 (of only the msgpack) |
            hash.update(bites)
            data_bytes = cobs.encode(int.to_bytes(id) + bites + hash.digest())
            log("Writing: "+ str(data_bytes))
            self.bt_handle.write(data_bytes)
            self.bt_handle.write(bytes(1)) # Write a null byte. This is probably not the best way but this way works :)
            time.sleep(0.001)

class ControlPanel(Static):
    """Layout for all the controls for the vehicle."""
    """Needs: PID Steering Tuning, Current Speed Percentage, Current Race Time"""
    """Needs: Camera Learned Status, Internal Drive State"""

    Ksp = reactive(0.0)
    Ksi = reactive(0.0)
    Ksd = reactive(0.0)
    power = reactive(0.0)
    speed = reactive(0.0)

    class RaceStateChanged(Message):
        def __init__(self, started=False):
            self.started = started
            super().__init__()
            

    def on_mount(self):
        self.disabled = True

    def compose(self):
        with Horizontal():
            with Vertical(classes="grid_item"):
                yield TimeDisplay(id="millis")
                yield TimeDisplay(id="car_time")
                yield Label("Voltage: 0 V", id="voltage")
                yield Label("Current: 0 mA", id="current")
                yield Label("Power: 0 mW", id="power")
                yield Label("Energy: 0 J", id="energy")
                yield Label("Speed: 0", id="speed")
                yield Label("Steering: 0", id="steering")

            with Container(classes="grid_item"):
                yield Label("Race Controls")
                yield Button("Start", id="start_car_button", classes="success")
                yield Button("Stop", id="stop_car_button", classes="error")
            with Vertical(classes="grid_item"):
                yield Label("PID Tuning")
                yield Input(placeholder="Ksp", id="Ksp")
                yield Input(placeholder="Ksi", id="Ksi")
                yield Input(placeholder="Ksd", id="Ksd")
                yield Button("Update", id="update_pid_button")

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        connection = self.app.query_one(ConnectionSettings)
        start:Button = self.query_one("#start_car_button")
        stop:Button = self.query_one("#stop_car_button")

        log("Start/Stopping Race: "+ str(event.button.label))

        if event.button.id == "start_car_button":
            connection.transmission_worker(id=comm_link.START_RACE, payload={'race_duration':90_000})
            self.add_class("started")
            stop.focus()
            self.app.post_message(self.RaceStateChanged(True))
            event.stop()
        elif event.button.id == "stop_car_button":
            connection.transmission_worker(id=comm_link.STOP_RACE, payload={"Race?":1})
            self.remove_class("started")
            start.focus()
            self.app.post_message(self.RaceStateChanged(False))
            event.stop()
        elif event.button.id == "update_pid_button":
            self.verify_inputs()
            event.stop()
        return
    
    ### REACTIVE

    def watch_packets_sent(self, packets):
        self.query_one("#packets_sent").update(f"Packets Sent: {packets}")
        pass

    def watch_packets_received(self, packets):
        self.query_one("#packets_received").update(f"Packets received: {packets}")
        pass

    ### Workers

    @work(exclusive=True)
    async def verify_inputs(self):
        """Validate the inputs of the PID queries. If not successful, throw an error. (Workers are graceful for errors)"""
        connection = self.app.query_one(ConnectionSettings)
        inputs = self.query(Input).results()
        values:dict = dict()
        for input in inputs:
            values.update({input.id: float(input.value)})
            
        log(f"Values from PID: {values}")    
        connection.transmission_worker(comm_link.UPDATE_PID, values)

    async def on_connection_settings_vehicle_state_changed(self, message: ConnectionSettings.VehicleStateChanged) -> None:
        payload = message.data
        log(f"Processing Vehicle State Changes: Id: {message.id}")
        if message.id == comm_link.VEHICLE_STATUS:
            timedisplay = self.query_one("#millis")
            racedisplay = self.query_one("#car_time")
            timedisplay.time = payload["ms"]
            racedisplay.time = payload["raceTime"]
            self.query_one("#voltage").update("Voltage: {:.2f} V".format(payload['busVoltage']))
            self.query_one("#current").update("Current: {:.2f} mA".format(payload['current']))
            self.query_one("#power").update("Power: {:.2f} mW".format(payload['power']))
            self.query_one("#energy").update("Energy: {:.2f} J".format(payload['energySpent']))  
            self.query_one("#speed").update("Speed: {}".format(payload['driveSpeed']))
            self.query_one("#steering").update("Steering: {}".format(payload['servoSetting']-90))

        message.stop()
    
    async def on_connected_state_changed(self, message: ConnectionSettings.ConnectedStateChange) -> None:
        if not message.connected:
            self.query_one("#stop_car_button").action_press()


class CarControlApp(App):

    CSS_PATH = "css/app.css"
    TITLE = "Car Controller"

    figure:plt.Figure = None
    voltAxis: plt.Axes = None
    time:list = []
    voltage:list = []

    file_handle = None

    BINDINGS = [
        Binding(key="q", action="quit", description="Quit the app"),
    ]

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()
        yield Footer()

        with Horizontal(id="buttons"):  
            yield Button("Connection Settings", id="ports")  
            yield Button("Control Panel", id="control_panel")

        with ContentSwitcher(initial="ports"):
            yield ConnectionSettings(id = "ports")
            yield ControlPanel(id = "control_panel")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        switcher = self.query_one(ContentSwitcher)
        b_id = event.button.id
        log("Button ID: "+str(b_id))
        if b_id is None:
            return
        
        if not switcher.get_child_by_id(b_id) is None:
            switcher.current = b_id

    def on_connection_settings_vehicle_state_changed(self, event: ConnectionSettings.VehicleStateChanged):
        log("Parent Recieved Change")
        ctrls:ControlPanel = self.query_one(ControlPanel)
        ctrls.post_message(event)

        if event.data.get("state", 0) == 0:
            pass

        if event.data.get("state", 0) == 1:
            # Start graphing/plotting
            self.time.insert(0)
            self.voltage.insert(event.data["busVoltage"])
            self.voltAxis.scatter(self.time, self.voltage)
            
            for k in event.data.keys():
                self.file_handle.write(str(k) + ", ")
            self.file_handle.write("\n")
            for k in event.data.values():
                self.file_handle.write(str(k) + ", ")
                self.file_handle.write("\n")

        elif event.data.get("state", 0) == 2:
            # Record data.
            self.time.append(event.data["raceTime"])
            self.voltage.append(event.data["busVoltage"])
            self.figure.canvas.draw()
            log(f"File Exists: {self.file_handle}, Closed: {self.file_handle.closed}")
            if self.file_handle.closed:
                return
            
            if not self.printed_header:
                self.printed_header = True
                for k in event.data.keys():
                    self.file_handle.write(str(k) + ", ")
                self.file_handle.write("\n")

            for k in event.data.values():
                self.file_handle.write(str(k) + ", ")

            self.file_handle.write("\n")

    def on_connected_state_changed(self, event: ConnectionSettings.ConnectedStateChange):
        ctrls:ControlPanel = self.query_one(ControlPanel)
        ctrls.post_message(event)

    def on_control_panel_race_state_changed(self, event: ControlPanel.RaceStateChanged):
        if event.started:
            file_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S Device Logs.csv")
            log("Creating file "+ str(file_name))
            self.file_handle = open(file_name, 'w')
            if self.figure != None:
                plt.close(self.figure)
            if self.file_handle.closed:
                log("WTF")
            (figure, axes) = plt.subplots(2,2)
            self.figure = figure
            self.voltAxis = axes[0,0]
            self.steeringAxis = axes[1,0]
            self.figure.canvas.draw()
            # plt.show(block=False)
            self.printed_header = False
        else:
            log("Closing File Handle")
            self.file_handle.close()


    @work(exclusive=True)
    def beginDataCollection(self):
        pass


# Entrypoint
if __name__ == '__main__':
    app = CarControlApp()
    app.run()