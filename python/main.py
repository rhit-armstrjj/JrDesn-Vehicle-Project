
from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal, Vertical
from textual.widgets import *
from textual.worker import Worker, get_current_worker, WorkerState

from textual import work, log
from textual.reactive import reactive
from textual.timer import Timer

import comm_link
from comm_link import VehicleStateChanged

from time_display import TimeDisplay

import serial
import serial.tools.list_ports
import msgpack
import time
import threading

class DataPacket():
    packet_id:int = 0
    payload_id: int
    payload: dict

    def __init__(self, payload_id, payload, packet_id = 0) -> None:
        self.payload_id = payload_id
        self.payload = payload
        self.packet_id = packet_id

    def asdict(self):
        return {'packet_id': self.packet_id, 'payload_id': self.payload_id, 'payload': self.payload}

class ControlPanel(Static):
    """Layout for all the controls for the vehicle."""
    """Needs: PID Steering Tuning, Current Speed Percentage, Current Race Time"""
    """Needs: Camera Learned Status, Internal Drive State"""
    
    def on_vehicle_state_changed(self, event: VehicleStateChanged) -> None:

        pass

    def on_mount(self):
        self.disabled = True

    def compose(self):
        with Horizontal():
            with Container(classes="grid_item"):
                yield TimeDisplay()   
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

        if event.button.id == "start_car_button":
            connection.transmission_worker(id=comm_link.START_RACE, payload={})
            self.add_class("started")
            stop.focus()
            event.stop()
        elif event.button.id == "stop_car_button":
            connection.transmission_worker(id=comm_link.STOP_RACE, paylaod={})
            self.remove_class("started")
            start.focus()
            event.stop()
        elif event.button.id == "update_pid_button":
            inputs = self.query(Input).results()
            pids = []
            

            connection.transmission_worker(comm_link.UPDATE_PID, {'new_pid':[]})
            event.stop()
            raise NotImplementedError("PID Update Method Not Completed")
        return

    async def on_vehicle_state_changed(self, message: VehicleStateChanged) -> None:
        data = message.vehicle_data
        payload = data["payload"]

        if data["payload_id"] == 0:
            timedisplay = self.query_one(TimeDisplay)
            timedisplay.time = payload["millis"]
            pass     

        pass

class ConnectionSettings(Static):
    """Lets user select the port to use."""

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
        self.transmission_worker(comm_link.REQUEST_TELEMETRY, {})

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
            self.app.query_one(ControlPanel).disabled = False
            self.packets_received = 0
            self.packets_sent = 0
            log("Starting telemetry timer.")
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

        self.telemetry_timer = self.set_interval(0.05, self.recurring_telemetry, pause=True)

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
    
    async def on_vehicle_state_changed(self, message: VehicleStateChanged) -> None:
        log("State Changed!")
        control_panel:ControlPanel = self.app.query_one("#control_panel")
        control_panel.post_message(message)

    def handle_connection(self):
        """Handles when the connection button is pressed."""
        self.try_connecting = not self.try_connecting
        if self.try_connecting or self.connection_handle.is_finished:
            self.set_error_text_state(None)
            self.connection_worker()
        else:
            with self.connection_handle_lock:
                self.connection_handle.cancel()
                self.connection_handle = None

    def receive_telemetry(self, port:serial.Serial, unpacker: msgpack.Unpacker, worker: Worker):
        incoming = port.read_all()
        unpacker.feed(incoming)
        for o in unpacker:
            state = VehicleStateChanged(o)
            self.post_message(state)

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
        with serial.Serial(self.selected_port, baudrate=rate_val, stopbits=stopb_val, parity=parity) as bt_port:
            self.app.call_from_thread(self.set_connected_state, 2)
            # Write Handle to Widget so that we can send data through it.
            with self.bt_handle_lock:
                self.bt_handle = bt_port
            
            unpacker = msgpack.Unpacker()
            while not worker.is_cancelled:
                self.receive_telemetry(bt_port, unpacker, worker)
                time.sleep(0.01)
        

        raise NotImplementedError("The connection_worker method has not been completely implemented yet.")
        # self.app.call_from_thread(self.set_connected_state, False)

    @work(exit_on_error=False)
    def transmission_worker(self, id:int, payload:dict):
        self.packets_sent += 1
        if self.bt_handle.is_open:
            packet = DataPacket(id, payload, packet_id=self.packets_sent)
            bites = msgpack.packb(packet.asdict())
            self.bt_handle.write(bites)
            time.sleep(0.001)

class CarControlApp(App):

    CSS_PATH = "css/app.css"
    TITLE = "Car Controller"

    BINDINGS = []

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

    def on_vehicle_state_changed(self, event: VehicleStateChanged):
        ctrls:ControlPanel = self.query_one(ControlPanel)
        ctrls.on_vehicle_state_changed(event)

# Entrypoint
if __name__ == '__main__':
    app = CarControlApp()
    app.run()