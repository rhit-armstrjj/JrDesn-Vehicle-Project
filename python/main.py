import serial
import serial.tools.list_ports

from comm_link import VehicleLink

import textual.widget as tw
from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal, Vertical
from textual.widgets import *
from textual.reactive import reactive
from textual import log

from textual import work
from textual.message import Message
from textual.worker import Worker, get_current_worker, WorkerState

import msgpack
import time
import threading

class VehicleStateChanged(Message):
    def __init__(self, data:dict) -> None:
        self.vehicle_data:dict = data

class ControlPanel(tw.Widget):
    """Layout for all the controls for the vehicle."""
    """Needs: PID Steering Tuning, Current Speed Percentage, Current Race Time"""
    """Needs: Camera Learned Status, Internal Drive State"""
    
    def on_vehicle_state_changed(self, event: VehicleStateChanged) -> None:
        pass

    def compose(self):
        yield Label("Control Panel", classes="header")
        yield Button("Start", id="start", class="success")
        yield Button("Stop", id="stop", class="error")
        yield Input(placeholder="Ksp")
        yield Input(placeholder="Ksi")
        yield Input(placeholder="Ksd")
        yield TimeDisplay("00:00.00")

    async def on_vehicle_state_changed(self, message: VehicleStateChanged) -> None:
        pass

class ConnectionSettings(Static):
    """Lets user select the port to use."""

    selected_port: str|None = None
    try_connecting: bool = False

    connection_handle: Worker = None
    connection_handle_lock: threading.Lock = threading.Lock()

    bt_handle: serial.Serial
    bt_handle_lock: threading.Lock = threading.Lock()

    def set_connected_state(self, connected:bool) -> None:
        button:Button = self.query_one('#connect_button')
        if connected:
            button.label = "Disconnect"
            button.remove_class("succ")
            button.add_class("warning")
        else:
            button.label = "Connect!"
            button.add_class("succ")            
            button.remove_class("warning")

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
            with Horizontal():
                pass
            yield Label(id="conn_settings")

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
                self.set_connected_state(False)
            
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

    ### WORKER #################### WORKER ###################

    @work(exit_on_error=False)
    def connection_worker(self):
        # Set the current worker in case needs canceling.
        self.app.call_from_thread(self.set_connected_state, True)
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


        while not worker.is_cancelled:
            log("New Serial Port Opening")
            with serial.Serial(self.selected_port, baudrate=rate_val, stopbits=stopb_val, parity=parity) as bt_port:
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
    def transmission_worker(self, data:dict):
        if self.bt_handle.is_open:
            bites = msgpack.pack(data)
            self.bt_handle.write(bites)
            

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
            yield Button("Control Panel", id="panel") 
            #yield Button("Raw Terminal", id="terminal")

        with ContentSwitcher(initial="ports"):
            yield ConnectionSettings(id = "ports")
            yield ControlPanel(id = "control_panel")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        switcher = self.query_one(ContentSwitcher)
        b_id = event.button.id
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