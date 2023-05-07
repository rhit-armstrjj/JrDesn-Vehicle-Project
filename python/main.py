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

class ControlPanel(tw.Widget):
    """Layout for all the controls for the vehicle."""
    """Needs: PID Steering Tuning, Current Speed Percentage, Current Race Time"""
    """Needs: Camera Learned Status, Internal Drive State"""
    
    def compose(self):
        yield Label("Control Panel", classes="header")
        # yield Button("Start", id="start", variant="success")
        # yield Button("Stop", id="stop", variant="error")
        # yield Input(placeholder="Ksp")
        # yield Input(placeholder="Ksi")
        # yield Input(placeholder="Ksd")
        # yield TimeDisplay("00:00.00")

class ConnectionSettings(Static):
    """Lets user select the port to use."""

    selected_port: str|None = None
    try_connecting: bool = False

    connection_handle: Worker = None
    connection_handle_lock: threading.Lock = threading.Lock()

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
                yield Input(placeholder="0, 1, 2", id="stopb_input", classes="form_data")

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
        ports = self.query_one(OptionList)
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

    ### WORKER

    @work(exclusive=True, exit_on_error=False)
    def connection_worker(self):
        worker = get_current_worker()
        self.app.call_from_thread(self.set_connected_state, True)
        with self.connection_handle_lock:
            self.connection_handle = worker

        count = 0

        while not worker.is_cancelled and count < 5:
            log("Working " + str(count) + '/5')
            time.sleep(2)
            count += 1

        raise ValueError("Get Good")
        # self.app.call_from_thread(self.set_connected_state, False)

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
            yield ControlPanel(id = "panel")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        switcher = self.query_one(ContentSwitcher)
        b_id = event.button.id
        if b_id is None:
            return
        
        if not switcher.get_child_by_id(b_id) is None:
            switcher.current = b_id

# Entrypoint
if __name__ == '__main__':
    app = CarControlApp()
    app.run()