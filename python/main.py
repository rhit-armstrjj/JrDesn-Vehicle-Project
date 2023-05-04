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
from textual.worker import Worker, get_current_worker

device_link: VehicleLink

class ValidationError(Message):
    def __init__(self, message:str) -> None:
        self.message = message

class SerialError(Message):
    def __init__(self, message:str) -> None:
        self.message = message

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

    bt_port: serial.Serial = serial.Serial()

    class VehicleStatusUpdate(Message):
        def __init__(self, payload, conn_status):
            self.payload = payload
            self.conn_status = conn_status

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
                yield Label("Connection Status: ")
                yield Label("Disconnected", classes="error", id="conn_status")
            yield Label(id="conn_settings")

        with Vertical(classes="grid_item", id="connect_button_box"):
            yield Button("Connect!", id="connect_button")
            yield ProgressBar(show_percentage=False, show_eta=False)

        return

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

    def __toggle_indicator(self):
        prog = self.query_one(ProgressBar)
        button = self.query_one(Button)
        #ind.display = not ind.display
        prog.display = not prog.display
        button.display = not button.display

    def __check_status(self):
        status_label:Label = self.query_one("#conn_status") # type: ignore
        connect_button:Button = self.query_one("#connect_button") #type: ignore
        
        log("Checking Status")
        self.__toggle_indicator()
        if self.bt_port.is_open:
            status_label.remove_class('error')
            status_label.add_class('success')
            connect_button.add_class('error')
            connect_button.label = "Disconnect"
            
        else:
            status_label.add_class('error')
            status_label.remove_class('success')
            connect_button.remove_class('error')
            connect_button.label = "Connect!"

    @work(exit_on_error=False) # type: ignore
    def connect_to_car(self):
        pb = self.query_one(ProgressBar)
        self.app.call_from_thread(self.__toggle_indicator)

        pb.advance(1)

        baud_box:Input = self.query_one("#baud_input") # type: ignore
        stop_box:Input = self.query_one("#stopb_input") #type: ignore

        parity_en_check:Switch = self.query_one("#parity_en_sw")  # type: ignore
        parity_odd_check:Switch = self.query_one("#parity_sw") # type: ignore

        pb.advance(4)

        self.bt_port.port = self.selected_port

        #TODO Actually handle when error is raised.
        
        if not baud_box.value.isnumeric():
            raise ValueError('Alphabetical Characters in Baudrate')
        
        if not stop_box.value.isnumeric():
            raise ValueError('Alphabetical Characters in Stop Bits')
        
        self.bt_port.baudrate = int(baud_box.value)

        if parity_en_check.value:
            if parity_odd_check.value:
                self.bt_port.parity = serial.PARITY_ODD
            else:
                self.bt_port.parity = serial.PARITY_EVEN
        else:
            self.bt_port.parity = serial.PARITY_NONE

        self.bt_port.rtscts = True
        self.bt_port.xonxoff = True
        self.bt_port.timeout = 0.5

        log(self.bt_port)

        if not get_current_worker().is_cancelled:
            try:
                self.bt_port.open()
            except serial.SerialException:
                pass

        self.app.call_from_thread(self.set_timer, 0.5, self.__check_status)
        

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        if(event.button.id == "connect_button"):
            if self.bt_port.is_open:
                pass
            self.connect_to_car()

            #self.__toggle_indicator()
            # Await connection
            event.stop()


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