import asyncio

import serial
import serial.tools.list_ports
import argparse
from datetime import datetime, timedelta
import time, sys


from comm_link import CommLink

from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal, Vertical
import textual.widget as tw
from textual.widgets import *
from textual.reactive import reactive
from textual import log

CAR_LINK: CommLink = CommLink()

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

        with Vertical(classes="grid_item", id="connect_button"):
            yield Button("Connect!", id="connect_button")
            yield ProgressBar()

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

    def toggle_indicator(self):
        prog = self.query_one(ProgressBar)
        button = self.query_one(Button)
        #ind.display = not ind.display
        prog.display = not prog.display
        button.display = not button.display

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        
        self.toggle_indicator()

        # Add worker that awaits connection

        self.set_timer(5.0, self.toggle_indicator)
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