from textual.widgets import Static
from textual.reactive import reactive

class TimeDisplay(Static):
    """Display a time formatted integer in milliseconds."""
    time = reactive(0)

    def watch_time(self, time: int) -> None:
        """Called when time changes"""
        seconds, mils = divmod(time, 1000)
        mins, seconds = divmod(seconds, 60)
        self.update(f"{mins:02.0f}:{seconds:02.0f}.{mils:03.0f}")
