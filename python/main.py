import serial
import logging
import argparse
from datetime import datetime, timedelta
import time, sys

logger = logging.getLogger()
logger.setLevel(logging.INFO)

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
logger.addHandler(handler)

def handle_data(port:serial.Serial):
    """Infinite loop for the data over the serial port."""
    file_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S Device Logs.csv")
    logging.info("Creating file "+ file_name)
    with open(file_name, 'w') as logs:
        try:
            while True:
                data = port.read_until()
                if data is None:
                    continue
                logging.info(str(data, encoding="ascii").strip().replace('|',''))
                logs.write(str(data, encoding="ascii").strip().replace('|', "\n")) # Tried to mitigate double lines
                time.sleep(0.050)
        except KeyboardInterrupt:
            logs.write("#}")
            logging.info("Logging Ended.")
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog="ECE362 Project for Reading Telemetry from our project.",
    description="Helpers for running our project",
    epilog="Good luck!")

    parser.add_argument("port")

    args = parser.parse_args()
    
    logger.info("Opening: COM"+str(args.port))

    connection_time = timedelta(seconds=30)
    init_time = datetime.now()
    while datetime.now() - init_time < connection_time:
        try:
            with serial.Serial('COM'+str(args.port), 115200, timeout=0.5, parity=serial.PARITY_NONE, rtscts=True, xonxoff=True) as port:
                handle_data(port);
        
        except serial.SerialException:
            print("Could not open port. Retrying")

    exit()