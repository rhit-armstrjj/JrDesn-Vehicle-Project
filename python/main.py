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
    file_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S Device Logs.csv")
    logging.info("Creating file "+ file_name)
    with open(file_name, 'w') as logs:
        while True:
            data = port.read_all()
            if data is None:
                continue
            logging.info(str(data, encoding="ascii").strip())
            logs.write(str(data, encoding="ascii").strip().replace('|', '\r\n'))
            time.sleep(0.050)
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog="ECE230 Term Project Z&A Encryption Support",
    description="Helpers for running our project",
    epilog="Good luck!")

    parser.add_argument("port")
    # parser.add_argument("")

    args = parser.parse_args()
    
    #pkeyGen.encryp_file('C:\\Users\\armstrjj\\workspace_v12\\ece230TermProjectZiemer&Armstrong\\python_app\\NeverGonnaGiveYouUp.txt', outfile='NeverGonnaGiveYouUp.hash')
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