
import logging

error_log_path = 'error.log'

fh = logging.FileHandler(error_log_path, encoding='UTF-8', mode="a")  


logging.basicConfig(
            level=logging.INFO,
            format='[%(asctime)s] %(levelname)s - %(message)s',
            handlers = [fh])

from uhf.entrypoint import UHFServices
import os
from uhf.entrypoint import S

app = UHFServices.app
app.hComm = 0
app.inventory = False

if __name__ == "__main__":
    LISTENING_HOST = "0.0.0.0"
    LISTENING_PORT = 8888
    S.serve(host=LISTENING_HOST, port=LISTENING_PORT, debug=False)