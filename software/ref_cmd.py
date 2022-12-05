import json, time, queue
import multiprocessing as mp    
import websocket as wsc
import robot_utilities

class Referee_cmd_client:
    def __init__(self, logger):
        # TODO - Get IP and Port through config and command line arguments
        self.ip = "192.168.3.19" #"192.168.3.220"
        self.port = "8222"
        self.logger = logger
        self.queue = mp.Queue()
        self.logger.LOGI("Referee cmd client initialized")

    def open(self):
        self.ws = wsc.WebSocket()
        self.connect()
        self.process = mp.Process(target=self.listen, args=())
        self.process.start()
        self.logger.LOGI("Referee cmd client started")

    def close(self):
        self.process.join()
        self.ws.close()
        self.logger.LOGI("Referee cmd client closed")

    def connect(self):
        for i in range(10): # Make 10 attempts at connecting
            try:
                self.ws.connect("ws://" + self.ip + ":" + self.port)
            except ConnectionRefusedError:
                print("Error")
                time.sleep(2)
                continue
            else:
                print("No error")
                return True
                break
        return False

    def get_cmd(self):
        try:
            return self.queue.get_nowait()
        except queue.Empty:
            return None

    def listen(self):
        self.logger.LOGI("Listening started")
        while True:
            try:
                msg = self.ws.recv()
            except wsc.WebSocketConnectionClosedException:
                self.logger.LOGE("Lost connection to referee server")
                if self.connect():
                    self.logger.LOGI("Reconnected to referee server")
                    continue
                else:
                    self.logger.LOGE("Could not reconnect to referee server")
                    break
            try:
                self.queue.put(json.loads(msg))
            except json.JSONDecodeError:
                self.logger.LOGE("Referee sent invalid message")
                continue

if __name__ == "__main__":
    logger = robot_utilities.Logging()
    client = Referee_cmd_client(logger)
    client.open()
    try:
        while(True):
            print("getting")
            msg = client.get_cmd()
            print(msg)
            time.sleep(1)
    except KeyboardInterrupt:
        client.logger.LOGI("Exiting...")
        client.close()

