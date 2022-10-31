import json, time, queue
import multiprocessing as mp    
import websocket as wsc
import robot_utilities

class Referee_cmd_client:
    def __init__(self, logger):
        self.ip = "localhost"
        self.port = "8222"
        self.logger = logger
        self.queue = mp.Queue()
        self.logger.LOGI("Referee cmd client initialized")

    def open(self):
        self.ws = wsc.WebSocket()
        self.ws.connect("ws://" + self.ip + ":" + self.port)
        self.logger.LOGI("Referee cmd client opened")

    def close(self):
        self.ws.close()
        self.logger.LOGI("Referee cmd client closed")

    def get_cmd(self):
        try:
            return self.queue.get_nowait()
        except queue.Empty:
            return None


    def listen(self):
        self.logger.LOGI("Listening started")
        while True:
            msg = self.ws.recv()
            try:
                self.queue.put(json.loads(msg))
            except json.JSONDecodeError:
                self.logger.LOGE("Referee send invalid message")
                continue


if __name__ == "__main__":
    logger = robot_utilities.Logging()
    client = Referee_cmd_client(logger)
    client.open()
    p = mp.Process(target=client.listen, args=())
    p.start()
    while(True):
        print("getting")
        msg = client.get_cmd()
        print(msg)
        time.sleep(1)

