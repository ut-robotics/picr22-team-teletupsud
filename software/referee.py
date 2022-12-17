# This code is taken from team OWO with permission

import json, time, queue
import multiprocessing as mp    
import websocket as wsc

class Referee_cmd_client:
    def __init__(self):
        self.ip = "192.168.3.220"
        self.port = "8111"
        self.queue = mp.Queue()

    def open(self):
        self.ws = wsc.WebSocket()
        self.connect()
        self.process = mp.Process(target=self.listen, args=())
        self.process.start()

    def close(self):
        self.process.join()
        self.ws.close()

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
        while True:
            try:
                msg = self.ws.recv()
            except wsc.WebSocketConnectionClosedException:
                if self.connect():
                    continue
                else:
                    break
            try:
                self.queue.put(json.loads(msg))
            except json.JSONDecodeError:
                continue

if __name__ == "__main__":
    client = Referee_cmd_client()
    client.open()
    try:
        while(True):
            print("getting")
            msg = client.get_cmd()
            print(msg)
            time.sleep(1)
    except KeyboardInterrupt:
        client.close()