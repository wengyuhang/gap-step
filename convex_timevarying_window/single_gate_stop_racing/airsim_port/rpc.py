"""Small synchronous MessagePack-RPC transport; no legacy Tornado dependency."""
import socket
import msgpack


class Client:
    def __init__(self, host='127.0.0.1', port=41451, timeout=10.):
        self.socket = socket.create_connection((host, port), timeout)
        self.unpacker = msgpack.Unpacker(raw=False)
        self.serial = 0
        self.responses = {}

    def call(self, method, *args):
        return self.wait(self.send(method, *args))

    def send(self, method, *args):
        self.serial += 1
        serial = self.serial
        self.socket.sendall(msgpack.packb([0, serial, method, list(args)], use_bin_type=True))
        return serial

    def batch(self, requests):
        ids = []
        packets = []
        for method, args in requests:
            self.serial += 1
            ids.append(self.serial)
            packets.append(msgpack.packb([0, self.serial, method, list(args)], use_bin_type=True))
        self.socket.sendall(b''.join(packets))
        return [self.wait(i) for i in ids]

    def wait(self, serial):
        while True:
            if serial in self.responses:
                response = self.responses.pop(serial)
                if response[2] is not None:
                    raise RuntimeError(str(response[2]))
                return response[3]
            for response in self.unpacker:
                if response[0] == 1:
                    self.responses[response[1]] = response
            if serial in self.responses:
                continue
            data = self.socket.recv(65536)
            if not data:
                raise ConnectionError('AirSim closed RPC connection')
            self.unpacker.feed(data)

    def close(self):
        self.socket.close()
