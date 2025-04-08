import socket, struct, cv2, numpy as np

server = socket.socket()
server.bind(('0.0.0.0', 5000))
server.listen(1)
conn, _ = server.accept()

while True:
    size = struct.unpack(">L", conn.recv(4))[0]
    data = b''
    while len(data) < size:
        data += conn.recv(4096)
    img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    # Process with YOLOv8
