import cv2, socket, struct

client = socket.socket()
client.connect(('YOUR_PC_IP', 5000))
cam = cv2.VideoCapture(0)

while True:
    ret, frame = cam.read()
    if not ret:
        break
    _, buffer = cv2.imencode('.jpg', frame)
    data = buffer.tobytes()
    client.sendall(struct.pack(">L", len(data)) + data)
