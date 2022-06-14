import socket
import json
import torch
import cv2
import ssl
import numpy as np

from PIL import Image

SVR_ADD = 2007
SVR_QS = 5

CLT_ADD = 2006
CLT_QS = 5

BYTE_STREAM = 1024

MODELS = "/home/carlosahs42/Documents/models"
YOLOV5_PATH = "/home/carlosahs42/Documents/te3002b/yolov5"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), SVR_ADD))
s.listen(SVR_QS)

model = torch.hub.load(
    YOLOV5_PATH, 'custom',
    path=MODELS + "/best_5s.pt", source='local'
)

while True:
    c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    c.connect((socket.gethostname(), CLT_ADD))

    buf = c.recv(BYTE_STREAM)
    img_path = buf.decode("utf-8")

    with open(img_path, "rb") as f:
        img = np.load(f)

    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    results = model(rgb)
    data = results.pandas().xyxy[0].to_dict()

    clientsocket, address = s.accept()
    clientsocket.send(json.dumps(data, indent=2).encode("utf-8"))
    clientsocket.close()

    # cv2.imshow('YOLO', np.squeeze(results.render()))
    # if cv2.waitKey(10) & 0xFF == ord('q'):
    #     cv2.destroyAllWindows()
