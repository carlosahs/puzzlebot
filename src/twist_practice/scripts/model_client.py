import socket
import json
import torch
import cv2
import ssl
import numpy as np
import pandas as pd

from PIL import Image

BYTE_STREAM = 1024
IMG_PATH = "/home/carlosahs42/Documents/imgbin.npy"

MODELS = "/home/carlosahs42/Documents/models"
YOLOV5_PATH = "/home/carlosahs42/Documents/te3002b/yolov5"

model = torch.hub.load(
    YOLOV5_PATH, 'custom',
    path=MODELS + "/best.pt", source='local'
)

# model = torch.hub.load(
#     "ultralytics/yolov5", 'custom',
#     path=MODELS + "/best.pt", force_reload=True
# )

SVR_ADD = 1234
SVR_QS = 5

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), SVR_ADD))
s.listen(SVR_QS)

while True:
    img_read = False
    while not img_read:
        with open(img_path, "rb") as f:
            try:
                img = np.load(f)
                img_read = True
            except ValueError:
                img_read = False

    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = model(rgb)

    data = results.pandas().xyxy[0]
    data['area'] = abs(data['xmin'] - data['xmax']) * abs(data['ymin'] - data['ymax'])
    data.drop(columns=['xmax', 'xmin', 'ymax', 'ymin'])

    data = data.to_dict()

    clientsocket, address = s.accept()
    clientsocket.send(json.dumps(data, indent=2).encode("utf-8"))
    clientsocket.close()

    # cv2.imshow('YOLO', np.squeeze(results.render()))
    # if cv2.waitKey(10) & 0xFF == ord('q'):
    #     cv2.destroyAllWindows()
