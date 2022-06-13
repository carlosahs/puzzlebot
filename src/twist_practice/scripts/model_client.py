import socket
import torch
import cv2
import numpy as np

from PIL import Image

SVR_ADD = 1247
SVR_QS = 5

CLT_ADD = 1246
CLT_QS = 5

BYTE_STREAM = 1024

MODELS = "/home/carlosahs42/Documents/models"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), SVR_ADD))
s.listen(SVR_QS)

model = torch.hub.load(
    'ultralytics/yolov5', 'custom',
    path=MODELS + "/best_5s.pt", force_reload=True
)

while True:
    c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    c.connect((socket.gethostname(), CLT_ADD))

    buf = c.recv(BYTE_STREAM)
    img = np.frombuffer(buf, dtype=np.uint8)
    print(img, img.nbytes, img.dtype, img.shape)
    img.reshape((480, 640, 3))

    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    results = model(rgb)
    # results_info = results.pandas()

    pred = 0

    clientsocket, address = s.accept()
    clientsocket.send(bytes(str(pred), "utf-8"))
    clientsocket.close()

    # cv2.imshow('YOLO', np.squeeze(results.render())) 
    # if cv2.waitKey(10) & 0xFF == ord('q'):
    #     cv2.destroyAllWindows()
