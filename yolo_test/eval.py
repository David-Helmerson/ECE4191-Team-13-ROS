import torch
import time
from PIL import Image

img = Image.open('image_0.jpg')


model = torch.hub.load('ultralytics/yolov5', 'custom', path='v5n6best.pt')
with torch.no_grad():
    df = model(img).pandas().xyxy[0]
    t0 = time.time()
    for _ in range(100):
    	_ = model(img)
    print((time.time()-t0)/100)
    arr = df[df['class'] == 0].to_numpy()[:, :-3]
    print(arr)
