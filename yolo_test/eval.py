import torch
from PIL import Image

img = Image.open('image_0.jpg')


model = torch.hub.load('ultralytics/yolov5', 'custom', path='v5best.pt')
with torch.no_grad():
    df = model(img).pandas().xyxy[0]
    arr = df[df['class'] == 0].to_numpy()[:, :-3]
    print(arr)