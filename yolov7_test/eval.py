import time
import torch
from torchvision import transforms
from PIL import Image
from nms import *

width, height = 640, 480
conf_thres, iou_thres = 0.4, 0.4
lb = ['normal', 'magnet']
device = 'cpu'

model = torch.jit.load('yolov7_marbles_script.pt')
img_to_tensor = transforms.ToTensor()
img = img_to_tensor(Image.open('image_0.jpg')).unsqueeze(0)

print(img.shape)
with torch.no_grad():
    t0 = time.time()
    out, _ = model(img)
    print(out.shape)

    # Run NMS
    out = non_max_suppression(out, conf_thres=conf_thres, iou_thres=iou_thres, labels=None, multi_label=True)[0]
    print(out)
    boxes = torch.round(out[:4, torch.where(out[:, -1] == 0)]).transpose(0, 1).tolist()